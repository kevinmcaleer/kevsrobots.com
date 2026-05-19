"""Mass-deletion detection + auto-disable + rollback (issue #136).

Defends the parts wiki against a logged-in user nuking content in bulk.

Detection thresholds
--------------------
``Trigger B`` — *this* request is itself a mass deletion:
    * removes more than ``DESTRUCTIVE_CONTENT_FRACTION`` (50%) of an
      existing populated part's content (where "populated" means the
      previous combined content length was >= ``POPULATED_MIN_CHARS``);
      OR
    * clears ``DESTRUCTIVE_FIELD_COUNT`` (3) or more structured fields in
      one go (a "structured field" = clears sku / mpn / description_md /
      image_url, OR removes one-or-more tags, OR removes one-or-more
      suppliers).

``Trigger A`` — this user has issued ``DESTRUCTIVE_EDIT_LIMIT`` (5)
destructive edits in the last ``DESTRUCTIVE_WINDOW_MINUTES`` (60). A
"destructive edit" is one that itself meets either of the Trigger-B
sub-conditions. We compare each historical revision to its predecessor.

Rollback semantics
------------------
When triggered, we:

1. Reject the in-progress request with HTTP 403.
2. For every part the user touched in the last hour, restore its content
   to the revision immediately *before* the user's first edit in that
   window. We do this by writing a NEW revision (history is linear — same
   pattern as the public restore endpoint).
3. Set ``users.is_disabled = True`` (lazily inserting the row if needed).
4. Commit it all in a single transaction. If anything raises, the caller
   is expected to ``rollback()`` the session, leaving everything
   unchanged — including the original mutation that fired the heuristic.

Admin exemption
---------------
Users listed in ``settings.admin_usernames_list`` skip the heuristic
entirely.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from typing import Optional

from sqlalchemy import delete, select
from sqlalchemy.ext.asyncio import AsyncSession

from .config import get_settings
from .models import Part, PartRevision, PartSupplier, User

logger = logging.getLogger(__name__)

# --- Tunable thresholds -------------------------------------------------

DESTRUCTIVE_CONTENT_FRACTION = 0.5
"""Removing more than this fraction of a part's prior content length is
considered destructive."""

POPULATED_MIN_CHARS = 200
"""A part must have at least this many chars of combined content before
the content-fraction rule kicks in. Trims false positives on near-empty
draft parts where any edit is mathematically >50%."""

DESTRUCTIVE_FIELD_COUNT = 3
"""Clearing N+ structured fields in one edit counts as destructive."""

DESTRUCTIVE_EDIT_LIMIT = 5
"""User is auto-disabled after this many destructive edits in the
window."""

DESTRUCTIVE_WINDOW_MINUTES = 60
"""Rolling window used by Trigger A and for rollback scope."""


# --- Snapshot helpers ---------------------------------------------------


@dataclass(frozen=True)
class PartSnapshot:
    """Subset of part / revision state used for diffing.

    Works against either a live ``Part`` (+ its current suppliers) or a
    ``PartRevision`` row.
    """

    name: str
    sku: Optional[str]
    mpn: Optional[str]
    description_md: Optional[str]
    image_url: Optional[str]
    tags: tuple[str, ...]
    suppliers: tuple[tuple[Optional[str], str], ...]  # (name, url) pairs

    def content_length(self) -> int:
        """Total characters across the free-form content fields."""
        return sum(
            len(v or "")
            for v in (
                self.name,
                self.sku,
                self.mpn,
                self.description_md,
                self.image_url,
            )
        )


def snapshot_from_revision(rev: PartRevision) -> PartSnapshot:
    suppliers_json = rev.suppliers_json or []
    return PartSnapshot(
        name=rev.name,
        sku=rev.sku,
        mpn=rev.mpn,
        description_md=rev.description_md,
        image_url=rev.image_url,
        tags=tuple(rev.tags or []),
        suppliers=tuple(
            (s.get("name"), s.get("url") or "")
            for s in suppliers_json
            if s.get("url")
        ),
    )


def snapshot_from_part(
    part: Part, suppliers: list[PartSupplier]
) -> PartSnapshot:
    return PartSnapshot(
        name=part.name,
        sku=part.sku,
        mpn=part.mpn,
        description_md=part.description_md,
        image_url=part.image_url,
        tags=tuple(part.tags or []),
        suppliers=tuple(
            (s.supplier_name, s.url) for s in suppliers if s.url
        ),
    )


def snapshot_from_inputs(
    name: str,
    sku: Optional[str],
    mpn: Optional[str],
    description_md: Optional[str],
    image_url: Optional[str],
    tags: list[str],
    suppliers_json: list[dict],
) -> PartSnapshot:
    return PartSnapshot(
        name=name,
        sku=sku,
        mpn=mpn,
        description_md=description_md,
        image_url=image_url,
        tags=tuple(tags or []),
        suppliers=tuple(
            (s.get("name"), s.get("url") or "")
            for s in suppliers_json
            if s.get("url")
        ),
    )


# --- Heuristic ----------------------------------------------------------


def is_destructive(prev: PartSnapshot, nxt: PartSnapshot) -> bool:
    """Does ``nxt`` qualify as a destructive edit of ``prev``?

    True when either:

    * The content-length dropped by more than
      ``DESTRUCTIVE_CONTENT_FRACTION`` AND ``prev`` had at least
      ``POPULATED_MIN_CHARS`` of content to begin with.
    * The edit clears ``DESTRUCTIVE_FIELD_COUNT`` or more structured
      fields (description / image / sku / mpn / tags / suppliers).
    """
    prev_len = prev.content_length()
    nxt_len = nxt.content_length()
    if prev_len >= POPULATED_MIN_CHARS:
        drop = prev_len - nxt_len
        if drop > 0 and drop / prev_len > DESTRUCTIVE_CONTENT_FRACTION:
            return True

    cleared = 0

    def _cleared_value(prev_val: Optional[str], nxt_val: Optional[str]) -> bool:
        return bool((prev_val or "").strip()) and not (nxt_val or "").strip()

    if _cleared_value(prev.description_md, nxt.description_md):
        cleared += 1
    if _cleared_value(prev.image_url, nxt.image_url):
        cleared += 1
    if _cleared_value(prev.sku, nxt.sku):
        cleared += 1
    if _cleared_value(prev.mpn, nxt.mpn):
        cleared += 1
    if prev.tags and len(nxt.tags) < len(prev.tags):
        cleared += 1
    if prev.suppliers and len(nxt.suppliers) < len(prev.suppliers):
        cleared += 1

    return cleared >= DESTRUCTIVE_FIELD_COUNT


# --- Detection + rollback ----------------------------------------------


def is_admin(username: str) -> bool:
    settings = get_settings()
    return username in settings.admin_usernames_list


async def _previous_revision(
    session: AsyncSession, part_id: int, before_rev_id: int
) -> Optional[PartRevision]:
    """Newest revision for ``part_id`` whose id is < ``before_rev_id``."""
    return await session.scalar(
        select(PartRevision)
        .where(PartRevision.part_id == part_id)
        .where(PartRevision.id < before_rev_id)
        .order_by(PartRevision.id.desc())
        .limit(1)
    )


async def _user_destructive_count_in_window(
    session: AsyncSession, username: str
) -> int:
    """Count how many of ``username``'s recent revisions are destructive
    vs. their immediate predecessor."""
    cutoff = datetime.now(timezone.utc).replace(tzinfo=None) - timedelta(
        minutes=DESTRUCTIVE_WINDOW_MINUTES
    )
    revs = (
        await session.scalars(
            select(PartRevision)
            .where(PartRevision.author == username)
            .where(PartRevision.created_at >= cutoff)
            .order_by(PartRevision.created_at.asc(), PartRevision.id.asc())
        )
    ).all()

    count = 0
    for rev in revs:
        prev = await _previous_revision(session, rev.part_id, rev.id)
        if prev is None:
            # First-ever revision for this part — it's the "Initial draft",
            # cannot be destructive by definition.
            continue
        if is_destructive(snapshot_from_revision(prev), snapshot_from_revision(rev)):
            count += 1
    return count


async def _disable_user(
    session: AsyncSession, username: str, reason: str
) -> None:
    """Set ``is_disabled=True`` for ``username``, inserting the row if
    needed. Idempotent."""
    user = await session.scalar(select(User).where(User.username == username))
    now = datetime.now(timezone.utc).replace(tzinfo=None)
    if user is None:
        session.add(
            User(
                username=username,
                is_disabled=True,
                disabled_reason=reason,
                disabled_at=now,
            )
        )
    else:
        user.is_disabled = True
        user.disabled_reason = reason
        user.disabled_at = now


async def _rollback_user_recent_edits(
    session: AsyncSession, username: str, system_summary: str
) -> int:
    """For every part this user edited in the destructive window, write
    one new revision restoring it to the state *immediately before* the
    user's first edit in that window. Returns the number of parts
    restored."""
    cutoff = datetime.now(timezone.utc).replace(tzinfo=None) - timedelta(
        minutes=DESTRUCTIVE_WINDOW_MINUTES
    )
    user_revs = (
        await session.scalars(
            select(PartRevision)
            .where(PartRevision.author == username)
            .where(PartRevision.created_at >= cutoff)
            .order_by(PartRevision.created_at.asc(), PartRevision.id.asc())
        )
    ).all()

    # Map part_id -> earliest user revision in window.
    first_by_part: dict[int, PartRevision] = {}
    for rev in user_revs:
        first_by_part.setdefault(rev.part_id, rev)

    restored_count = 0
    now = datetime.now(timezone.utc).replace(tzinfo=None)
    for part_id, first_rev in first_by_part.items():
        prev = await _previous_revision(session, part_id, first_rev.id)
        if prev is None:
            # The user's first edit in the window was also the part's first
            # revision. We can't roll back to a "pre-user" state because no
            # such state exists — skip.
            logger.warning(
                "Cannot roll back part %s for user %s: no prior revision",
                part_id, username,
            )
            continue

        part = await session.get(Part, part_id)
        if part is None:
            continue

        # Write a non-destructive restore revision (same pattern as
        # /api/parts/{slug}/revisions/{rev_id}/restore).
        suppliers_json = list(prev.suppliers_json or [])
        new_rev = PartRevision(
            part_id=part_id,
            author="system",
            change_summary=system_summary,
            name=prev.name,
            sku=prev.sku,
            mpn=prev.mpn,
            description_md=prev.description_md,
            image_url=prev.image_url,
            tags=list(prev.tags or []),
            suppliers_json=suppliers_json,
            created_at=now,
        )
        session.add(new_rev)
        await session.flush()

        part.name = prev.name
        part.sku = prev.sku
        part.mpn = prev.mpn
        part.description_md = prev.description_md
        part.image_url = prev.image_url
        part.tags = list(prev.tags or [])
        part.current_revision_id = new_rev.id
        part.updated_at = now

        await session.execute(
            delete(PartSupplier).where(PartSupplier.part_id == part_id)
        )
        for entry in suppliers_json:
            if not entry.get("url"):
                continue
            session.add(
                PartSupplier(
                    part_id=part_id,
                    supplier_name=entry.get("name"),
                    url=entry.get("url"),
                )
            )

        restored_count += 1
    return restored_count


@dataclass
class MassDeletionResult:
    triggered: bool
    reason: str = ""
    restored_count: int = 0


async def check_and_handle_mass_deletion(
    session: AsyncSession,
    username: str,
    prev_snapshot: PartSnapshot,
    next_snapshot: PartSnapshot,
) -> MassDeletionResult:
    """Decide whether this proposed edit should be blocked.

    Run BEFORE applying the edit. If ``triggered`` is True the caller
    must NOT commit its own changes — it must roll back the session and
    re-commit only the side-effects produced by this function (rollback
    revisions + user disable), which have already been added to the
    session.

    Admins are exempt — returns ``triggered=False`` immediately.
    """
    if is_admin(username):
        return MassDeletionResult(triggered=False)

    trigger_b = is_destructive(prev_snapshot, next_snapshot)
    trigger_a = False
    if not trigger_b:
        # No need to do the (more expensive) historical scan if Trigger B
        # already fired.
        historical = await _user_destructive_count_in_window(session, username)
        # The current request is itself a candidate destructive edit only
        # if `is_destructive(prev, next)` would be True — already covered
        # by trigger_b above. So Trigger A purely checks history.
        if historical >= DESTRUCTIVE_EDIT_LIMIT:
            trigger_a = True

    if not (trigger_a or trigger_b):
        return MassDeletionResult(triggered=False)

    reason = "Trigger B (single mass-delete)" if trigger_b else "Trigger A (rate)"
    now = datetime.now(timezone.utc).replace(tzinfo=None)
    disabled_reason = (
        f"Auto-disabled: mass deletion detected at {now.isoformat()}Z "
        f"({reason})"
    )
    logger.warning(
        "Mass-deletion auto-disable for %s — %s", username, disabled_reason,
    )

    restored_count = await _rollback_user_recent_edits(
        session,
        username,
        system_summary=f"Auto-rollback: mass deletion by {username}",
    )
    await _disable_user(session, username, disabled_reason)

    return MassDeletionResult(
        triggered=True,
        reason=disabled_reason,
        restored_count=restored_count,
    )
