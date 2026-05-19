"""Auto-verify lifecycle helpers for the parts catalog (issue #122).

A part starts life as ``draft``. It is promoted to ``verified`` when ALL
of the following are true:

* At least 2 distinct authors (excluding the original ``created_by``)
  have contributed revisions to it.
* At least one supplier link has been health-checked at least once AND
  is currently not flagged as broken (i.e. ``last_checked_at IS NOT
  NULL AND is_broken = FALSE``).
* The part has been in ``draft`` for at least 7 days (prevents a
  same-day creator + buddy from rubber-stamping each other's parts).

A ``verified`` part is demoted to ``disputed`` if a ``part_reports``
table exists (Phase 3, issue #123) and contains an open report whose
``reason`` is ``"wrong"`` or ``"duplicate"``. If the table does not yet
exist on the running database, the demotion check is silently skipped —
this keeps Phase 2 deployable before #123 ships.

Verification is heuristic-only. There is **no manual verify endpoint**
in this PR. Admins who want to flip status by hand poke the column
directly via psql / a one-off task.
"""

from __future__ import annotations

import logging
from datetime import datetime, timedelta, timezone
from typing import Optional

from sqlalchemy import distinct, func, select, text
from sqlalchemy.ext.asyncio import AsyncSession

from .models import Part, PartRevision, PartSupplier

logger = logging.getLogger(__name__)

# The number of distinct *other* authors required for promotion. Two
# means "the creator plus at least two other people have touched it".
_PROMOTION_AUTHOR_THRESHOLD = 2

# Minimum age in draft before a part is eligible for promotion. Stops a
# same-day rubber-stamp from a buddy account.
_PROMOTION_MIN_AGE_DAYS = 7


def _utcnow_naive() -> datetime:
    return datetime.now(timezone.utc).replace(tzinfo=None)


async def _distinct_other_authors(
    session: AsyncSession, part_id: int, original_creator: str
) -> int:
    """Count distinct revision authors EXCLUDING ``original_creator``."""
    return int(
        await session.scalar(
            select(func.count(distinct(PartRevision.author)))
            .where(PartRevision.part_id == part_id)
            .where(PartRevision.author != original_creator)
        )
        or 0
    )


async def _has_healthy_supplier(session: AsyncSession, part_id: int) -> bool:
    """True if at least one supplier has been checked and is not broken."""
    row = await session.scalar(
        select(PartSupplier.id)
        .where(PartSupplier.part_id == part_id)
        .where(PartSupplier.last_checked_at.is_not(None))
        .where(PartSupplier.is_broken.is_(False))
        .limit(1)
    )
    return row is not None


async def _has_open_disputed_report(
    session: AsyncSession, part_id: int
) -> bool:
    """Check the (yet-to-be-built) ``part_reports`` table for open disputes.

    Phase 3 (#123) will introduce a ``part_reports`` table with columns
    ``(part_id, reason, status)``. Until then this returns False so the
    demotion path is a no-op. We probe ``information_schema`` once per
    call — cheap on Postgres, no-op on SQLite (where the dialect check
    short-circuits).
    """
    bind = session.get_bind()
    dialect = getattr(bind, "dialect", None)
    dialect_name = getattr(dialect, "name", "") if dialect else ""
    if dialect_name == "postgresql":
        exists = await session.scalar(
            text(
                "SELECT 1 FROM information_schema.tables "
                "WHERE table_schema = current_schema() "
                "  AND table_name = 'part_reports'"
            )
        )
        if not exists:
            return False
    elif dialect_name == "sqlite":
        # Query sqlite_master inside the existing session transaction
        # rather than opening a fresh connection on the engine — under
        # the test fixture's StaticPool that would deadlock.
        exists = await session.scalar(
            text(
                "SELECT 1 FROM sqlite_master "
                "WHERE type = 'table' AND name = 'part_reports'"
            )
        )
        if not exists:
            return False
    else:
        # Unknown dialect — be conservative and skip the check.
        return False

    row = await session.scalar(
        text(
            "SELECT 1 FROM part_reports "
            "WHERE part_id = :pid "
            "  AND status = 'open' "
            "  AND reason IN ('wrong', 'duplicate') "
            "LIMIT 1"
        ),
        {"pid": part_id},
    )
    return bool(row)


async def compute_part_status(
    session: AsyncSession,
    part: Part,
    *,
    now: Optional[datetime] = None,
    commit: bool = False,
) -> Part:
    """Re-evaluate ``part.status`` against the lifecycle rules.

    Idempotent — safe to call repeatedly. Callers invoke this after any
    write that could move the needle (new revision, new supplier health
    result, new talk post). The caller owns the surrounding transaction;
    we only flush. If ``commit`` is true, we commit before returning —
    that's the path the supplier health-check uses since it doesn't have
    a request transaction to piggyback on.
    """
    when = now or _utcnow_naive()

    # Demotion path first: a verified part with an open dispute drops to
    # ``disputed`` regardless of the other rules. Skipped silently when
    # the part_reports table doesn't exist yet.
    if part.status == "verified":
        if await _has_open_disputed_report(session, part.id):
            part.status = "disputed"
            part.verified_at = None
            await session.flush()
            if commit:
                await session.commit()
            return part

    # Promotion path: only attempt if currently a draft.
    if part.status == "draft":
        age_days = (when - part.created_at).total_seconds() / 86400.0
        if age_days < _PROMOTION_MIN_AGE_DAYS:
            return part
        other_authors = await _distinct_other_authors(
            session, part.id, part.created_by
        )
        if other_authors < _PROMOTION_AUTHOR_THRESHOLD:
            return part
        if not await _has_healthy_supplier(session, part.id):
            return part
        part.status = "verified"
        part.verified_at = when
        part.verified_signals = (part.verified_signals or 0) + 1
        await session.flush()
        if commit:
            await session.commit()
    return part
