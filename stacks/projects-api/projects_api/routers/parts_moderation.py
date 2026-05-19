"""Parts moderation: reports + community merge proposals (issue #123).

Sits beside the main ``parts.py`` router as a separate module because the
moderation surface is wide enough (8 endpoints + a non-trivial auto-merge
helper) that bolting it onto ``parts.py`` would push that file past 1k
lines. The choice is purely cosmetic — both modules share the same
``/api/parts`` prefix from the caller's perspective.

Two sub-features:

* **Reports** — anyone logged-in can flag a part with reason
  ``spam | wrong | duplicate | other``. No 14-day account-age gate
  applies here: noise from new accounts is cheaper than missing spam
  they're trying to flag. One open report per (user, part) is enforced
  in :func:`report_part`. Admins resolve from
  ``GET /api/admin/parts/reports`` and ``PATCH …/reports/{id}``.

* **Merge proposals** — anyone with a 14-day-old account can propose
  merging ``source`` into ``target``. Anyone with a 14-day-old account
  can vote ``approve`` or ``reject``; voters can change their mind by
  re-POSTing (the unique constraint on ``(proposal_id, voter_username)``
  upserts).

Auto-merge rule (see :func:`_check_merge_threshold`)
----------------------------------------------------
A proposal triggers ``outcome=merged`` automatically when ALL of:

1. ``approves >= 5``
2. ``approves >= 2 * rejects`` (lopsided in favour)
3. ``now - created_at >= 48h`` — gives the community a window to
   express dissent before the vote auto-resolves.

When that fires, :func:`_perform_merge` does the actual rewiring:

* Every ``project_bom_items.part_id == source.id`` is repointed to
  ``target.id`` and the two parts' ``usage_count`` is recomputed.
* ``part_relations`` rows on either side of the source are rewritten
  to the target (skipping any that would self-link or collide with an
  existing relation on the target).
* ``part_aliases`` rows are reparented to the target. The source's slug
  and name are also added as aliases so future searches still find the
  merged part.
* ``part_suppliers`` rows are reparented (no dedup — duplicate URLs are
  rare and harmless).
* ``part_talk_threads`` rows are reparented **only if the table exists**
  on this DB (issue #122 may not have landed yet on every deploy).
* The source ``Part`` is then deleted; the FK on bom rows is already
  pointing at the target so the cascade is clean.

The 48h gate uses ``created_at`` rather than the first-vote timestamp
because the proposer themselves is the implicit first approve in spirit
— we want the door open for objections, not just for affirmations to
pile up quickly.

Cross-issue safety
------------------
This file references tables that may or may not exist on the live DB
yet (e.g. ``part_talk_threads`` from issue #122 lands on a parallel
branch). Every such read is wrapped in :func:`_table_exists` so a
Postgres deploy that doesn't have the table yet doesn't 500 here.
"""

from __future__ import annotations

import logging
from datetime import datetime, timedelta, timezone
from typing import Optional

from fastapi import APIRouter, Depends, HTTPException, Query, status
from sqlalchemy import and_, delete, func, or_, select, text, update
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import (
    get_current_user,
    require_admin,
    require_terms_accepted_aged,
)
from ..db import get_session
from ..models import (
    Part,
    PartAlias,
    PartMergeProposal,
    PartMergeVote,
    PartRelation,
    PartReport,
    PartSupplier,
    ProjectBOMItem,
)
from ..schemas import (
    PART_MERGE_VOTES,
    PART_REPORT_REASONS,
    PART_REPORT_RESOLUTIONS,
    PartMergePartRef,
    PartMergeProposalCreate,
    PartMergeProposalListResponse,
    PartMergeProposalResponse,
    PartMergeVoteCreate,
    PartMergeVoteResponse,
    PartReportCreate,
    PartReportListResponse,
    PartReportPartRef,
    PartReportResolve,
    PartReportResponse,
)

logger = logging.getLogger(__name__)

router = APIRouter(tags=["parts-moderation"])


# --- helpers -----------------------------------------------------------


MERGE_AUTO_APPROVES = 5  # absolute floor of approve votes
MERGE_AUTO_RATIO = 2  # approves must be >= ratio * rejects
MERGE_AUTO_MIN_AGE = timedelta(hours=48)


def _now_naive_utc() -> datetime:
    return datetime.now(timezone.utc).replace(tzinfo=None)


async def _get_part_by_slug(session: AsyncSession, slug: str) -> Part:
    part = await session.scalar(select(Part).where(Part.slug == slug))
    if part is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Part not found")
    return part


def _part_ref(part: Part) -> PartMergePartRef:
    return PartMergePartRef(
        id=part.id, slug=part.slug, name=part.name, status=part.status
    )


def _report_part_ref(part: Part) -> PartReportPartRef:
    return PartReportPartRef(
        id=part.id, slug=part.slug, name=part.name, status=part.status
    )


async def _table_exists(session: AsyncSession, table_name: str) -> bool:
    """Cross-dialect table-existence probe.

    Used to guard reads against tables that come from other in-flight
    issues (e.g. issue #122's ``part_talk_threads``) — they may not be
    on the live DB yet. Postgres uses ``information_schema``; SQLite
    uses ``sqlite_master``. On any dialect we don't know, return False
    and let the caller skip that step.
    """
    bind = session.get_bind()
    dialect = bind.dialect.name if bind is not None else ""
    try:
        if dialect == "postgresql":
            row = await session.execute(
                text(
                    "SELECT 1 FROM information_schema.tables "
                    "WHERE table_schema = current_schema() AND table_name = :t"
                ),
                {"t": table_name},
            )
        else:
            # SQLite (used by tests) — sqlite_master always exists.
            row = await session.execute(
                text("SELECT 1 FROM sqlite_master WHERE type='table' AND name=:t"),
                {"t": table_name},
            )
        return row.first() is not None
    except Exception as exc:  # pragma: no cover — defensive
        logger.warning("table_exists probe failed for %s: %s", table_name, exc)
        return False


async def _count_votes(
    session: AsyncSession, proposal_id: int
) -> tuple[int, int]:
    """Return ``(approves, rejects)`` for a proposal."""
    rows = (
        await session.execute(
            select(PartMergeVote.vote, func.count(PartMergeVote.id))
            .where(PartMergeVote.proposal_id == proposal_id)
            .group_by(PartMergeVote.vote)
        )
    ).all()
    approves = 0
    rejects = 0
    for vote_kind, n in rows:
        if vote_kind == "approve":
            approves = int(n)
        elif vote_kind == "reject":
            rejects = int(n)
    return approves, rejects


async def _proposal_response(
    session: AsyncSession, proposal: PartMergeProposal
) -> PartMergeProposalResponse:
    source = await session.get(Part, proposal.source_part_id)
    target = await session.get(Part, proposal.target_part_id)
    approves, rejects = await _count_votes(session, proposal.id)
    # If a side of the merge has been deleted (e.g. after an admin force-
    # delete), surface a synthetic ref so the UI doesn't crash. Real
    # source/target rows are the common case.
    if source is None:
        source_ref = PartMergePartRef(
            id=proposal.source_part_id, slug="", name="(deleted)", status="merged"
        )
    else:
        source_ref = _part_ref(source)
    if target is None:
        target_ref = PartMergePartRef(
            id=proposal.target_part_id, slug="", name="(deleted)", status="merged"
        )
    else:
        target_ref = _part_ref(target)
    return PartMergeProposalResponse(
        id=proposal.id,
        proposer_username=proposal.proposer_username,
        rationale=proposal.rationale,
        created_at=proposal.created_at,
        resolved_at=proposal.resolved_at,
        outcome=proposal.outcome,
        source=source_ref,
        target=target_ref,
        approves=approves,
        rejects=rejects,
    )


# --- Reports -----------------------------------------------------------


@router.post(
    "/api/parts/{slug}/report",
    response_model=PartReportResponse,
    status_code=201,
)
async def report_part(
    slug: str,
    body: PartReportCreate,
    user: str = Depends(get_current_user),
    session: AsyncSession = Depends(get_session),
) -> PartReportResponse:
    """Flag a part. No 14-day account-age gate here (see module docstring)."""
    part = await _get_part_by_slug(session, slug)
    if body.reason not in PART_REPORT_REASONS:
        raise HTTPException(
            status.HTTP_400_BAD_REQUEST,
            detail=f"Reason must be one of {PART_REPORT_REASONS}",
        )

    # One open report per user-per-part. Re-opening after an admin
    # resolved a prior report is fine — only ``resolved_at IS NULL``
    # blocks the new write.
    existing = await session.scalar(
        select(PartReport.id).where(
            PartReport.part_id == part.id,
            PartReport.reporter_username == user,
            PartReport.resolved_at.is_(None),
        )
    )
    if existing is not None:
        raise HTTPException(
            status.HTTP_409_CONFLICT,
            detail="You already have an open report on this part",
        )

    note = (body.note or "").strip() or None
    report = PartReport(
        part_id=part.id,
        reporter_username=user,
        reason=body.reason,
        note=note,
        created_at=_now_naive_utc(),
    )
    session.add(report)
    await session.commit()
    await session.refresh(report)
    return PartReportResponse(
        id=report.id,
        part_id=report.part_id,
        reporter_username=report.reporter_username,
        reason=report.reason,
        note=report.note,
        created_at=report.created_at,
        resolved_at=report.resolved_at,
        resolved_by=report.resolved_by,
        resolution=report.resolution,
    )


@router.get(
    "/api/admin/parts/reports",
    response_model=PartReportListResponse,
)
async def list_part_reports(
    status_filter: str = Query(default="open", alias="status", pattern=r"^(open|resolved)$"),
    limit: int = Query(default=50, ge=1, le=200),
    offset: int = Query(default=0, ge=0),
    admin: str = Depends(require_admin),
    session: AsyncSession = Depends(get_session),
) -> PartReportListResponse:
    """Admin queue: open reports (default) or already-resolved history."""
    base = select(PartReport)
    count_q = select(func.count(PartReport.id))
    if status_filter == "open":
        base = base.where(PartReport.resolved_at.is_(None))
        count_q = count_q.where(PartReport.resolved_at.is_(None))
    else:
        base = base.where(PartReport.resolved_at.is_not(None))
        count_q = count_q.where(PartReport.resolved_at.is_not(None))

    total = int((await session.execute(count_q)).scalar_one() or 0)
    base = (
        base.order_by(PartReport.created_at.desc(), PartReport.id.desc())
        .limit(limit)
        .offset(offset)
    )
    rows = (await session.scalars(base)).all()

    # Bulk-fetch the parts so we don't N+1 the queue render.
    part_ids = sorted({r.part_id for r in rows})
    parts_by_id: dict[int, Part] = {}
    if part_ids:
        parts = (
            await session.scalars(select(Part).where(Part.id.in_(part_ids)))
        ).all()
        parts_by_id = {p.id: p for p in parts}

    items: list[PartReportResponse] = []
    for r in rows:
        part = parts_by_id.get(r.part_id)
        items.append(
            PartReportResponse(
                id=r.id,
                part_id=r.part_id,
                reporter_username=r.reporter_username,
                reason=r.reason,
                note=r.note,
                created_at=r.created_at,
                resolved_at=r.resolved_at,
                resolved_by=r.resolved_by,
                resolution=r.resolution,
                part=_report_part_ref(part) if part is not None else None,
            )
        )
    return PartReportListResponse(items=items, total=total)


@router.patch(
    "/api/admin/parts/reports/{report_id}",
    response_model=PartReportResponse,
)
async def resolve_part_report(
    report_id: int,
    body: PartReportResolve,
    admin: str = Depends(require_admin),
    session: AsyncSession = Depends(get_session),
) -> PartReportResponse:
    """Admin: accept or dismiss a report.

    Idempotent on already-resolved rows: re-PATCHing flips the resolution
    in place but keeps the original ``resolved_at`` / ``resolved_by`` so
    we don't lose the audit trail. (In practice the UI only offers this
    action on open reports.)
    """
    report = await session.get(PartReport, report_id)
    if report is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Report not found")
    if body.resolution not in PART_REPORT_RESOLUTIONS:
        raise HTTPException(
            status.HTTP_400_BAD_REQUEST,
            detail=f"resolution must be one of {PART_REPORT_RESOLUTIONS}",
        )

    if report.resolved_at is None:
        report.resolved_at = _now_naive_utc()
        report.resolved_by = admin
    report.resolution = body.resolution

    await session.commit()
    await session.refresh(report)

    part = await session.get(Part, report.part_id)
    return PartReportResponse(
        id=report.id,
        part_id=report.part_id,
        reporter_username=report.reporter_username,
        reason=report.reason,
        note=report.note,
        created_at=report.created_at,
        resolved_at=report.resolved_at,
        resolved_by=report.resolved_by,
        resolution=report.resolution,
        part=_report_part_ref(part) if part is not None else None,
    )


# --- Merge proposals ---------------------------------------------------


@router.post(
    "/api/parts/{slug}/merge-proposal",
    response_model=PartMergeProposalResponse,
    status_code=201,
)
async def propose_merge(
    slug: str,
    body: PartMergeProposalCreate,
    user: str = Depends(require_terms_accepted_aged),
    session: AsyncSession = Depends(get_session),
) -> PartMergeProposalResponse:
    """Propose merging the part identified by URL slug INTO ``target_slug``.

    14-day account-age gate + T&Cs gate (via :func:`require_terms_accepted_aged`). The
    source part is identified by the URL slug, target by the body field.
    """
    source = await _get_part_by_slug(session, slug)
    target_slug = (body.target_slug or "").strip()
    if not target_slug:
        raise HTTPException(
            status.HTTP_400_BAD_REQUEST, detail="target_slug is required"
        )
    if target_slug == source.slug:
        raise HTTPException(
            status.HTTP_400_BAD_REQUEST,
            detail="Cannot merge a part into itself",
        )
    target = await session.scalar(select(Part).where(Part.slug == target_slug))
    if target is None:
        raise HTTPException(
            status.HTTP_404_NOT_FOUND, detail="Target part not found"
        )
    if target.id == source.id:
        # Defensive — same as the slug check above, in case of slug aliasing.
        raise HTTPException(
            status.HTTP_400_BAD_REQUEST,
            detail="Cannot merge a part into itself",
        )

    # One open proposal per (source, target) ordered pair.
    existing = await session.scalar(
        select(PartMergeProposal.id).where(
            PartMergeProposal.source_part_id == source.id,
            PartMergeProposal.target_part_id == target.id,
            PartMergeProposal.resolved_at.is_(None),
        )
    )
    if existing is not None:
        raise HTTPException(
            status.HTTP_409_CONFLICT,
            detail="An open merge proposal already exists for this pair",
        )

    proposal = PartMergeProposal(
        source_part_id=source.id,
        target_part_id=target.id,
        proposer_username=user,
        rationale=body.rationale.strip(),
        created_at=_now_naive_utc(),
    )
    session.add(proposal)
    await session.commit()
    await session.refresh(proposal)
    return await _proposal_response(session, proposal)


@router.get(
    "/api/parts/merge-proposals",
    response_model=PartMergeProposalListResponse,
)
async def list_merge_proposals(
    status_filter: str = Query(
        default="open", alias="status", pattern=r"^(open|resolved)$"
    ),
    limit: int = Query(default=50, ge=1, le=200),
    offset: int = Query(default=0, ge=0),
    session: AsyncSession = Depends(get_session),
) -> PartMergeProposalListResponse:
    """Public list. ``status=open`` (default) shows unresolved proposals
    newest-first; ``status=resolved`` shows merged / rejected / withdrawn."""
    base = select(PartMergeProposal)
    count_q = select(func.count(PartMergeProposal.id))
    if status_filter == "open":
        base = base.where(PartMergeProposal.resolved_at.is_(None))
        count_q = count_q.where(PartMergeProposal.resolved_at.is_(None))
    else:
        base = base.where(PartMergeProposal.resolved_at.is_not(None))
        count_q = count_q.where(PartMergeProposal.resolved_at.is_not(None))
    total = int((await session.execute(count_q)).scalar_one() or 0)

    base = (
        base.order_by(
            PartMergeProposal.created_at.desc(), PartMergeProposal.id.desc()
        )
        .limit(limit)
        .offset(offset)
    )
    proposals = (await session.scalars(base)).all()
    items: list[PartMergeProposalResponse] = []
    for p in proposals:
        items.append(await _proposal_response(session, p))
    return PartMergeProposalListResponse(items=items, total=total)


@router.get(
    "/api/parts/merge-proposals/{proposal_id}",
    response_model=PartMergeProposalResponse,
)
async def get_merge_proposal(
    proposal_id: int,
    session: AsyncSession = Depends(get_session),
) -> PartMergeProposalResponse:
    proposal = await session.get(PartMergeProposal, proposal_id)
    if proposal is None:
        raise HTTPException(
            status.HTTP_404_NOT_FOUND, detail="Proposal not found"
        )
    return await _proposal_response(session, proposal)


@router.post(
    "/api/parts/merge-proposals/{proposal_id}/vote",
    response_model=PartMergeVoteResponse,
)
async def vote_on_merge_proposal(
    proposal_id: int,
    body: PartMergeVoteCreate,
    user: str = Depends(require_terms_accepted_aged),
    session: AsyncSession = Depends(get_session),
) -> PartMergeVoteResponse:
    """Approve or reject — re-POSTing flips the existing vote in place.

    The 14-day account-age gate applies. The proposer is allowed to vote
    on their own proposal (it's the only way for them to formally
    register an approval; the proposal itself isn't auto-counted as one).
    """
    proposal = await session.get(PartMergeProposal, proposal_id)
    if proposal is None:
        raise HTTPException(
            status.HTTP_404_NOT_FOUND, detail="Proposal not found"
        )
    if proposal.resolved_at is not None:
        raise HTTPException(
            status.HTTP_409_CONFLICT,
            detail="Proposal already resolved — voting is closed",
        )
    if body.vote not in PART_MERGE_VOTES:
        raise HTTPException(
            status.HTTP_400_BAD_REQUEST,
            detail=f"vote must be one of {PART_MERGE_VOTES}",
        )

    existing = await session.scalar(
        select(PartMergeVote).where(
            PartMergeVote.proposal_id == proposal.id,
            PartMergeVote.voter_username == user,
        )
    )
    now = _now_naive_utc()
    if existing is None:
        session.add(
            PartMergeVote(
                proposal_id=proposal.id,
                voter_username=user,
                vote=body.vote,
                voted_at=now,
            )
        )
    else:
        existing.vote = body.vote
        existing.voted_at = now

    await session.commit()
    await session.refresh(proposal)

    # Check threshold AFTER the vote landed.
    triggered = await _check_merge_threshold(session, proposal)
    approves, rejects = await _count_votes(session, proposal.id)
    return PartMergeVoteResponse(
        proposal_id=proposal.id,
        voter_username=user,
        vote=body.vote,
        approves=approves,
        rejects=rejects,
        outcome=proposal.outcome,
        resolved_at=proposal.resolved_at,
    )


@router.delete(
    "/api/parts/merge-proposals/{proposal_id}",
    status_code=204,
)
async def withdraw_merge_proposal(
    proposal_id: int,
    user: str = Depends(get_current_user),
    session: AsyncSession = Depends(get_session),
) -> None:
    """Withdraw a proposal. Allowed for the proposer OR any admin.

    Idempotent on already-resolved proposals: returns 204 even if there
    was nothing to withdraw, so the UI's "I changed my mind" button is
    safe to double-click.
    """
    from ..config import get_settings

    proposal = await session.get(PartMergeProposal, proposal_id)
    if proposal is None:
        raise HTTPException(
            status.HTTP_404_NOT_FOUND, detail="Proposal not found"
        )

    if proposal.resolved_at is not None:
        return None

    settings = get_settings()
    is_admin = user in settings.admin_usernames_list
    if proposal.proposer_username != user and not is_admin:
        raise HTTPException(
            status.HTTP_403_FORBIDDEN,
            detail="Only the proposer or an admin can withdraw a proposal",
        )

    proposal.outcome = "withdrawn"
    proposal.resolved_at = _now_naive_utc()
    await session.commit()
    return None


# --- Auto-merge ---------------------------------------------------------


async def _check_merge_threshold(
    session: AsyncSession, proposal: PartMergeProposal
) -> bool:
    """Evaluate the auto-merge rule. Returns True if the merge fired.

    Rule (documented at the top of the module):

        approves >= 5
        AND approves >= 2 * rejects
        AND now - created_at >= 48h
    """
    if proposal.resolved_at is not None:
        return False
    approves, rejects = await _count_votes(session, proposal.id)
    if approves < MERGE_AUTO_APPROVES:
        return False
    if approves < MERGE_AUTO_RATIO * rejects:
        return False
    age = _now_naive_utc() - (
        proposal.created_at if proposal.created_at is not None else _now_naive_utc()
    )
    if age < MERGE_AUTO_MIN_AGE:
        return False
    await _perform_merge(session, proposal)
    return True


async def _perform_merge(
    session: AsyncSession, proposal: PartMergeProposal
) -> None:
    """Run the actual merge: reparent everything, then delete the source.

    Order matters: we reparent the foreign-keyed children FIRST (BOM
    rows, relations, aliases, suppliers, talk threads) and only then
    delete the source ``Part``. The BOM rows use ``ON DELETE SET NULL``
    so technically a leftover would survive a source-part delete, but
    rewiring beforehand keeps the BOM rows pointing at the correct
    target (which is the whole point of the merge).
    """
    source_id = proposal.source_part_id
    target_id = proposal.target_part_id

    # Defensive: refuse to merge if either side has vanished.
    source = await session.get(Part, source_id)
    target = await session.get(Part, target_id)
    if source is None or target is None:
        proposal.outcome = "rejected"
        proposal.resolved_at = _now_naive_utc()
        await session.commit()
        return

    # 1. BOM rows: repoint part_id and refresh usage counts.
    await session.execute(
        update(ProjectBOMItem)
        .where(ProjectBOMItem.part_id == source_id)
        .values(part_id=target_id)
    )

    # 2. Part relations: rewrite both sides. Skip rows that would self-
    # link to the target after the rewrite, or that would collide with
    # an existing relation on the target.
    existing_target_rels: set[tuple[int, int]] = set()
    rel_rows = (
        await session.scalars(
            select(PartRelation).where(
                or_(
                    PartRelation.part_id == target_id,
                    PartRelation.related_id == target_id,
                )
            )
        )
    ).all()
    for r in rel_rows:
        a, b = (r.part_id, r.related_id) if r.part_id < r.related_id else (r.related_id, r.part_id)
        existing_target_rels.add((a, b))

    source_rels = (
        await session.scalars(
            select(PartRelation).where(
                or_(
                    PartRelation.part_id == source_id,
                    PartRelation.related_id == source_id,
                )
            )
        )
    ).all()
    for rel in source_rels:
        other = rel.related_id if rel.part_id == source_id else rel.part_id
        if other == target_id:
            # Would self-link the target after the rewrite — drop.
            await session.delete(rel)
            continue
        new_a, new_b = (target_id, other) if target_id < other else (other, target_id)
        if (new_a, new_b) in existing_target_rels:
            await session.delete(rel)
            continue
        rel.part_id = new_a
        rel.related_id = new_b
        existing_target_rels.add((new_a, new_b))

    # 3. Aliases: reparent + add the source's slug + name as aliases so
    # search still finds the merged result.
    await session.execute(
        update(PartAlias)
        .where(PartAlias.part_id == source_id)
        .values(part_id=target_id)
    )
    for alias_str in {source.slug, source.name}:
        alias_str = (alias_str or "").strip()
        if not alias_str:
            continue
        already = await session.scalar(
            select(PartAlias.id).where(
                PartAlias.part_id == target_id,
                func.lower(PartAlias.alias) == alias_str.lower(),
            )
        )
        if already is None:
            session.add(PartAlias(part_id=target_id, alias=alias_str))

    # 4. Suppliers — straight reparent.
    await session.execute(
        update(PartSupplier)
        .where(PartSupplier.part_id == source_id)
        .values(part_id=target_id)
    )

    # 5. Talk threads (issue #122, may not exist yet).
    if await _table_exists(session, "part_talk_threads"):
        try:
            await session.execute(
                text(
                    'UPDATE part_talk_threads SET part_id = :target '
                    'WHERE part_id = :source AND (closed IS NULL OR closed = :is_open)'
                ),
                {"target": target_id, "source": source_id, "is_open": False},
            )
        except Exception as exc:  # pragma: no cover — column shape may vary
            logger.warning("Reparent of talk threads failed: %s", exc)

    # 6. Recompute usage counts.
    target_count = await session.scalar(
        select(func.count(func.distinct(ProjectBOMItem.project_id))).where(
            ProjectBOMItem.part_id == target_id
        )
    )
    target.usage_count = int(target_count or 0)

    # 7. Delete the source — child rows have already moved.
    await session.delete(source)

    proposal.outcome = "merged"
    proposal.resolved_at = _now_naive_utc()
    await session.commit()
