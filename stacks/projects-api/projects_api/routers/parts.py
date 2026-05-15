"""Parts catalog endpoints (issue #121, Phase 1 MVP).

This router implements:

* ``GET    /api/parts``                 — fuzzy search across name + aliases
                                          + sku + mpn.
* ``POST   /api/parts``                 — create a draft part (14-day gated).
* ``GET    /api/parts/{slug}``          — full part incl. suppliers + last
                                          10 revisions.
* ``PUT    /api/parts/{slug}``          — write a new revision (14-day
                                          gated). Rejects if no field
                                          actually changed.
* ``GET    /api/parts/{slug}/revisions``— paginated revision list.
* ``POST   /api/parts/{slug}/revisions/{rev_id}/restore`` — non-destructive
                                          restore (writes a new revision
                                          whose snapshot equals rev_id).

Account-age gate
----------------
Mutating endpoints depend on :func:`get_current_user_aged` which calls
:func:`fetch_account_created_at` against Chatter's ``/api/me``. If Chatter
is unreachable or omits the ``account_created_at`` field, the gate fails
closed (HTTP 403 "Account age cannot be verified"). Tests monkeypatch
``fetch_account_created_at`` directly.

Usage-count semantics
---------------------
``parts.usage_count`` is a denormalised cache of *distinct projects* that
reference the part. It's maintained transactionally in
``routers/bom.py`` as BOM rows are inserted / updated / deleted. If it
ever drifts (e.g. due to a manual DB poke), a Phase-2 cron will recompute
it from ``SELECT COUNT(DISTINCT project_id) FROM project_bom_items WHERE
part_id=?``.

Search v1
---------
We use ``lower()``-substring matching against ``parts.name``,
``parts.sku``, ``parts.mpn`` and ``part_aliases.alias``. This is fine for
the small corpus we'll have in Phase 1. When the catalog grows we'll swap
this for ``pg_trgm`` (or a dedicated search index) without changing the
API contract.
"""

from __future__ import annotations

import re
from datetime import datetime, timezone
from typing import Optional

from fastapi import APIRouter, Depends, HTTPException, Query, status
from sqlalchemy import delete, func, or_, select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import get_current_user_aged
from ..db import get_session
from ..models import Part, PartAlias, PartRevision, PartSupplier
from ..schemas import (
    PartCreate,
    PartDetail,
    PartRevisionDetail,
    PartRevisionSummary,
    PartSearchResult,
    PartSupplierInput,
    PartSupplierResponse,
    PartUpdate,
)

router = APIRouter(prefix="/api/parts", tags=["parts"])


# --- helpers ------------------------------------------------------------


_SLUG_BAD_CHARS = re.compile(r"[^a-z0-9]+")


def _slugify(name: str) -> str:
    base = _SLUG_BAD_CHARS.sub("-", name.lower()).strip("-")
    # Cap at 80 chars; the DB column is 120 so disambig suffixes have room.
    return (base[:80] or "part").rstrip("-")


async def _unique_slug(session: AsyncSession, base: str) -> str:
    """Append -2, -3, ... until the slug is unique."""
    candidate = base
    suffix = 2
    while True:
        existing = await session.scalar(select(Part.id).where(Part.slug == candidate))
        if existing is None:
            return candidate
        candidate = f"{base}-{suffix}"
        suffix += 1


def _now_naive_utc() -> datetime:
    return datetime.now(timezone.utc).replace(tzinfo=None)


async def _load_suppliers(session: AsyncSession, part_id: int) -> list[PartSupplier]:
    return list(
        (
            await session.scalars(
                select(PartSupplier)
                .where(PartSupplier.part_id == part_id)
                .order_by(PartSupplier.id)
            )
        ).all()
    )


def _supplier_response(s: PartSupplier) -> PartSupplierResponse:
    return PartSupplierResponse(
        id=s.id,
        name=s.supplier_name,
        url=s.url,
        last_checked_at=s.last_checked_at,
        last_status=s.last_status,
    )


def _suppliers_to_json(suppliers: list[PartSupplier]) -> list[dict]:
    return [{"name": s.supplier_name, "url": s.url} for s in suppliers]


async def _recompute_usage_count(session: AsyncSession, part_id: int) -> int:
    """Recompute usage_count as ``COUNT(DISTINCT project_id)`` of BOM rows.

    Cheaper alternative used by the BOM mutation hooks (in ``bom.py``).
    Exposed here for the rare case the cache needs to be refreshed.
    """
    from ..models import ProjectBOMItem

    count = await session.scalar(
        select(func.count(func.distinct(ProjectBOMItem.project_id))).where(
            ProjectBOMItem.part_id == part_id
        )
    )
    return int(count or 0)


async def _part_detail(session: AsyncSession, part: Part) -> PartDetail:
    suppliers = await _load_suppliers(session, part.id)
    revs = (
        await session.scalars(
            select(PartRevision)
            .where(PartRevision.part_id == part.id)
            .order_by(PartRevision.created_at.desc(), PartRevision.id.desc())
            .limit(10)
        )
    ).all()
    return PartDetail(
        id=part.id,
        slug=part.slug,
        name=part.name,
        sku=part.sku,
        mpn=part.mpn,
        description_md=part.description_md,
        image_url=part.image_url,
        tags=list(part.tags or []),
        status=part.status,
        created_by=part.created_by,
        created_at=part.created_at,
        updated_at=part.updated_at,
        current_revision_id=part.current_revision_id,
        usage_count=part.usage_count,
        suppliers=[_supplier_response(s) for s in suppliers],
        recent_revisions=[
            PartRevisionSummary(
                id=r.id,
                author=r.author,
                created_at=r.created_at,
                change_summary=r.change_summary,
            )
            for r in revs
        ],
    )


async def _primary_supplier_url(session: AsyncSession, part_id: int) -> Optional[str]:
    row = await session.scalar(
        select(PartSupplier.url)
        .where(PartSupplier.part_id == part_id)
        .order_by(PartSupplier.id)
        .limit(1)
    )
    return row


# --- endpoints ----------------------------------------------------------


@router.get("", response_model=list[PartSearchResult])
async def search_parts(
    q: Optional[str] = Query(None, max_length=200),
    limit: int = Query(10, ge=1, le=50),
    session: AsyncSession = Depends(get_session),
) -> list[PartSearchResult]:
    """Substring search across name / sku / mpn / aliases.

    Empty or whitespace-only ``q`` returns the most-used parts so the
    BOM-editor autosuggest can show something useful on focus.
    """
    if not q or not q.strip():
        parts = (
            await session.scalars(
                select(Part).order_by(Part.usage_count.desc(), Part.name.asc()).limit(limit)
            )
        ).all()
    else:
        needle = f"%{q.strip().lower()}%"
        # Match against name/sku/mpn directly, plus any alias that matches.
        alias_part_ids_subq = (
            select(PartAlias.part_id)
            .where(func.lower(PartAlias.alias).like(needle))
        )
        parts = (
            await session.scalars(
                select(Part)
                .where(
                    or_(
                        func.lower(Part.name).like(needle),
                        func.lower(func.coalesce(Part.sku, "")).like(needle),
                        func.lower(func.coalesce(Part.mpn, "")).like(needle),
                        Part.id.in_(alias_part_ids_subq),
                    )
                )
                .order_by(Part.usage_count.desc(), Part.name.asc())
                .limit(limit)
            )
        ).all()

    results: list[PartSearchResult] = []
    for p in parts:
        results.append(
            PartSearchResult(
                id=p.id,
                slug=p.slug,
                name=p.name,
                sku=p.sku,
                status=p.status,
                usage_count=p.usage_count,
                primary_supplier_url=await _primary_supplier_url(session, p.id),
            )
        )
    return results


@router.post("", response_model=PartDetail, status_code=201)
async def create_part(
    body: PartCreate,
    user: str = Depends(get_current_user_aged),
    session: AsyncSession = Depends(get_session),
) -> PartDetail:
    slug = await _unique_slug(session, _slugify(body.name))
    now = _now_naive_utc()
    tags = [t.strip() for t in body.tags if t and t.strip()]
    part = Part(
        slug=slug,
        name=body.name,
        sku=body.sku or None,
        mpn=body.mpn or None,
        description_md=body.description_md,
        image_url=body.image_url,
        tags=tags,
        status="draft",
        created_by=user,
        created_at=now,
        updated_at=now,
        usage_count=0,
    )
    session.add(part)
    await session.flush()

    suppliers: list[PartSupplier] = []
    if body.supplier_url:
        sup = PartSupplier(
            part_id=part.id,
            supplier_name=body.supplier_name,
            url=body.supplier_url,
        )
        session.add(sup)
        await session.flush()
        suppliers.append(sup)

    rev = PartRevision(
        part_id=part.id,
        author=user,
        change_summary="Initial draft",
        name=part.name,
        sku=part.sku,
        mpn=part.mpn,
        description_md=part.description_md,
        image_url=part.image_url,
        tags=list(tags),
        suppliers_json=_suppliers_to_json(suppliers),
        created_at=now,
    )
    session.add(rev)
    await session.flush()
    part.current_revision_id = rev.id

    await session.commit()
    await session.refresh(part)
    return await _part_detail(session, part)


async def _get_part_by_slug(session: AsyncSession, slug: str) -> Part:
    part = await session.scalar(select(Part).where(Part.slug == slug))
    if part is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Part not found")
    return part


@router.get("/{slug}", response_model=PartDetail)
async def get_part(
    slug: str,
    session: AsyncSession = Depends(get_session),
) -> PartDetail:
    part = await _get_part_by_slug(session, slug)
    return await _part_detail(session, part)


def _normalize_tags(tags: Optional[list[str]]) -> list[str]:
    if not tags:
        return []
    return [t.strip() for t in tags if t and t.strip()]


def _supplier_signature(suppliers: list[dict]) -> list[tuple]:
    """Hashable view of suppliers for change detection (order matters)."""
    return [(s.get("name"), s.get("url")) for s in suppliers]


@router.put("/{slug}", response_model=PartDetail)
async def update_part(
    slug: str,
    body: PartUpdate,
    user: str = Depends(get_current_user_aged),
    session: AsyncSession = Depends(get_session),
) -> PartDetail:
    part = await _get_part_by_slug(session, slug)

    # Resolve the *next* state for each editable field. None on the body
    # means "no change requested for this field".
    next_name = body.name if body.name is not None else part.name
    next_sku = body.sku if body.sku is not None else part.sku
    next_mpn = body.mpn if body.mpn is not None else part.mpn
    next_desc = body.description_md if body.description_md is not None else part.description_md
    next_image = body.image_url if body.image_url is not None else part.image_url
    next_tags = _normalize_tags(body.tags) if body.tags is not None else list(part.tags or [])

    current_suppliers = await _load_suppliers(session, part.id)
    current_supplier_json = _suppliers_to_json(current_suppliers)
    if body.suppliers is not None:
        next_supplier_json = [
            {"name": (s.name or None), "url": s.url} for s in body.suppliers
        ]
    else:
        next_supplier_json = current_supplier_json

    # Diff vs current — reject if nothing actually changed.
    changed = (
        next_name != part.name
        or next_sku != part.sku
        or next_mpn != part.mpn
        or next_desc != part.description_md
        or next_image != part.image_url
        or list(next_tags) != list(part.tags or [])
        or _supplier_signature(next_supplier_json) != _supplier_signature(current_supplier_json)
    )
    if not changed:
        raise HTTPException(
            status.HTTP_400_BAD_REQUEST,
            detail="No fields changed",
        )

    now = _now_naive_utc()
    # Write the revision first so we have an id to point `current_revision_id` at.
    rev = PartRevision(
        part_id=part.id,
        author=user,
        change_summary=body.change_summary,
        name=next_name,
        sku=next_sku,
        mpn=next_mpn,
        description_md=next_desc,
        image_url=next_image,
        tags=list(next_tags),
        suppliers_json=next_supplier_json,
        created_at=now,
    )
    session.add(rev)
    await session.flush()

    # Apply to the part itself.
    part.name = next_name
    part.sku = next_sku
    part.mpn = next_mpn
    part.description_md = next_desc
    part.image_url = next_image
    part.tags = list(next_tags)
    part.current_revision_id = rev.id
    part.updated_at = now

    # Replace suppliers en-bloc (simplest correct behaviour for v1).
    if body.suppliers is not None:
        await session.execute(
            delete(PartSupplier).where(PartSupplier.part_id == part.id)
        )
        for entry in next_supplier_json:
            session.add(
                PartSupplier(
                    part_id=part.id,
                    supplier_name=entry.get("name"),
                    url=entry.get("url"),
                )
            )

    await session.commit()
    await session.refresh(part)
    return await _part_detail(session, part)


@router.get("/{slug}/revisions", response_model=list[PartRevisionSummary])
async def list_revisions(
    slug: str,
    limit: int = Query(20, ge=1, le=100),
    offset: int = Query(0, ge=0),
    session: AsyncSession = Depends(get_session),
) -> list[PartRevisionSummary]:
    part = await _get_part_by_slug(session, slug)
    revs = (
        await session.scalars(
            select(PartRevision)
            .where(PartRevision.part_id == part.id)
            .order_by(PartRevision.created_at.desc(), PartRevision.id.desc())
            .limit(limit)
            .offset(offset)
        )
    ).all()
    return [
        PartRevisionSummary(
            id=r.id,
            author=r.author,
            created_at=r.created_at,
            change_summary=r.change_summary,
        )
        for r in revs
    ]


@router.get("/{slug}/revisions/{rev_id}", response_model=PartRevisionDetail)
async def get_revision(
    slug: str,
    rev_id: int,
    session: AsyncSession = Depends(get_session),
) -> PartRevisionDetail:
    part = await _get_part_by_slug(session, slug)
    rev = await session.get(PartRevision, rev_id)
    if rev is None or rev.part_id != part.id:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Revision not found")
    suppliers_raw = rev.suppliers_json or []
    suppliers = [
        PartSupplierInput(name=s.get("name"), url=s.get("url", ""))
        for s in suppliers_raw
        if s.get("url")
    ]
    return PartRevisionDetail(
        id=rev.id,
        author=rev.author,
        created_at=rev.created_at,
        change_summary=rev.change_summary,
        name=rev.name,
        sku=rev.sku,
        mpn=rev.mpn,
        description_md=rev.description_md,
        image_url=rev.image_url,
        tags=list(rev.tags or []),
        suppliers=suppliers,
    )


@router.post("/{slug}/revisions/{rev_id}/restore", response_model=PartDetail)
async def restore_revision(
    slug: str,
    rev_id: int,
    user: str = Depends(get_current_user_aged),
    session: AsyncSession = Depends(get_session),
) -> PartDetail:
    """Non-destructive restore: write a NEW revision whose snapshot
    equals ``rev_id``. History stays linear."""
    part = await _get_part_by_slug(session, slug)
    source = await session.get(PartRevision, rev_id)
    if source is None or source.part_id != part.id:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Revision not found")

    now = _now_naive_utc()
    suppliers_json = list(source.suppliers_json or [])
    new_rev = PartRevision(
        part_id=part.id,
        author=user,
        change_summary=f"Restored from revision #{source.id}",
        name=source.name,
        sku=source.sku,
        mpn=source.mpn,
        description_md=source.description_md,
        image_url=source.image_url,
        tags=list(source.tags or []),
        suppliers_json=suppliers_json,
        created_at=now,
    )
    session.add(new_rev)
    await session.flush()

    part.name = source.name
    part.sku = source.sku
    part.mpn = source.mpn
    part.description_md = source.description_md
    part.image_url = source.image_url
    part.tags = list(source.tags or [])
    part.current_revision_id = new_rev.id
    part.updated_at = now

    await session.execute(
        delete(PartSupplier).where(PartSupplier.part_id == part.id)
    )
    for entry in suppliers_json:
        if not entry.get("url"):
            continue
        session.add(
            PartSupplier(
                part_id=part.id,
                supplier_name=entry.get("name"),
                url=entry.get("url"),
            )
        )

    await session.commit()
    await session.refresh(part)
    return await _part_detail(session, part)
