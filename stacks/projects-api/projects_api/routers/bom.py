"""Bill of Materials endpoints.

When BOM rows are mutated, we keep ``parts.usage_count`` in sync. The
count tracks *distinct projects* that reference each part — adding a
second BOM row for the same part in the same project must NOT bump the
counter, and removing it must NOT decrement past the last row that
references it. The helpers below encode that rule using a fresh count
query inside the same transaction as the mutation.
"""

from __future__ import annotations

from typing import Optional

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy import func, select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import get_current_user, require_terms_accepted
from ..db import get_session
from ..models import Part, PartRevision, PartSupplier, Project, ProjectBOMItem
from ..schemas import BOMItemCreate, BOMItemResponse
from .parts import first_photo_cover_urls

router = APIRouter(prefix="/api/projects/{project_id}/bom", tags=["bom"])


async def _check_owner(session: AsyncSession, project_id: int, user: str) -> Project:
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Project not found")
    if project.author_username != user:
        raise HTTPException(status.HTTP_403_FORBIDDEN, detail="Not the project owner")
    return project


async def _fetch_parts_meta(
    session: AsyncSession, part_ids: list[int]
) -> dict[int, tuple[Optional[str], Optional[str], Optional[int]]]:
    """Batch-load (slug, primary_supplier_url, current_revision_id) per part.

    The primary supplier is the supplier with the smallest id for the
    part. ``current_revision_id`` lets ``_to_response`` flag a row whose
    pinned revision is behind the part's latest (versioning Phase 3).
    Returns a dict keyed by part_id so the caller can decorate
    BOMItemResponse rows in O(1) per row.
    """
    if not part_ids:
        return {}
    parts = (
        await session.scalars(
            select(Part).where(Part.id.in_(part_ids))
        )
    ).all()
    # Primary supplier per part (lowest id wins).
    suppliers = (
        await session.execute(
            select(PartSupplier.part_id, func.min(PartSupplier.id))
            .where(PartSupplier.part_id.in_(part_ids))
            .group_by(PartSupplier.part_id)
        )
    ).all()
    primary_supplier_ids = [row[1] for row in suppliers]
    supplier_url_by_part: dict[int, str] = {}
    if primary_supplier_ids:
        rows = (
            await session.execute(
                select(PartSupplier.part_id, PartSupplier.url)
                .where(PartSupplier.id.in_(primary_supplier_ids))
            )
        ).all()
        supplier_url_by_part = {pid: url for pid, url in rows}
    return {
        p.id: (p.slug, supplier_url_by_part.get(p.id), p.current_revision_id)
        for p in parts
    }


async def _fetch_supplier_pricing(
    session: AsyncSession, supplier_ids: list[int]
) -> dict[int, tuple[Optional[float], Optional[str]]]:
    """Batch-load (unit_cost, currency_code) for the given supplier ids.

    Returns a dict keyed by supplier_id; missing keys mean the supplier
    no longer exists (ON DELETE SET NULL would normally clear the FK,
    but the resolver is defensive in case the row is stale for any
    other reason). The caller folds this into ``_to_response`` so the
    BOM list endpoint stays at O(1) extra query per request regardless
    of how many rows reference suppliers — important because a project
    with 50 BOM rows each linked to a different supplier must not
    issue 50 separate SELECTs (N+1).

    SQL pattern: a single ``WHERE id IN (:ids)`` against
    ``part_suppliers`` is one round-trip and uses the PK index. The
    cost is bounded by ``len(supplier_ids)`` regardless of which
    project we're loading.
    """
    if not supplier_ids:
        return {}
    rows = (
        await session.execute(
            select(
                PartSupplier.id, PartSupplier.unit_cost, PartSupplier.currency_code
            ).where(PartSupplier.id.in_(supplier_ids))
        )
    ).all()
    return {sid: (cost, code) for sid, cost, code in rows}


async def _covers_for(session: AsyncSession, parts_meta) -> dict[int, str]:
    """First-photo cover URL per part id, derived from parts_meta (which
    carries the slug). Batched — one query for the whole BOM."""
    return await first_photo_cover_urls(
        session, {pid: m[0] for pid, m in parts_meta.items() if m[0]}
    )


def _to_response(
    item: ProjectBOMItem,
    parts_meta: dict[int, tuple[Optional[str], Optional[str], Optional[int]]],
    supplier_pricing: dict[int, tuple[Optional[float], Optional[str]]],
    covers: Optional[dict[int, str]] = None,
) -> BOMItemResponse:
    """Build a BOMItemResponse, decorating with part slug / supplier.

    Resolution order for the effective price (supplier-pricing feature):

      1. If the row has a ``supplier_id`` AND the supplier still exists
         AND the supplier has a non-NULL ``unit_cost``: use the
         supplier's live values. ``price_source = "supplier"``.
      2. Otherwise (no supplier link, supplier deleted under us, or
         supplier has no price recorded): fall back to the row's own
         ``unit_cost`` + ``currency_code``. ``price_source = "row"``.

    A stale ``supplier_id`` pointing at a deleted supplier behaves
    identically to having no link at all — ON DELETE SET NULL on the FK
    will eventually NULL it out, but until then the resolver silently
    falls through to the row values rather than 500ing.
    """
    slug = None
    primary_supplier = None
    current_revision_id = None
    if item.part_id is not None:
        slug, primary_supplier, current_revision_id = parts_meta.get(
            item.part_id, (None, None, None)
        )
    # "Newer version available": the row is pinned to an older revision than
    # the part's latest. Only meaningful when both ids are known.
    outdated = (
        item.part_revision_id is not None
        and current_revision_id is not None
        and item.part_revision_id != current_revision_id
    )

    effective_unit_cost: Optional[float] = item.unit_cost
    effective_currency_code: Optional[str] = item.currency_code
    price_source = "row"
    if item.supplier_id is not None:
        sup_cost, sup_currency = supplier_pricing.get(
            item.supplier_id, (None, None)
        )
        if sup_cost is not None:
            effective_unit_cost = sup_cost
            effective_currency_code = sup_currency
            price_source = "supplier"

    return BOMItemResponse(
        id=item.id,
        name=item.name,
        quantity=item.quantity,
        unit=item.unit,
        unit_cost=item.unit_cost,
        currency_code=item.currency_code,
        supplier_url=item.supplier_url,
        sort_order=item.sort_order,
        part_id=item.part_id,
        part_slug=slug,
        part_primary_supplier_url=primary_supplier,
        part_revision_id=item.part_revision_id,
        part_revision_outdated=outdated,
        supplier_id=item.supplier_id,
        effective_unit_cost=effective_unit_cost,
        effective_currency_code=effective_currency_code,
        price_source=price_source,
        cover_url=(covers or {}).get(item.part_id) if item.part_id is not None else None,
    )


async def _refresh_usage_count(session: AsyncSession, part_id: int) -> None:
    """Recompute ``parts.usage_count`` as COUNT(DISTINCT project_id).

    Cheap (single index lookup) and avoids tricky "is this the only row
    for this part in this project?" bookkeeping. Clamped to >= 0.
    """
    part = await session.get(Part, part_id)
    if part is None:
        return
    count = await session.scalar(
        select(func.count(func.distinct(ProjectBOMItem.project_id))).where(
            ProjectBOMItem.part_id == part_id
        )
    )
    part.usage_count = max(0, int(count or 0))


@router.get("", response_model=list[BOMItemResponse])
async def list_bom(
    project_id: int,
    session: AsyncSession = Depends(get_session),
) -> list[BOMItemResponse]:
    items = (
        await session.scalars(
            select(ProjectBOMItem)
            .where(ProjectBOMItem.project_id == project_id)
            .order_by(ProjectBOMItem.sort_order)
        )
    ).all()
    part_ids = [i.part_id for i in items if i.part_id is not None]
    parts_meta = await _fetch_parts_meta(session, list(set(part_ids)))
    # Batch-load supplier prices in one round-trip — see
    # ``_fetch_supplier_pricing`` for the N+1 mitigation rationale. A
    # 50-row BOM linked to 50 different suppliers still issues exactly
    # one extra SELECT, not 50.
    supplier_ids = [i.supplier_id for i in items if i.supplier_id is not None]
    supplier_pricing = await _fetch_supplier_pricing(
        session, list(set(supplier_ids))
    )
    covers = await _covers_for(session, parts_meta)
    return [_to_response(i, parts_meta, supplier_pricing, covers) for i in items]


async def _validate_supplier_id(
    session: AsyncSession, supplier_id: Optional[int]
) -> Optional[int]:
    """Return ``supplier_id`` if the supplier exists, else None.

    Same validate-or-drop pattern as ``part_id`` on the BOM endpoints —
    a stale or unknown id shouldn't make the request 4xx because the
    user might be saving other field changes at the same time. The
    resolver in ``_to_response`` is also defensive against this case so
    the row stays useful even if the supplier disappears mid-edit.
    """
    if supplier_id is None:
        return None
    existing = await session.get(PartSupplier, supplier_id)
    return supplier_id if existing is not None else None


async def _resolve_part_revision(
    session: AsyncSession, part: Part, requested: Optional[int]
) -> Optional[int]:
    """Pin to a valid requested revision (must belong to ``part``), else the
    part's current revision. Used on create and on a part-change in update."""
    if requested is not None:
        rev = await session.get(PartRevision, requested)
        if rev is not None and rev.part_id == part.id:
            return requested
    return part.current_revision_id


@router.post("", response_model=BOMItemResponse, status_code=201)
async def add_bom_item(
    project_id: int,
    body: BOMItemCreate,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> BOMItemResponse:
    await _check_owner(session, project_id, user)
    payload = body.model_dump()
    requested_rev = payload.pop("part_revision_id", None)
    part_id: Optional[int] = payload.get("part_id")
    part_obj: Optional[Part] = None
    if part_id is not None:
        # Validate the part exists; if not, drop the link rather than 400.
        # (BOM creation shouldn't blow up because the catalog row was
        # deleted out from under us.)
        part_obj = await session.get(Part, part_id)
        if part_obj is None:
            payload["part_id"] = None
            part_id = None
    # Versioning Phase 3: pin to the requested revision (if valid) or the
    # part's current revision; null when no part is linked.
    payload["part_revision_id"] = (
        await _resolve_part_revision(session, part_obj, requested_rev)
        if part_obj is not None else None
    )
    payload["supplier_id"] = await _validate_supplier_id(
        session, payload.get("supplier_id")
    )
    item = ProjectBOMItem(project_id=project_id, **payload)
    session.add(item)
    await session.flush()
    if part_id is not None:
        await _refresh_usage_count(session, part_id)
    await session.commit()
    await session.refresh(item)
    parts_meta = await _fetch_parts_meta(session, [item.part_id] if item.part_id else [])
    supplier_pricing = await _fetch_supplier_pricing(
        session, [item.supplier_id] if item.supplier_id else []
    )
    covers = await _covers_for(session, parts_meta)
    return _to_response(item, parts_meta, supplier_pricing, covers)


@router.put("/{item_id}", response_model=BOMItemResponse)
async def update_bom_item(
    project_id: int,
    item_id: int,
    body: BOMItemCreate,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> BOMItemResponse:
    await _check_owner(session, project_id, user)
    item = await session.get(ProjectBOMItem, item_id)
    if item is None or item.project_id != project_id:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="BOM item not found")
    old_part_id = item.part_id
    old_rev = item.part_revision_id
    payload = body.model_dump()
    requested_rev = payload.pop("part_revision_id", None)
    new_part_id: Optional[int] = payload.get("part_id")
    part_obj: Optional[Part] = None
    if new_part_id is not None:
        part_obj = await session.get(Part, new_part_id)
        if part_obj is None:
            payload["part_id"] = None
            new_part_id = None
    payload["supplier_id"] = await _validate_supplier_id(
        session, payload.get("supplier_id")
    )
    for field, value in payload.items():
        setattr(item, field, value)
    # Versioning Phase 3: resolve the revision pin. An explicit request, or a
    # part change, (re)pins; an unrelated edit (same part, no request) keeps
    # the existing pin so saving the row doesn't silently bump it to latest.
    if part_obj is None:
        item.part_revision_id = None
    elif requested_rev is not None:
        item.part_revision_id = await _resolve_part_revision(
            session, part_obj, requested_rev
        )
    elif new_part_id == old_part_id and old_rev is not None:
        item.part_revision_id = old_rev
    else:
        item.part_revision_id = part_obj.current_revision_id
    await session.flush()
    if old_part_id != new_part_id:
        if old_part_id is not None:
            await _refresh_usage_count(session, old_part_id)
        if new_part_id is not None:
            await _refresh_usage_count(session, new_part_id)
    await session.commit()
    await session.refresh(item)
    parts_meta = await _fetch_parts_meta(session, [item.part_id] if item.part_id else [])
    supplier_pricing = await _fetch_supplier_pricing(
        session, [item.supplier_id] if item.supplier_id else []
    )
    covers = await _covers_for(session, parts_meta)
    return _to_response(item, parts_meta, supplier_pricing, covers)


@router.post("/{item_id}/upgrade-revision", response_model=BOMItemResponse)
async def upgrade_bom_item_revision(
    project_id: int,
    item_id: int,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> BOMItemResponse:
    """Versioning Phase 4: re-pin a BOM row to its part's CURRENT revision.

    The "take the update" action behind the row's update-available badge.
    The server knows the current revision, so the client doesn't need to —
    it just calls this. No-op-safe: re-pinning an already-current row is
    harmless. 400 if the row isn't linked to a part."""
    await _check_owner(session, project_id, user)
    item = await session.get(ProjectBOMItem, item_id)
    if item is None or item.project_id != project_id:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="BOM item not found")
    if item.part_id is None:
        raise HTTPException(
            status.HTTP_400_BAD_REQUEST,
            detail="BOM item is not linked to a part",
        )
    part = await session.get(Part, item.part_id)
    if part is None:
        raise HTTPException(
            status.HTTP_400_BAD_REQUEST, detail="Linked part no longer exists"
        )
    item.part_revision_id = part.current_revision_id
    await session.commit()
    await session.refresh(item)
    parts_meta = await _fetch_parts_meta(session, [item.part_id])
    supplier_pricing = await _fetch_supplier_pricing(
        session, [item.supplier_id] if item.supplier_id else []
    )
    covers = await _covers_for(session, parts_meta)
    return _to_response(item, parts_meta, supplier_pricing, covers)


@router.delete("/{item_id}", status_code=204)
async def delete_bom_item(
    project_id: int,
    item_id: int,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> None:
    await _check_owner(session, project_id, user)
    item = await session.get(ProjectBOMItem, item_id)
    if item is None or item.project_id != project_id:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="BOM item not found")
    old_part_id = item.part_id
    await session.delete(item)
    await session.flush()
    if old_part_id is not None:
        await _refresh_usage_count(session, old_part_id)
    await session.commit()
