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
from ..models import Part, PartSupplier, Project, ProjectBOMItem
from ..schemas import BOMItemCreate, BOMItemResponse

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
) -> dict[int, tuple[Optional[str], Optional[str]]]:
    """Batch-load (slug, primary_supplier_url) for the given part ids.

    The primary supplier is the supplier with the smallest id for the
    part. Returns a dict keyed by part_id so the caller can decorate
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
        p.id: (p.slug, supplier_url_by_part.get(p.id))
        for p in parts
    }


def _to_response(
    item: ProjectBOMItem,
    parts_meta: dict[int, tuple[Optional[str], Optional[str]]],
) -> BOMItemResponse:
    """Build a BOMItemResponse, decorating with part slug / supplier."""
    slug = None
    primary_supplier = None
    if item.part_id is not None:
        slug, primary_supplier = parts_meta.get(item.part_id, (None, None))
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
    return [_to_response(i, parts_meta) for i in items]


@router.post("", response_model=BOMItemResponse, status_code=201)
async def add_bom_item(
    project_id: int,
    body: BOMItemCreate,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> BOMItemResponse:
    await _check_owner(session, project_id, user)
    payload = body.model_dump()
    part_id: Optional[int] = payload.get("part_id")
    if part_id is not None:
        # Validate the part exists; if not, drop the link rather than 400.
        # (BOM creation shouldn't blow up because the catalog row was
        # deleted out from under us.)
        existing = await session.get(Part, part_id)
        if existing is None:
            payload["part_id"] = None
            part_id = None
    item = ProjectBOMItem(project_id=project_id, **payload)
    session.add(item)
    await session.flush()
    if part_id is not None:
        await _refresh_usage_count(session, part_id)
    await session.commit()
    await session.refresh(item)
    parts_meta = await _fetch_parts_meta(session, [item.part_id] if item.part_id else [])
    return _to_response(item, parts_meta)


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
    payload = body.model_dump()
    new_part_id: Optional[int] = payload.get("part_id")
    if new_part_id is not None:
        existing = await session.get(Part, new_part_id)
        if existing is None:
            payload["part_id"] = None
            new_part_id = None
    for field, value in payload.items():
        setattr(item, field, value)
    await session.flush()
    if old_part_id != new_part_id:
        if old_part_id is not None:
            await _refresh_usage_count(session, old_part_id)
        if new_part_id is not None:
            await _refresh_usage_count(session, new_part_id)
    await session.commit()
    await session.refresh(item)
    parts_meta = await _fetch_parts_meta(session, [item.part_id] if item.part_id else [])
    return _to_response(item, parts_meta)


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
