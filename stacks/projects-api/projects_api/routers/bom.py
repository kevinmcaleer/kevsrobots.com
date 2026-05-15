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

from ..auth import get_current_user
from ..db import get_session
from ..models import Part, Project, ProjectBOMItem
from ..schemas import BOMItemCreate, BOMItemResponse

router = APIRouter(prefix="/api/projects/{project_id}/bom", tags=["bom"])


async def _check_owner(session: AsyncSession, project_id: int, user: str) -> Project:
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Project not found")
    if project.author_username != user:
        raise HTTPException(status.HTTP_403_FORBIDDEN, detail="Not the project owner")
    return project


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
    return [BOMItemResponse.model_validate(i, from_attributes=True) for i in items]


@router.post("", response_model=BOMItemResponse, status_code=201)
async def add_bom_item(
    project_id: int,
    body: BOMItemCreate,
    user: str = Depends(get_current_user),
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
    return BOMItemResponse.model_validate(item, from_attributes=True)


@router.put("/{item_id}", response_model=BOMItemResponse)
async def update_bom_item(
    project_id: int,
    item_id: int,
    body: BOMItemCreate,
    user: str = Depends(get_current_user),
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
    return BOMItemResponse.model_validate(item, from_attributes=True)


@router.delete("/{item_id}", status_code=204)
async def delete_bom_item(
    project_id: int,
    item_id: int,
    user: str = Depends(get_current_user),
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
