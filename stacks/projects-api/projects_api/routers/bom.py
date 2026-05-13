"""Bill of Materials endpoints."""

from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import get_current_user
from ..db import get_session
from ..models import Project, ProjectBOMItem
from ..schemas import BOMItemCreate, BOMItemResponse

router = APIRouter(prefix="/api/projects/{project_id}/bom", tags=["bom"])


async def _check_owner(session: AsyncSession, project_id: int, user: str) -> Project:
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Project not found")
    if project.author_username != user:
        raise HTTPException(status.HTTP_403_FORBIDDEN, detail="Not the project owner")
    return project


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
    item = ProjectBOMItem(project_id=project_id, **body.model_dump())
    session.add(item)
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
    for field, value in body.model_dump().items():
        setattr(item, field, value)
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
    await session.delete(item)
    await session.commit()
