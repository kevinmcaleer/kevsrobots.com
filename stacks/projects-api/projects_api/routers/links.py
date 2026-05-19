"""Related links endpoints."""

from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import get_current_user, require_terms_accepted
from ..db import get_session
from ..models import Project, ProjectLink
from ..schemas import LinkCreate, LinkResponse, LinkUpdate

router = APIRouter(prefix="/api/projects/{project_id}/links", tags=["links"])


@router.get("", response_model=list[LinkResponse])
async def list_links(
    project_id: int,
    session: AsyncSession = Depends(get_session),
) -> list[LinkResponse]:
    items = (
        await session.scalars(
            select(ProjectLink).where(ProjectLink.project_id == project_id)
        )
    ).all()
    return [LinkResponse.model_validate(i, from_attributes=True) for i in items]


@router.post("", response_model=LinkResponse, status_code=201)
async def add_link(
    project_id: int,
    body: LinkCreate,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> LinkResponse:
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    if project.author_username != user:
        raise HTTPException(status.HTTP_403_FORBIDDEN)
    link = ProjectLink(project_id=project_id, **body.model_dump())
    session.add(link)
    await session.commit()
    await session.refresh(link)
    return LinkResponse.model_validate(link, from_attributes=True)


@router.put("/{link_id}", response_model=LinkResponse)
async def update_link(
    project_id: int,
    link_id: int,
    body: LinkUpdate,
    user: str = Depends(get_current_user),
    session: AsyncSession = Depends(get_session),
) -> LinkResponse:
    """Partial-update a related link (issue #171).

    Owner-only. Each field is independently optional so the inline
    editor in the frontend can PUT one column at a time (mirrors the
    BOM row autosave pattern). Fields left ``None`` are not touched.
    """
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    if project.author_username != user:
        raise HTTPException(status.HTTP_403_FORBIDDEN)
    link = await session.get(ProjectLink, link_id)
    if link is None or link.project_id != project_id:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    # ``exclude_unset`` so a missing field stays missing rather than
    # being patched to None — partial-update semantics.
    payload = body.model_dump(exclude_unset=True)
    for field, value in payload.items():
        setattr(link, field, value)
    await session.commit()
    await session.refresh(link)
    return LinkResponse.model_validate(link, from_attributes=True)


@router.delete("/{link_id}", status_code=204)
async def delete_link(
    project_id: int,
    link_id: int,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> None:
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    if project.author_username != user:
        raise HTTPException(status.HTTP_403_FORBIDDEN)
    link = await session.get(ProjectLink, link_id)
    if link is None or link.project_id != project_id:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    await session.delete(link)
    await session.commit()
