"""Build journal endpoints."""

from __future__ import annotations

from datetime import datetime, timezone

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import get_current_user, require_terms_accepted
from ..db import get_session
from ..models import Project, ProjectJournalEntry
from ..schemas import JournalEntryCreate, JournalEntryResponse

router = APIRouter(prefix="/api/projects/{project_id}/journal", tags=["journal"])


@router.get("", response_model=list[JournalEntryResponse])
async def list_entries(
    project_id: int,
    session: AsyncSession = Depends(get_session),
) -> list[JournalEntryResponse]:
    entries = (
        await session.scalars(
            select(ProjectJournalEntry)
            .where(ProjectJournalEntry.project_id == project_id)
            .order_by(ProjectJournalEntry.created_at.desc())
        )
    ).all()
    return [JournalEntryResponse.model_validate(e, from_attributes=True) for e in entries]


@router.post("", response_model=JournalEntryResponse, status_code=201)
async def add_entry(
    project_id: int,
    body: JournalEntryCreate,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> JournalEntryResponse:
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    if project.author_username != user:
        raise HTTPException(status.HTTP_403_FORBIDDEN)
    entry = ProjectJournalEntry(project_id=project_id, **body.model_dump())
    session.add(entry)
    await session.commit()
    await session.refresh(entry)
    return JournalEntryResponse.model_validate(entry, from_attributes=True)


@router.put("/{entry_id}", response_model=JournalEntryResponse)
async def update_entry(
    project_id: int,
    entry_id: int,
    body: JournalEntryCreate,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> JournalEntryResponse:
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    if project.author_username != user:
        raise HTTPException(status.HTTP_403_FORBIDDEN)
    entry = await session.get(ProjectJournalEntry, entry_id)
    if entry is None or entry.project_id != project_id:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    for field, value in body.model_dump().items():
        setattr(entry, field, value)
    entry.updated_at = datetime.now(timezone.utc).replace(tzinfo=None)
    await session.commit()
    await session.refresh(entry)
    return JournalEntryResponse.model_validate(entry, from_attributes=True)


@router.delete("/{entry_id}", status_code=204)
async def delete_entry(
    project_id: int,
    entry_id: int,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> None:
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    if project.author_username != user:
        raise HTTPException(status.HTTP_403_FORBIDDEN)
    entry = await session.get(ProjectJournalEntry, entry_id)
    if entry is None or entry.project_id != project_id:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    await session.delete(entry)
    await session.commit()
