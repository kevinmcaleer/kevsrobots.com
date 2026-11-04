"""Admin view of how much storage each project uses, for the page at
``/admin/projects/storage/``.

``used`` is what counts toward the per-project limit (``max_project_storage``):
the working copy's Files & Models. Files removed from the working copy but
kept for a published version (``removed_at``, issue #439) are ``kept``,
shown apart because they don't count. Pictures (gallery and graphics) are
stored without a size and never count, so only their number is given.
"""

from __future__ import annotations

from datetime import datetime
from typing import Optional

from fastapi import APIRouter, Depends, HTTPException, status
from pydantic import BaseModel
from sqlalchemy import case, func, select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import require_admin
from ..config import get_settings
from ..db import get_session
from ..models import Project, ProjectFile, ProjectImage
from ..storage import file_category

router = APIRouter(prefix="/api/admin/storage", tags=["admin-storage"])


class ProjectStorage(BaseModel):
    project_id: int
    title: str
    author: str
    status: str
    used: int
    kept: int
    files: int
    kept_files: int
    images: int
    largest_file: Optional[str] = None
    largest_size: int = 0


class StorageOverview(BaseModel):
    limit: int
    projects: list[ProjectStorage]


class StoredFile(BaseModel):
    id: int
    filename: str
    file_size: int
    file_type: str
    uploaded_at: Optional[datetime] = None
    removed_at: Optional[datetime] = None


@router.get("/projects", response_model=StorageOverview)
async def project_storage(
    _admin: str = Depends(require_admin),
    session: AsyncSession = Depends(get_session),
) -> StorageOverview:
    active = ProjectFile.removed_at.is_(None)
    file_rows = (
        await session.execute(
            select(
                ProjectFile.project_id,
                func.coalesce(func.sum(case((active, ProjectFile.file_size), else_=0)), 0),
                func.coalesce(func.sum(case((active, 0), else_=ProjectFile.file_size)), 0),
                func.sum(case((active, 1), else_=0)),
                func.sum(case((active, 0), else_=1)),
            ).group_by(ProjectFile.project_id)
        )
    ).all()
    sizes = {pid: (int(u), int(k), int(n or 0), int(m or 0)) for pid, u, k, n, m in file_rows}

    images = dict(
        (
            await session.execute(
                select(ProjectImage.project_id, func.count(ProjectImage.id))
                .where(ProjectImage.removed_at.is_(None))
                .group_by(ProjectImage.project_id)
            )
        ).all()
    )

    # Each project's largest working-copy file: one pass, keeping the biggest.
    largest: dict[int, tuple[str, int]] = {}
    for pid, name, size in (
        await session.execute(
            select(ProjectFile.project_id, ProjectFile.filename, ProjectFile.file_size).where(active)
        )
    ).all():
        if size and size > largest.get(pid, ("", -1))[1]:
            largest[pid] = (name, size)

    projects = (await session.execute(select(Project.id, Project.title, Project.author_username, Project.status))).all()
    out = []
    for pid, title, author, state in projects:
        used, kept, n, m = sizes.get(pid, (0, 0, 0, 0))
        name, size = largest.get(pid, (None, 0))
        out.append(ProjectStorage(
            project_id=pid, title=title or "", author=author or "", status=str(state or ""),
            used=used, kept=kept, files=n, kept_files=m, images=int(images.get(pid, 0)),
            largest_file=name, largest_size=size,
        ))
    out.sort(key=lambda p: p.used, reverse=True)
    return StorageOverview(limit=get_settings().max_project_storage, projects=out)


@router.get("/projects/{project_id}/files", response_model=list[StoredFile])
async def project_files(
    project_id: int,
    _admin: str = Depends(require_admin),
    session: AsyncSession = Depends(get_session),
) -> list[StoredFile]:
    """Every file row of one project, kept ones included, biggest first."""
    if await session.get(Project, project_id) is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    rows = (
        await session.scalars(
            select(ProjectFile).where(ProjectFile.project_id == project_id).order_by(ProjectFile.file_size.desc())
        )
    ).all()
    return [
        StoredFile(
            id=f.id, filename=f.filename, file_size=f.file_size or 0, file_type=file_category(f.filename),
            uploaded_at=f.uploaded_at, removed_at=f.removed_at,
        )
        for f in rows
    ]
