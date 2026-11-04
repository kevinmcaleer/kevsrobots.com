"""Admin view of how much storage each project uses.

The page at ``/admin/projects/storage/`` lists every project with the
bytes its files take against the per-project limit, split into the files
in the working copy and the ones removed from it but kept because a
version still uses them (issue #439). Drafts are included: these routes
are admin-only.
"""

from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy import case, func, select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import require_admin
from ..config import get_settings
from ..db import get_session
from ..models import Project, ProjectFile, ProjectImage, ProjectVersion
from ..schemas import ProjectStorageFile, ProjectStorageList, ProjectStorageRow
from ..versions import _ids, list_versions

router = APIRouter(prefix="/api/admin/storage", tags=["admin-storage"])


@router.get("/projects", response_model=ProjectStorageList)
async def list_project_storage(
    _admin: str = Depends(require_admin),
    session: AsyncSession = Depends(get_session),
) -> ProjectStorageList:
    removed = ProjectFile.removed_at.is_not(None)
    files = (
        select(
            ProjectFile.project_id,
            func.sum(case((removed, 0), else_=1)).label("file_count"),
            func.coalesce(func.sum(case((removed, 0), else_=ProjectFile.file_size)), 0).label("file_bytes"),
            func.sum(case((removed, 1), else_=0)).label("removed_count"),
            func.coalesce(func.sum(case((removed, ProjectFile.file_size), else_=0)), 0).label("removed_bytes"),
        )
        .group_by(ProjectFile.project_id)
        .subquery()
    )
    images = (
        select(ProjectImage.project_id, func.count().label("n"))
        .where(ProjectImage.removed_at.is_(None))
        .group_by(ProjectImage.project_id)
        .subquery()
    )
    versions = (
        select(ProjectVersion.project_id, func.count().label("n"))
        .group_by(ProjectVersion.project_id)
        .subquery()
    )
    rows = (
        await session.execute(
            select(
                Project,
                func.coalesce(files.c.file_count, 0),
                func.coalesce(files.c.file_bytes, 0),
                func.coalesce(files.c.removed_count, 0),
                func.coalesce(files.c.removed_bytes, 0),
                func.coalesce(images.c.n, 0),
                func.coalesce(versions.c.n, 0),
            )
            .outerjoin(files, files.c.project_id == Project.id)
            .outerjoin(images, images.c.project_id == Project.id)
            .outerjoin(versions, versions.c.project_id == Project.id)
        )
    ).all()

    # The biggest file per project, removed ones included since they count.
    largest: dict[int, tuple[str, int]] = {}
    for pid, name, size in (
        await session.execute(
            select(ProjectFile.project_id, ProjectFile.filename, ProjectFile.file_size)
        )
    ).all():
        if size > largest.get(pid, ("", -1))[1]:
            largest[pid] = (name, size)

    projects = []
    for p, file_count, file_bytes, removed_count, removed_bytes, image_count, version_count in rows:
        name, size = largest.get(p.id, (None, 0))
        projects.append(ProjectStorageRow(
            project_id=p.id, title=p.title, slug=p.slug,
            author_username=p.author_username, status=p.status,
            file_count=file_count, file_bytes=file_bytes,
            removed_count=removed_count, removed_bytes=removed_bytes,
            counted_bytes=file_bytes + removed_bytes,
            image_count=image_count, version_count=version_count,
            largest_filename=name, largest_bytes=size,
        ))
    projects.sort(key=lambda r: r.counted_bytes, reverse=True)
    return ProjectStorageList(limit_bytes=get_settings().max_project_storage, projects=projects)


@router.get("/projects/{project_id}/files", response_model=list[ProjectStorageFile])
async def list_project_storage_files(
    project_id: int,
    _admin: str = Depends(require_admin),
    session: AsyncSession = Depends(get_session),
) -> list[ProjectStorageFile]:
    """Every file row of the project, removed ones included, biggest first,
    with the numbers of the versions that use each."""
    if await session.get(Project, project_id) is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    used_by: dict[int, list[int]] = {}
    for v in await list_versions(session, project_id):
        for file_id in _ids(v.snapshot, "files"):
            used_by.setdefault(file_id, []).append(v.number)
    rows = (
        await session.scalars(
            select(ProjectFile)
            .where(ProjectFile.project_id == project_id)
            .order_by(ProjectFile.file_size.desc(), ProjectFile.id)
        )
    ).all()
    return [
        ProjectStorageFile(
            id=f.id, filename=f.filename, file_size=f.file_size,
            uploaded_at=f.uploaded_at, removed_at=f.removed_at,
            versions=sorted(used_by.get(f.id, [])),
        )
        for f in rows
    ]
