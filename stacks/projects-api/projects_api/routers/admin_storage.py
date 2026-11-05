"""Admin view of how much storage each project uses, for the page at
``/admin/projects/storage/``.

``used`` is what counts toward the per-project limit (``max_project_storage``):
the working copy's Files & Models. Files removed from the working copy but
kept for a published version (``removed_at``, issue #439) are ``kept``,
shown apart because they don't count. Sizes are as stored: a model kept
gzipped (storage.compress_for_storage) counts its gzipped size. Pictures
(gallery and graphics) are stored without a size and never count, so only
their number is given. ``POST /compress`` gzips a project's files stored
before that.
"""

from __future__ import annotations

from datetime import datetime
from typing import Optional

import logging
from pathlib import Path

from fastapi import APIRouter, Depends, HTTPException, status
from pydantic import BaseModel
from sqlalchemy import case, func, select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import require_admin
from ..config import get_settings
from ..db import get_session
from ..models import Project, ProjectFile, ProjectImage, ProjectVersion
from ..storage import COMPRESSIBLE_EXTENSIONS, STORED_GZ, compress_for_storage, delete_file, file_category, read_file, save_file
from ..versions import _ids, list_versions

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/admin/storage", tags=["admin-storage"])

#: Bytes a file takes in storage.
STORED = func.coalesce(ProjectFile.stored_size, ProjectFile.file_size)


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
    versions: int = 0
    largest_file: Optional[str] = None
    largest_size: int = 0
    #: Files that would be smaller gzipped but aren't yet (stored before compression).
    compressible: int = 0


class StorageOverview(BaseModel):
    limit: int
    projects: list[ProjectStorage]


class StoredFile(BaseModel):
    id: int
    filename: str
    file_size: int
    file_type: str
    #: Bytes in storage (smaller than file_size when stored gzipped).
    stored_size: int = 0
    uploaded_at: Optional[datetime] = None
    removed_at: Optional[datetime] = None
    #: Numbers of the versions that use the file.
    versions: list[int] = []


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
                func.coalesce(func.sum(case((active, STORED), else_=0)), 0),
                func.coalesce(func.sum(case((active, 0), else_=STORED)), 0),
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

    versions = dict(
        (
            await session.execute(
                select(ProjectVersion.project_id, func.count(ProjectVersion.id)).group_by(ProjectVersion.project_id)
            )
        ).all()
    )

    # Each project's largest working-copy file: one pass, keeping the biggest.
    largest: dict[int, tuple[str, int]] = {}
    compressible: dict[int, int] = {}
    for pid, name, size, packed, path, removed in (
        await session.execute(
            select(ProjectFile.project_id, ProjectFile.filename, STORED, ProjectFile.stored_size, ProjectFile.file_path, ProjectFile.removed_at)
        )
    ).all():
        if packed is None and not (path or "").endswith(STORED_GZ) and _may_compress(name):
            compressible[pid] = compressible.get(pid, 0) + 1
        if removed is not None:
            continue
        if size and size > largest.get(pid, ("", -1))[1]:
            largest[pid] = (name, size)

    projects = (await session.execute(select(Project.id, Project.title, Project.author_username, Project.status))).all()
    out = []
    for pid, title, author, state in projects:
        used, kept, n, m = sizes.get(pid, (0, 0, 0, 0))
        name, size = largest.get(pid, (None, 0))
        out.append(ProjectStorage(
            project_id=pid, title=title or "", author=author or "", status=str(state or ""),
            used=used, kept=kept, files=n, kept_files=m, images=int(images.get(pid, 0)), versions=int(versions.get(pid, 0)),
            largest_file=name, largest_size=size, compressible=compressible.get(pid, 0),
        ))
    out.sort(key=lambda p: p.used, reverse=True)
    return StorageOverview(limit=get_settings().max_project_storage, projects=out)


@router.get("/projects/{project_id}/files", response_model=list[StoredFile])
async def project_files(
    project_id: int,
    _admin: str = Depends(require_admin),
    session: AsyncSession = Depends(get_session),
) -> list[StoredFile]:
    """Every file row of one project, kept ones included, biggest first,
    with the numbers of the versions that use each."""
    if await session.get(Project, project_id) is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    used_by: dict[int, list[int]] = {}
    for ver in await list_versions(session, project_id):
        for file_id in _ids(ver.snapshot, "files"):
            used_by.setdefault(file_id, []).append(ver.number)
    rows = (
        await session.scalars(
            select(ProjectFile).where(ProjectFile.project_id == project_id).order_by(STORED.desc())
        )
    ).all()
    return [
        StoredFile(
            id=f.id, filename=f.filename, file_size=f.file_size or 0, file_type=file_category(f.filename), stored_size=f.stored_size or f.file_size or 0,
            uploaded_at=f.uploaded_at, removed_at=f.removed_at, versions=sorted(used_by.get(f.id, [])),
        )
        for f in rows
    ]


def _may_compress(filename: str) -> bool:
    return Path(filename or "").suffix.lower() in COMPRESSIBLE_EXTENSIONS


class CompressResult(BaseModel):
    compressed: int
    skipped: int
    failed: int
    saved: int


@router.post("/projects/{project_id}/compress", response_model=CompressResult)
async def compress_project(
    project_id: int,
    _admin: str = Depends(require_admin),
    session: AsyncSession = Depends(get_session),
) -> CompressResult:
    """Gzips the project's files stored before files were kept compressed
    (kept-for-a-version ones too). Each is written anew, checked to read
    back byte for byte, and only then is the old file deleted, so nothing
    is lost if a step fails. Downloads are unchanged."""
    if await session.get(Project, project_id) is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    rows = (
        await session.scalars(
            select(ProjectFile).where(ProjectFile.project_id == project_id, ProjectFile.stored_size.is_(None))
        )
    ).all()
    done = skipped = failed = saved = 0
    for f in rows:
        if f.file_path.endswith(STORED_GZ) or not _may_compress(f.filename):
            skipped += 1
            continue
        data = read_file(f.file_path)
        if data is None:
            failed += 1
            continue
        packed, ok = compress_for_storage(f.filename, data)
        if not ok:
            skipped += 1
            continue
        name = f.file_path.rsplit("/", 1)[-1] + STORED_GZ
        path = save_file(packed, name, project_id, "files")
        if path is None or read_file(path) != data:
            if path:
                delete_file(path)
            failed += 1
            continue
        old = f.file_path
        f.file_path = path
        f.stored_size = len(packed)
        await session.commit()
        delete_file(old)
        done += 1
        saved += len(data) - len(packed)
    logger.info("Compressed %s files of project %s, saving %s bytes", done, project_id, saved)
    return CompressResult(compressed=done, skipped=skipped, failed=failed, saved=saved)
