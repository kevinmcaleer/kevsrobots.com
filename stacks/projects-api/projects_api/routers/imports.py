"""Imports from other apps: BetterCad sends a model and its build steps to a
project (``bettercad_import.py``), and the editor offers to bring them in.

- ``POST /api/projects/{id}/imports``: BetterCad sends the build steps .zip.
  It waits as the project's pending import, replacing any older pending one.
- ``GET /api/projects/{id}/imports/pending``: the editor asks whether one is
  waiting (``null`` when not).
- ``POST …/imports/{import_id}/apply``: the editor brings it in. Pictures go
  to the gallery and models to Files & Models, replacing the last import's,
  and the editor's text comes back with the Build steps section in it. The
  editor puts that text in and saves it the usual way. The parts' STLs go
  into the project's 3D model too (``<project>.model.json``), so the 3D
  scenes can show them.
- ``POST …/imports/{import_id}/dismiss``: not now.
"""

from __future__ import annotations

import json
import posixpath
import re
import time
from datetime import datetime, timezone
from typing import Optional

from fastapi import APIRouter, Depends, Form, HTTPException, UploadFile, status
from sqlalchemy import func, select
from sqlalchemy.ext.asyncio import AsyncSession

from .. import bettercad_import as bc
from .. import versions as v
from ..auth import require_terms_accepted
from ..config import get_settings
from ..db import get_session
from ..models import Project, ProjectFile, ProjectImage, ProjectImport
from ..schemas import ImportApplied, ImportApply, ImportResponse
from ..storage import (
    STORED_GZ,
    compress_for_storage,
    delete_file,
    file_category,
    generate_filename,
    read_file,
    save_file,
)
from .files import storage_used

router = APIRouter(prefix="/api/projects/{project_id}/imports", tags=["imports"])


def _now() -> datetime:
    return datetime.now(timezone.utc).replace(tzinfo=None)


async def _own_project(session: AsyncSession, project_id: int, user: str) -> Project:
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="There's no project with that id.")
    if project.author_username != user:
        raise HTTPException(status.HTTP_403_FORBIDDEN, detail="That project isn't yours.")
    return project


async def _last_applied(session: AsyncSession, project_id: int, source: str) -> Optional[ProjectImport]:
    return await session.scalar(
        select(ProjectImport)
        .where(
            ProjectImport.project_id == project_id,
            ProjectImport.source == source,
            ProjectImport.status == "applied",
        )
        .order_by(ProjectImport.applied_at.desc(), ProjectImport.id.desc())
        .limit(1)
    )


async def _response(session: AsyncSession, row: ProjectImport) -> ImportResponse:
    resp = ImportResponse.model_validate(row, from_attributes=True)
    last = await _last_applied(session, row.project_id, row.source)
    resp.is_update = last is not None and last.id != row.id
    return resp


@router.post("", response_model=ImportResponse, status_code=201)
async def send_import(
    project_id: int,
    file: UploadFile,
    source: str = Form("bettercad"),
    model_url: Optional[str] = Form(None, max_length=2000),
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> ImportResponse:
    """BetterCad's build steps .zip, waiting for the author to bring it in."""
    await _own_project(session, project_id, user)
    if source != "bettercad":
        raise HTTPException(status.HTTP_400_BAD_REQUEST, detail="Only BetterCad build steps can be sent.")
    settings = get_settings()
    data = await file.read()
    if len(data) > settings.max_project_storage:
        raise HTTPException(
            status.HTTP_400_BAD_REQUEST,
            detail=f"The build steps are over the {settings.max_project_storage // (1024 * 1024)} MB a project can hold.",
        )
    try:
        bundle = bc.read_bundle(data, max_image=settings.max_image_size, max_file=settings.max_file_size)
    except bc.BundleError as e:
        raise HTTPException(status.HTTP_400_BAD_REQUEST, detail=str(e))
    if model_url and not model_url.startswith("https://"):
        model_url = None

    path = save_file(data, generate_filename("bettercad.zip", project_id), project_id, "imports")
    if path is None:
        raise HTTPException(status.HTTP_500_INTERNAL_SERVER_ERROR, detail="The build steps couldn't be stored.")

    # Only the newest send waits; an older one it replaces is dropped.
    older = (
        await session.scalars(
            select(ProjectImport).where(
                ProjectImport.project_id == project_id,
                ProjectImport.source == source,
                ProjectImport.status == "pending",
            )
        )
    ).all()
    for row in older:
        row.status = "superseded"
        delete_file(row.archive_path)

    row = ProjectImport(
        project_id=project_id,
        source=source,
        status="pending",
        title=bundle.title,
        steps=bundle.steps,
        pictures=len(bundle.images),
        files=len(bundle.files),
        model_url=model_url,
        archive_path=path,
        archive_size=len(data),
    )
    session.add(row)
    await session.commit()
    await session.refresh(row)
    return await _response(session, row)


@router.get("/pending", response_model=Optional[ImportResponse])
async def pending_import(
    project_id: int,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> Optional[ImportResponse]:
    """The import waiting to be brought in, or ``null``."""
    await _own_project(session, project_id, user)
    row = await session.scalar(
        select(ProjectImport)
        .where(ProjectImport.project_id == project_id, ProjectImport.status == "pending")
        .order_by(ProjectImport.id.desc())
        .limit(1)
    )
    return await _response(session, row) if row else None


async def _pending(session: AsyncSession, project_id: int, import_id: int) -> ProjectImport:
    row = await session.get(ProjectImport, import_id)
    if row is None or row.project_id != project_id:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="There's no import with that id.")
    if row.status != "pending":
        raise HTTPException(status.HTTP_409_CONFLICT, detail="That import has already been dealt with.")
    return row


MODEL_FILE = re.compile(r"\.model\.json$", re.I)
MODEL_DESCRIPTION = (
    "The project's 3D model: the parts and joints its assembly pictures are drawn from (made in a graphic's 3D scene)."
)


def _model_file_name(project: Project) -> str:
    """As the editor names it (project-editor.js ``modelFileName``)."""
    slug = re.sub(r"[^a-z0-9]+", "-", (project.slug or project.title or "robot").lower()).strip("-")[:60]
    return (slug or "robot") + ".model.json"


async def _put_in_model(session: AsyncSession, project: Project, parts: list[dict]) -> int:
    """Puts BetterCad's parts into the project's 3D model file, made if there
    isn't one, replacing the old file as the editor does when it saves."""
    if not parts:
        return 0
    old_files = (
        await session.scalars(
            select(ProjectFile)
            .where(ProjectFile.project_id == project.id, ProjectFile.removed_at.is_(None))
            .order_by(ProjectFile.id.desc())
        )
    ).all()
    old_files = [f for f in old_files if MODEL_FILE.search(f.filename or "")]
    old = None
    if old_files:
        raw = read_file(old_files[0].file_path)
        try:
            old = json.loads(raw) if raw else None
        except ValueError:
            old = None
    model = bc.merge_model(old, parts, int(time.time() * 1000))
    content = json.dumps(model, separators=(",", ":")).encode("utf-8")
    name = _model_file_name(project)
    stored, packed = compress_for_storage(name, content)
    filename = generate_filename(name, project.id)
    stored_path = save_file(stored, filename + STORED_GZ if packed else filename, project.id, "files")
    if stored_path is None:
        raise HTTPException(status.HTTP_500_INTERNAL_SERVER_ERROR, detail="The 3D model couldn't be stored.")
    session.add(ProjectFile(
        project_id=project.id,
        filename=name,
        file_path=stored_path,
        file_size=len(content),
        stored_size=len(stored) if packed else None,
        file_type=file_category(name),
        description=MODEL_DESCRIPTION,
    ))
    for f in old_files:
        if await v.guard_removal(session, project, "files", f):
            delete_file(f.file_path)
            await session.delete(f)
    await session.flush()
    return len(parts)


async def _remove_last(session: AsyncSession, project: Project, last: Optional[ProjectImport]) -> None:
    """Takes the pictures and files the last import added out of the
    working copy, keeping any a version still shows (issue #439)."""
    if last is None:
        return
    for model, key, ids in ((ProjectImage, "images", last.image_ids), (ProjectFile, "files", last.file_ids)):
        for row_id in ids or []:
            row = await session.get(model, row_id)
            if row is None or row.project_id != project.id or row.removed_at is not None:
                continue
            if await v.guard_removal(session, project, key, row):
                delete_file(row.file_path)
                if getattr(row, "thumbnail_path", None):
                    delete_file(row.thumbnail_path)
                await session.delete(row)
    await session.flush()


@router.post("/{import_id}/apply", response_model=ImportApplied, response_model_by_alias=True)
async def apply_import(
    project_id: int,
    import_id: int,
    body: ImportApply,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> ImportApplied:
    project = await _own_project(session, project_id, user)
    row = await _pending(session, project_id, import_id)
    settings = get_settings()

    data = read_file(row.archive_path)
    if data is None:
        raise HTTPException(status.HTTP_410_GONE, detail="The build steps are gone. Send them again from BetterCad.")
    try:
        bundle = bc.read_bundle(data, max_image=settings.max_image_size, max_file=settings.max_file_size)
    except bc.BundleError as e:
        raise HTTPException(status.HTTP_400_BAD_REQUEST, detail=str(e))

    last = await _last_applied(session, project_id, row.source)
    await _remove_last(session, project, last)

    stored_files = []
    for path, content in bundle.files.items():
        stored, packed = compress_for_storage(path, content)
        stored_files.append((path, content, stored, packed))
    if await storage_used(session, project_id) + sum(len(s) for _, _, s, _ in stored_files) > settings.max_project_storage:
        await session.rollback()
        raise HTTPException(
            status.HTTP_400_BAD_REQUEST,
            detail=f"The model files don't fit in the project's {settings.max_project_storage // (1024 * 1024)} MB.",
        )

    base = settings.public_base_url.rstrip("/")
    urls: dict[str, str] = {}
    image_ids: list[int] = []
    file_ids: list[int] = []
    sort = await session.scalar(
        select(func.max(ProjectImage.sort_order)).where(ProjectImage.project_id == project_id)
    )
    sort = -1 if sort is None else sort
    for path, content in bundle.images.items():
        name = posixpath.basename(path)
        stored_path = save_file(content, generate_filename(name, project_id), project_id, "images")
        if stored_path is None:
            raise HTTPException(status.HTTP_500_INTERNAL_SERVER_ERROR, detail="A picture couldn't be stored.")
        sort += 1
        image = ProjectImage(
            project_id=project_id,
            filename=f"{bundle.title} {name}"[:255],
            file_path=stored_path,
            sort_order=sort,
        )
        session.add(image)
        await session.flush()
        image_ids.append(image.id)
        urls[path] = f"{base}/api/projects/{project_id}/images/{image.id}/view"

    for path, content, stored, packed in stored_files:
        name = posixpath.basename(path)
        filename = generate_filename(name, project_id)
        stored_path = save_file(stored, filename + STORED_GZ if packed else filename, project_id, "files")
        if stored_path is None:
            raise HTTPException(status.HTTP_500_INTERNAL_SERVER_ERROR, detail="A model file couldn't be stored.")
        record = ProjectFile(
            project_id=project_id,
            filename=name,
            file_path=stored_path,
            file_size=len(content),
            stored_size=len(stored) if packed else None,
            file_type=file_category(name),
            description="From BetterCad" if path.endswith(".bcad") else None,
        )
        session.add(record)
        await session.flush()
        file_ids.append(record.id)
        urls[path] = f"{base}/api/projects/{project_id}/files/{record.id}/download"

    content_md = bc.splice(body.content_md, bc.section(bundle.readme, urls.get))
    parts = await _put_in_model(session, project, bc.model_parts(bundle, urls.get))

    row.status = "applied"
    row.applied_at = _now()
    row.image_ids = image_ids
    row.file_ids = file_ids
    delete_file(row.archive_path)
    await session.commit()
    await session.refresh(row)
    resp = await _response(session, row)
    resp.is_update = last is not None
    return ImportApplied(content_md=content_md, import_=resp, pictures=len(image_ids), files=len(file_ids), parts=parts)


@router.post("/{import_id}/dismiss", response_model=ImportResponse)
async def dismiss_import(
    project_id: int,
    import_id: int,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> ImportResponse:
    await _own_project(session, project_id, user)
    row = await _pending(session, project_id, import_id)
    row.status = "dismissed"
    delete_file(row.archive_path)
    await session.commit()
    await session.refresh(row)
    return await _response(session, row)
