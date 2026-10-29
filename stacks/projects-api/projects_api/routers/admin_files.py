"""Admin backfill of 3D model thumbnails (issue #262).

Thumbnails are rendered in the browser at upload time. Models uploaded
before that have none, so the admin page at ``/admin/projects/thumbnails/``
lists them, reads each model, renders it and posts the PNG back here.
Drafts are included: these routes are admin-only.
"""

from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException, UploadFile, status
from fastapi.responses import Response
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import require_admin
from ..db import get_session
from ..models import Project, ProjectFile
from ..schemas import FileResponse, ModelMissingThumbnail
from ..storage import file_category, read_file
from .files import store_thumbnail

router = APIRouter(prefix="/api/admin/files", tags=["admin-files"])


@router.get("/missing-thumbnails", response_model=list[ModelMissingThumbnail])
async def list_models_missing_thumbnails(
    _admin: str = Depends(require_admin),
    session: AsyncSession = Depends(get_session),
) -> list[ModelMissingThumbnail]:
    rows = (
        await session.execute(
            select(ProjectFile, Project.title)
            .join(Project, Project.id == ProjectFile.project_id)
            .where(ProjectFile.thumbnail_path.is_(None))
            .order_by(ProjectFile.id)
        )
    ).all()
    # The category comes from the extension (legacy rows stored it as
    # file_type), so filter here rather than in SQL.
    return [
        ModelMissingThumbnail(
            file_id=f.id, project_id=f.project_id, project_title=title,
            filename=f.filename, file_size=f.file_size,
        )
        for f, title in rows
        if file_category(f.filename) == "model"
    ]


async def _model(session: AsyncSession, file_id: int) -> ProjectFile:
    record = await session.get(ProjectFile, file_id)
    if record is None or file_category(record.filename) != "model":
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    return record


@router.get("/{file_id}/model")
async def read_model(
    file_id: int,
    _admin: str = Depends(require_admin),
    session: AsyncSession = Depends(get_session),
) -> Response:
    """The model's bytes for rendering, without counting a download."""
    record = await _model(session, file_id)
    data = read_file(record.file_path)
    if data is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="File not found in storage")
    return Response(
        content=data, media_type="application/octet-stream",
        headers={"Cache-Control": "private, no-store"},
    )


@router.post("/{file_id}/thumbnail", response_model=FileResponse)
async def upload_model_thumbnail(
    file_id: int,
    file: UploadFile,
    _admin: str = Depends(require_admin),
    session: AsyncSession = Depends(get_session),
) -> FileResponse:
    return await store_thumbnail(session, await _model(session, file_id), file)
