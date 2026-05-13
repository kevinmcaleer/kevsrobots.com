"""File upload/download endpoints."""

from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException, UploadFile, status
from fastapi.responses import Response
from sqlalchemy import func, select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import get_current_user
from ..config import get_settings
from ..db import get_session
from ..models import Project, ProjectFile
from ..schemas import FileResponse
from ..storage import (
    delete_file,
    generate_filename,
    is_image,
    read_file,
    save_file,
    validate_file,
)

router = APIRouter(prefix="/api/projects/{project_id}/files", tags=["files"])


@router.get("", response_model=list[FileResponse])
async def list_files(
    project_id: int,
    session: AsyncSession = Depends(get_session),
) -> list[FileResponse]:
    items = (
        await session.scalars(
            select(ProjectFile)
            .where(ProjectFile.project_id == project_id)
            .order_by(ProjectFile.uploaded_at.desc())
        )
    ).all()
    return [FileResponse.model_validate(i, from_attributes=True) for i in items]


@router.post("", response_model=FileResponse, status_code=201)
async def upload_file(
    project_id: int,
    file: UploadFile,
    user: str = Depends(get_current_user),
    session: AsyncSession = Depends(get_session),
) -> FileResponse:
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    if project.author_username != user:
        raise HTTPException(status.HTTP_403_FORBIDDEN)

    settings = get_settings()
    content = await file.read()

    ok, err = validate_file(file.filename, content, settings.max_file_size)
    if not ok:
        raise HTTPException(status.HTTP_400_BAD_REQUEST, detail=err)

    # Check total storage for this project.
    total = await session.scalar(
        select(func.coalesce(func.sum(ProjectFile.file_size), 0))
        .where(ProjectFile.project_id == project_id)
    ) or 0
    if total + len(content) > settings.max_project_storage:
        raise HTTPException(
            status.HTTP_400_BAD_REQUEST,
            detail=f"Project storage limit exceeded ({settings.max_project_storage // (1024*1024)}MB)",
        )

    filename = generate_filename(file.filename, project_id)
    file_type = "images" if is_image(file.filename) else "files"
    path = save_file(content, filename, project_id, file_type)
    if path is None:
        raise HTTPException(status.HTTP_500_INTERNAL_SERVER_ERROR, detail="Failed to save file")

    ext = file.filename.rsplit(".", 1)[-1].lower() if "." in file.filename else "unknown"
    record = ProjectFile(
        project_id=project_id,
        filename=file.filename,
        file_path=path,
        file_size=len(content),
        file_type=ext,
    )
    session.add(record)
    await session.commit()
    await session.refresh(record)
    return FileResponse.model_validate(record, from_attributes=True)


@router.get("/{file_id}/download")
async def download_file(
    project_id: int,
    file_id: int,
    session: AsyncSession = Depends(get_session),
) -> Response:
    record = await session.get(ProjectFile, file_id)
    if record is None or record.project_id != project_id:
        raise HTTPException(status.HTTP_404_NOT_FOUND)

    data = read_file(record.file_path)
    if data is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="File not found in storage")

    return Response(
        content=data,
        media_type="application/octet-stream",
        headers={"Content-Disposition": f'attachment; filename="{record.filename}"'},
    )


@router.delete("/{file_id}", status_code=204)
async def delete_file_endpoint(
    project_id: int,
    file_id: int,
    user: str = Depends(get_current_user),
    session: AsyncSession = Depends(get_session),
) -> None:
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    if project.author_username != user:
        raise HTTPException(status.HTTP_403_FORBIDDEN)

    record = await session.get(ProjectFile, file_id)
    if record is None or record.project_id != project_id:
        raise HTTPException(status.HTTP_404_NOT_FOUND)

    delete_file(record.file_path)
    await session.delete(record)
    await session.commit()
