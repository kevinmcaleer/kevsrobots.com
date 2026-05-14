"""Image upload endpoints."""

from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException, UploadFile, status
from fastapi.responses import Response
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import get_current_user
from ..config import get_settings
from ..db import get_session
from ..models import Project, ProjectImage
from ..schemas import ImageResponse
from ..storage import (
    ALLOWED_IMAGE_EXTENSIONS,
    delete_file,
    generate_filename,
    read_file,
    save_file,
)

router = APIRouter(prefix="/api/projects/{project_id}/images", tags=["images"])


@router.get("", response_model=list[ImageResponse])
async def list_images(
    project_id: int,
    session: AsyncSession = Depends(get_session),
) -> list[ImageResponse]:
    items = (
        await session.scalars(
            select(ProjectImage)
            .where(ProjectImage.project_id == project_id)
            .order_by(ProjectImage.sort_order)
        )
    ).all()
    return [ImageResponse.model_validate(i, from_attributes=True) for i in items]


@router.post("", response_model=ImageResponse, status_code=201)
async def upload_image(
    project_id: int,
    file: UploadFile,
    user: str = Depends(get_current_user),
    session: AsyncSession = Depends(get_session),
) -> ImageResponse:
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    if project.author_username != user:
        raise HTTPException(status.HTTP_403_FORBIDDEN)

    settings = get_settings()
    content = await file.read()

    ext = "." + file.filename.rsplit(".", 1)[-1].lower() if "." in file.filename else ""
    if ext not in ALLOWED_IMAGE_EXTENSIONS:
        raise HTTPException(status.HTTP_400_BAD_REQUEST, detail=f"Image type {ext} not allowed")
    if len(content) > settings.max_image_size:
        raise HTTPException(status.HTTP_400_BAD_REQUEST, detail="Image too large")

    filename = generate_filename(file.filename, project_id)
    path = save_file(content, filename, project_id, "images")
    if path is None:
        raise HTTPException(status.HTTP_500_INTERNAL_SERVER_ERROR, detail="Failed to save image")

    record = ProjectImage(
        project_id=project_id,
        filename=file.filename,
        file_path=path,
    )
    session.add(record)
    await session.commit()
    await session.refresh(record)
    return ImageResponse.model_validate(record, from_attributes=True)


@router.get("/{image_id}/view")
async def view_image(
    project_id: int,
    image_id: int,
    session: AsyncSession = Depends(get_session),
) -> Response:
    record = await session.get(ProjectImage, image_id)
    if record is None or record.project_id != project_id:
        raise HTTPException(status.HTTP_404_NOT_FOUND)

    data = read_file(record.file_path)
    if data is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Image not found in storage")

    ext = record.filename.rsplit(".", 1)[-1].lower() if "." in record.filename else "png"
    media_types = {"png": "image/png", "jpg": "image/jpeg", "jpeg": "image/jpeg",
                   "gif": "image/gif", "webp": "image/webp", "svg": "image/svg+xml"}
    return Response(content=data, media_type=media_types.get(ext, "application/octet-stream"))


@router.put("/{image_id}", response_model=ImageResponse)
async def update_image(
    project_id: int,
    image_id: int,
    body: dict,
    user: str = Depends(get_current_user),
    session: AsyncSession = Depends(get_session),
) -> ImageResponse:
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    if project.author_username != user:
        raise HTTPException(status.HTTP_403_FORBIDDEN)
    record = await session.get(ProjectImage, image_id)
    if record is None or record.project_id != project_id:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    if "sort_order" in body:
        record.sort_order = body["sort_order"]
    if "caption" in body:
        record.caption = body["caption"]
    await session.commit()
    await session.refresh(record)
    return ImageResponse.model_validate(record, from_attributes=True)


@router.delete("/{image_id}", status_code=204)
async def delete_image(
    project_id: int,
    image_id: int,
    user: str = Depends(get_current_user),
    session: AsyncSession = Depends(get_session),
) -> None:
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    if project.author_username != user:
        raise HTTPException(status.HTTP_403_FORBIDDEN)

    record = await session.get(ProjectImage, image_id)
    if record is None or record.project_id != project_id:
        raise HTTPException(status.HTTP_404_NOT_FOUND)

    delete_file(record.file_path)
    await session.delete(record)
    await session.commit()
