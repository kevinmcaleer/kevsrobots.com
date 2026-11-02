"""Image upload endpoints."""

from __future__ import annotations

from typing import Optional

from fastapi import APIRouter, Depends, HTTPException, Query, UploadFile, status
from fastapi.responses import Response
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import get_current_user, get_optional_user, require_terms_accepted
from ..config import get_settings
from ..db import get_session
from ..models import Project, ProjectImage
from ..visibility import NO_STORE
from .. import versions as v
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
    user: Optional[str] = Depends(get_optional_user),
    version: Optional[str] = Query(None, description="Epic #433: live, working or a number"),
) -> list[ImageResponse]:
    """The gallery: the working copy's (removed pictures left out) for the
    author, the live version's for everyone else (epic #433)."""
    view = await v.resolve_view(session, project_id, user, version)
    if view.snapshot is None:
        items = (
            await session.scalars(
                select(ProjectImage)
                .where(ProjectImage.project_id == project_id, ProjectImage.removed_at.is_(None))
                .order_by(ProjectImage.sort_order)
            )
        ).all()
        return [ImageResponse.model_validate(i, from_attributes=True) for i in items]
    wanted = view.image_ids() or set()
    rows = (
        await session.scalars(
            select(ProjectImage).where(ProjectImage.project_id == project_id, ProjectImage.id.in_(wanted))
        )
    ).all() if wanted else []
    out = []
    for i in rows:
        resp = ImageResponse.model_validate(i, from_attributes=True)
        entry = view.entry("images", i.id) or {}
        resp.caption = entry.get("caption", resp.caption)
        resp.sort_order = entry.get("sort_order", resp.sort_order) or 0
        out.append(resp)
    out.sort(key=lambda r: (r.sort_order, r.id))
    return out


@router.post("", response_model=ImageResponse, status_code=201)
async def upload_image(
    project_id: int,
    file: UploadFile,
    user: str = Depends(require_terms_accepted),
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
    user: Optional[str] = Depends(get_optional_user),
) -> Response:
    record = await session.get(ProjectImage, image_id)
    if record is None or record.project_id != project_id:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    # Issue #260 / #435: a picture the live page doesn't show (a draft's,
    # or one only the working copy or an older version has) is the
    # author's alone. <img> requests carry the .kevsrobots.com
    # access_token cookie, so this works on the site; tell caches not to
    # keep a copy for anyone else.
    project = await session.get(Project, project_id)
    public = await v.item_public(session, project, "images", image_id)
    if not public and user != project.author_username:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Image not found", headers=NO_STORE)
    draft = not public

    data = read_file(record.file_path)
    if data is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Image not found in storage")

    ext = record.filename.rsplit(".", 1)[-1].lower() if "." in record.filename else "png"
    media_types = {"png": "image/png", "jpg": "image/jpeg", "jpeg": "image/jpeg",
                   "gif": "image/gif", "webp": "image/webp", "svg": "image/svg+xml"}
    return Response(
        content=data,
        media_type=media_types.get(ext, "application/octet-stream"),
        headers={"Cache-Control": "private, no-store"} if draft else None,
    )


@router.put("/{image_id}", response_model=ImageResponse)
async def update_image(
    project_id: int,
    image_id: int,
    body: dict,
    user: str = Depends(require_terms_accepted),
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
    user: str = Depends(require_terms_accepted),
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

    # Issue #439: never break the live page; keep what older versions use.
    if await v.guard_removal(session, project, "images", record):
        delete_file(record.file_path)
        await session.delete(record)
    await session.commit()
