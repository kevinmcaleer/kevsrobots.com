"""Part photo gallery endpoints (part-hub).

Multiple photos per part, each with an optional title (orientation note).
Two sources: uploaded to NAS/local storage (first-class, reducing reliance
on external links) or an off-site URL. Mirrors the ``ProjectImage`` upload
pattern (``storage.save_file`` + a ``/view`` byte-serving route) but adds
the link option and a per-photo title.

Gating mirrors the parts wiki: any logged-in account past the 14-day age
gate can add photos (community enhancement is encouraged). Editing /
deleting a photo is restricted to its uploader or an admin to blunt
casual vandalism.
"""

from __future__ import annotations

from typing import Optional

from fastapi import APIRouter, Depends, Form, HTTPException, UploadFile, status
from fastapi.responses import Response
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import require_terms_accepted_aged
from ..config import get_settings
from ..db import get_session
from ..models import Part, PartPhoto
from ..schemas import PartPhotoLinkCreate, PartPhotoResponse, PartPhotoUpdate
from ..storage import (
    ALLOWED_IMAGE_EXTENSIONS,
    delete_file,
    generate_filename,
    read_file,
    save_file,
)

router = APIRouter(prefix="/api/parts/{slug}/photos", tags=["part-photos"])

_MEDIA_TYPES = {
    "png": "image/png", "jpg": "image/jpeg", "jpeg": "image/jpeg",
    "gif": "image/gif", "webp": "image/webp", "svg": "image/svg+xml",
}


def _is_admin(user: str) -> bool:
    return user in get_settings().admin_usernames_list


def _photo_response(slug: str, photo: PartPhoto) -> PartPhotoResponse:
    """Build the API view. ``url`` is render-ready: the /view route for an
    upload (relative — the frontend prefixes the API base), or the external
    URL as-is for a link."""
    if photo.external_url:
        url = photo.external_url
        is_external = True
    else:
        url = f"/api/parts/{slug}/photos/{photo.id}/view"
        is_external = False
    return PartPhotoResponse(
        id=photo.id,
        title=photo.title,
        url=url,
        is_external=is_external,
        sort_order=photo.sort_order,
        created_by=photo.created_by,
        created_at=photo.created_at,
    )


async def _get_part(session: AsyncSession, slug: str) -> Part:
    part = await session.scalar(select(Part).where(Part.slug == slug))
    if part is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Part not found")
    return part


async def _get_photo(session: AsyncSession, part: Part, photo_id: int) -> PartPhoto:
    photo = await session.get(PartPhoto, photo_id)
    if photo is None or photo.part_id != part.id:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Photo not found")
    return photo


async def _next_sort_order(session: AsyncSession, part_id: int) -> int:
    rows = (
        await session.scalars(
            select(PartPhoto.sort_order).where(PartPhoto.part_id == part_id)
        )
    ).all()
    return (max(rows) + 1) if rows else 0


@router.get("", response_model=list[PartPhotoResponse])
async def list_photos(
    slug: str,
    session: AsyncSession = Depends(get_session),
) -> list[PartPhotoResponse]:
    part = await _get_part(session, slug)
    photos = (
        await session.scalars(
            select(PartPhoto)
            .where(PartPhoto.part_id == part.id)
            .order_by(PartPhoto.sort_order.asc(), PartPhoto.id.asc())
        )
    ).all()
    return [_photo_response(slug, p) for p in photos]


@router.post("", response_model=PartPhotoResponse, status_code=201)
async def upload_photo(
    slug: str,
    file: UploadFile,
    title: Optional[str] = Form(None),
    user: str = Depends(require_terms_accepted_aged),
    session: AsyncSession = Depends(get_session),
) -> PartPhotoResponse:
    """Upload an image file as a part photo (stored on NAS/local)."""
    part = await _get_part(session, slug)
    settings = get_settings()
    content = await file.read()

    ext = "." + file.filename.rsplit(".", 1)[-1].lower() if "." in (file.filename or "") else ""
    if ext not in ALLOWED_IMAGE_EXTENSIONS:
        raise HTTPException(status.HTTP_400_BAD_REQUEST, detail=f"Image type {ext} not allowed")
    if len(content) > settings.max_image_size:
        raise HTTPException(status.HTTP_400_BAD_REQUEST, detail="Image too large")

    filename = generate_filename(file.filename, part.id)
    path = save_file(content, filename, part.id, "part_images")
    if path is None:
        raise HTTPException(status.HTTP_500_INTERNAL_SERVER_ERROR, detail="Failed to save photo")

    photo = PartPhoto(
        part_id=part.id,
        title=(title or None),
        file_path=path,
        filename=file.filename,
        sort_order=await _next_sort_order(session, part.id),
        created_by=user,
    )
    session.add(photo)
    await session.commit()
    await session.refresh(photo)
    return _photo_response(slug, photo)


@router.post("/link", response_model=PartPhotoResponse, status_code=201)
async def link_photo(
    slug: str,
    body: PartPhotoLinkCreate,
    user: str = Depends(require_terms_accepted_aged),
    session: AsyncSession = Depends(get_session),
) -> PartPhotoResponse:
    """Add an off-site image by URL (no upload)."""
    part = await _get_part(session, slug)
    url = body.external_url.strip()
    if not (url.startswith("http://") or url.startswith("https://")):
        raise HTTPException(
            status.HTTP_400_BAD_REQUEST,
            detail="external_url must be an http(s) URL",
        )
    photo = PartPhoto(
        part_id=part.id,
        title=(body.title or None),
        external_url=url,
        sort_order=await _next_sort_order(session, part.id),
        created_by=user,
    )
    session.add(photo)
    await session.commit()
    await session.refresh(photo)
    return _photo_response(slug, photo)


@router.get("/{photo_id}/view")
async def view_photo(
    slug: str,
    photo_id: int,
    session: AsyncSession = Depends(get_session),
) -> Response:
    part = await _get_part(session, slug)
    photo = await _get_photo(session, part, photo_id)
    if not photo.file_path:
        # Linked photos live off-site — nothing to serve here.
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="No stored image for this photo")
    data = read_file(photo.file_path)
    if data is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Image not found in storage")
    ext = (photo.filename or "").rsplit(".", 1)[-1].lower() if "." in (photo.filename or "") else "png"
    return Response(content=data, media_type=_MEDIA_TYPES.get(ext, "application/octet-stream"))


@router.put("/{photo_id}", response_model=PartPhotoResponse)
async def update_photo(
    slug: str,
    photo_id: int,
    body: PartPhotoUpdate,
    user: str = Depends(require_terms_accepted_aged),
    session: AsyncSession = Depends(get_session),
) -> PartPhotoResponse:
    part = await _get_part(session, slug)
    photo = await _get_photo(session, part, photo_id)
    if photo.created_by != user and not _is_admin(user):
        raise HTTPException(
            status.HTTP_403_FORBIDDEN,
            detail="Only the uploader or an admin can edit this photo",
        )
    if body.title is not None:
        photo.title = body.title.strip() or None
    if body.sort_order is not None:
        photo.sort_order = body.sort_order
    await session.commit()
    await session.refresh(photo)
    return _photo_response(slug, photo)


@router.delete("/{photo_id}", status_code=204)
async def delete_photo(
    slug: str,
    photo_id: int,
    user: str = Depends(require_terms_accepted_aged),
    session: AsyncSession = Depends(get_session),
) -> None:
    part = await _get_part(session, slug)
    photo = await _get_photo(session, part, photo_id)
    if photo.created_by != user and not _is_admin(user):
        raise HTTPException(
            status.HTTP_403_FORBIDDEN,
            detail="Only the uploader or an admin can delete this photo",
        )
    if photo.file_path:
        delete_file(photo.file_path)
    await session.delete(photo)
    await session.commit()
