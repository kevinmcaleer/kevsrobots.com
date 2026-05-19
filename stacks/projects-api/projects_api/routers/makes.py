"""Community Makes ("I Made This!") endpoints — issue #107.

A `Make` is a user post on someone else's project that says "I built this!".
It carries optional notes + modifications text and up to 5 images.

Permissions:
  * Anyone authenticated may post a make on any project.
  * Only the make's `user_id` (the poster) may delete it.
  * Only the project's `author_username` may heart/unheart a make.

Out of scope here (see #106):
  * Badge evaluation hook — TODO markers are left where a Make is
    created and deleted so that subsystem can attach later without
    a refactor.
  * Activity feed integration.
"""

from __future__ import annotations

from datetime import datetime, timezone
from typing import Optional

from fastapi import APIRouter, Depends, Form, HTTPException, UploadFile, status
from fastapi.responses import Response
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import get_current_user, require_terms_accepted
from ..badges import evaluate_user
from ..config import get_settings
from ..db import get_session
from ..models import Make, MakeImage, Project
from ..schemas import MakeImageResponse, MakeResponse
from ..storage import (
    ALLOWED_IMAGE_EXTENSIONS,
    delete_file,
    generate_filename,
    read_file,
    save_file,
)
from .users import log_activity  # Issue #111: profile activity feed.

MAX_IMAGES_PER_MAKE = 5

# Two routers because makes span three URL spaces:
#   /api/projects/{project_id}/makes  (list + create)
#   /api/users/{username}/makes        (user-scoped list)
#   /api/makes/{make_id}/...           (detail / delete / heart / image view)
router = APIRouter(tags=["makes"])


def _serialize_image(img: MakeImage) -> MakeImageResponse:
    return MakeImageResponse(
        id=img.id,
        filename=img.filename,
        caption=img.caption,
        sort_order=img.sort_order,
        file_size=img.file_size or 0,
    )


def _serialize_make(
    make: Make,
    images: list[MakeImage],
    project_author: Optional[str] = None,
    project_title: Optional[str] = None,
) -> MakeResponse:
    return MakeResponse(
        id=make.id,
        project_id=make.project_id,
        user_id=make.user_id,
        created_at=make.created_at,
        notes=make.notes,
        modifications=make.modifications,
        images=[_serialize_image(i) for i in images],
        hearted_by_author=make.hearted_at is not None,
        project_title=project_title,
    )


async def _load_images_for(
    session: AsyncSession, make_ids: list[int]
) -> dict[int, list[MakeImage]]:
    if not make_ids:
        return {}
    rows = (
        await session.scalars(
            select(MakeImage)
            .where(MakeImage.make_id.in_(make_ids))
            .order_by(MakeImage.sort_order, MakeImage.id)
        )
    ).all()
    grouped: dict[int, list[MakeImage]] = {mid: [] for mid in make_ids}
    for img in rows:
        grouped.setdefault(img.make_id, []).append(img)
    return grouped


# --- Create / list under a project ----------------------------------------


@router.post(
    "/api/projects/{project_id}/makes",
    response_model=MakeResponse,
    status_code=201,
)
async def create_make(
    project_id: int,
    notes: Optional[str] = Form(default=None),
    modifications: Optional[str] = Form(default=None),
    images: list[UploadFile] = [],  # noqa: B006 — FastAPI handles default
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> MakeResponse:
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND)

    # Filter out the empty UploadFile FastAPI sometimes synthesises when
    # the client sends the multipart field with no actual file attached.
    real_images = [
        f for f in (images or [])
        if f is not None and getattr(f, "filename", None)
    ]
    if len(real_images) > MAX_IMAGES_PER_MAKE:
        raise HTTPException(
            status.HTTP_400_BAD_REQUEST,
            detail=f"Too many images (max {MAX_IMAGES_PER_MAKE})",
        )

    # Validate every image up-front before we write anything so a bad file
    # at index 4 doesn't leave us with orphans on disk.
    settings = get_settings()
    validated: list[tuple[UploadFile, bytes]] = []
    for f in real_images:
        content = await f.read()
        ext = (
            "." + f.filename.rsplit(".", 1)[-1].lower()
            if "." in f.filename
            else ""
        )
        if ext not in ALLOWED_IMAGE_EXTENSIONS:
            raise HTTPException(
                status.HTTP_400_BAD_REQUEST,
                detail=f"Image type {ext} not allowed",
            )
        if len(content) > settings.max_image_size:
            raise HTTPException(
                status.HTTP_400_BAD_REQUEST, detail="Image too large"
            )
        validated.append((f, content))

    make = Make(
        project_id=project_id,
        user_id=user,
        notes=notes,
        modifications=modifications,
    )
    session.add(make)
    await session.flush()  # need make.id for image rows / storage paths

    saved_images: list[MakeImage] = []
    for idx, (f, content) in enumerate(validated):
        # Reuse the storage layout: makes/<make_id>/<filename>. We pass
        # make.id as the "project_id" arg because the helper only uses it
        # to namespace the directory.
        stored_name = generate_filename(f.filename, make.id)
        path = save_file(content, stored_name, make.id, "makes")
        if path is None:
            # Best-effort clean up of anything we already wrote.
            for prior in saved_images:
                delete_file(prior.file_path)
            raise HTTPException(
                status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Failed to save image",
            )
        img = MakeImage(
            make_id=make.id,
            filename=f.filename,
            file_path=path,
            file_size=len(content),
            sort_order=idx,
        )
        session.add(img)
        saved_images.append(img)

    # Issue #111: log the make as an activity event before commit so it
    # rolls back with the rest if anything fails downstream.
    await log_activity(
        session,
        user_id=user,
        kind="make_posted",
        subject_id=make.id,
        subject_title=project.title,
        subject_url=f"/projects/view.html?id={project_id}#makes",
    )

    await session.commit()
    await session.refresh(make)
    for img in saved_images:
        await session.refresh(img)

    # Issue #106: evaluate badges for BOTH the make poster (could be
    # crossing future "first make" thresholds — none defined today but the
    # hook is here for symmetry) AND the project's author, who could be
    # crossing the Community Builder tier. Best-effort.
    poster_newly = []
    try:
        poster_newly = await evaluate_user(session, user)
    except Exception:  # noqa: BLE001
        pass
    try:
        # Only re-evaluate the author if they're a different user — saves a
        # roundtrip for self-makes (which are rare but possible).
        if project.author_username != user:
            await evaluate_user(session, project.author_username)
    except Exception:  # noqa: BLE001
        pass

    resp = _serialize_make(make, saved_images)
    resp.newly_awarded_badges = poster_newly
    return resp


@router.get(
    "/api/projects/{project_id}/makes",
    response_model=list[MakeResponse],
)
async def list_makes_for_project(
    project_id: int,
    session: AsyncSession = Depends(get_session),
) -> list[MakeResponse]:
    makes = (
        await session.scalars(
            select(Make)
            .where(Make.project_id == project_id)
            .order_by(Make.created_at.desc(), Make.id.desc())
        )
    ).all()
    img_map = await _load_images_for(session, [m.id for m in makes])
    return [_serialize_make(m, img_map.get(m.id, [])) for m in makes]


# --- User-scoped list -----------------------------------------------------


@router.get("/api/users/{username}/makes", response_model=list[MakeResponse])
async def list_makes_for_user(
    username: str,
    session: AsyncSession = Depends(get_session),
) -> list[MakeResponse]:
    makes = (
        await session.scalars(
            select(Make)
            .where(Make.user_id == username)
            .order_by(Make.created_at.desc(), Make.id.desc())
        )
    ).all()
    img_map = await _load_images_for(session, [m.id for m in makes])
    # Decorate with project title so the user-profile UI can render a
    # "for <project>" line without a follow-up fetch per row.
    title_map: dict[int, str] = {}
    if makes:
        project_ids = list({m.project_id for m in makes})
        projects = (
            await session.scalars(
                select(Project).where(Project.id.in_(project_ids))
            )
        ).all()
        title_map = {p.id: p.title for p in projects}
    return [
        _serialize_make(
            m,
            img_map.get(m.id, []),
            project_title=title_map.get(m.project_id),
        )
        for m in makes
    ]


# --- Single make detail / delete / heart ---------------------------------


@router.get("/api/makes/{make_id}", response_model=MakeResponse)
async def get_make(
    make_id: int,
    session: AsyncSession = Depends(get_session),
) -> MakeResponse:
    make = await session.get(Make, make_id)
    if make is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    img_map = await _load_images_for(session, [make.id])
    project = await session.get(Project, make.project_id)
    return _serialize_make(
        make,
        img_map.get(make.id, []),
        project_title=project.title if project else None,
    )


@router.delete("/api/makes/{make_id}", status_code=204)
async def delete_make(
    make_id: int,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> None:
    make = await session.get(Make, make_id)
    if make is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    if make.user_id != user:
        raise HTTPException(status.HTTP_403_FORBIDDEN)

    # Clean up image blobs from NAS/local before the rows go away. We
    # ignore individual failures — orphaned files are recoverable, lost
    # rows are not.
    image_rows = (
        await session.scalars(
            select(MakeImage).where(MakeImage.make_id == make_id)
        )
    ).all()
    for img in image_rows:
        try:
            delete_file(img.file_path)
        except Exception:  # noqa: BLE001 — best-effort
            pass

    await session.delete(make)
    await session.commit()

    # Note: we intentionally do NOT un-award badges on make deletion.
    # Badges are sticky — earning one and then losing it because someone
    # deleted a make would be a bad experience. Re-evaluation only adds.


@router.post("/api/makes/{make_id}/heart", response_model=MakeResponse)
async def heart_make(
    make_id: int,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> MakeResponse:
    make = await session.get(Make, make_id)
    if make is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    project = await session.get(Project, make.project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    if project.author_username != user:
        raise HTTPException(
            status.HTTP_403_FORBIDDEN,
            detail="Only the project author can heart a make",
        )
    make.hearted_at = datetime.now(timezone.utc).replace(tzinfo=None)
    await session.commit()
    await session.refresh(make)
    img_map = await _load_images_for(session, [make.id])
    return _serialize_make(
        make, img_map.get(make.id, []), project_title=project.title
    )


@router.delete("/api/makes/{make_id}/heart", response_model=MakeResponse)
async def unheart_make(
    make_id: int,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> MakeResponse:
    make = await session.get(Make, make_id)
    if make is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    project = await session.get(Project, make.project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    if project.author_username != user:
        raise HTTPException(
            status.HTTP_403_FORBIDDEN,
            detail="Only the project author can unheart a make",
        )
    make.hearted_at = None
    await session.commit()
    await session.refresh(make)
    img_map = await _load_images_for(session, [make.id])
    return _serialize_make(
        make, img_map.get(make.id, []), project_title=project.title
    )


# --- Image view (binary) -------------------------------------------------


@router.get("/api/makes/{make_id}/images/{image_id}/view")
async def view_make_image(
    make_id: int,
    image_id: int,
    session: AsyncSession = Depends(get_session),
) -> Response:
    img = await session.get(MakeImage, image_id)
    if img is None or img.make_id != make_id:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    data = read_file(img.file_path)
    if data is None:
        raise HTTPException(
            status.HTTP_404_NOT_FOUND, detail="Image not found in storage"
        )
    ext = (
        img.filename.rsplit(".", 1)[-1].lower()
        if "." in img.filename
        else "png"
    )
    media_types = {
        "png": "image/png",
        "jpg": "image/jpeg",
        "jpeg": "image/jpeg",
        "gif": "image/gif",
        "webp": "image/webp",
        "svg": "image/svg+xml",
    }
    return Response(
        content=data,
        media_type=media_types.get(ext, "application/octet-stream"),
    )
