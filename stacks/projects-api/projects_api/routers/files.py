"""File upload/download endpoints."""

from __future__ import annotations

import hashlib
import logging
from datetime import datetime, timedelta, timezone
from typing import Optional

from fastapi import APIRouter, Depends, HTTPException, Request, UploadFile, status
from fastapi.responses import Response
from sqlalchemy import func, select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import get_current_user, get_optional_user, require_terms_accepted
from ..config import get_settings
from ..db import get_session
from ..models import Download, Project, ProjectFile, ProjectImage
from ..visibility import hide_drafts
from ..schemas import FileResponse, FileUpdate, ImageResponse
from ..storage import (
    MAX_THUMBNAIL_SIZE,
    delete_file,
    file_category,
    generate_filename,
    is_image,
    is_png,
    read_file,
    save_file,
    validate_file,
)

logger = logging.getLogger(__name__)


def _client_ip(request: Request) -> str:
    """Best-effort client IP — honour standard reverse-proxy headers but
    never expose the raw IP anywhere; it's only used to compute ip_hash."""
    xff = request.headers.get("x-forwarded-for")
    if xff:
        return xff.split(",")[0].strip()
    real_ip = request.headers.get("x-real-ip")
    if real_ip:
        return real_ip.strip()
    if request.client and request.client.host:
        return request.client.host
    return "unknown"


def _hash_ip(ip: str, salt: str) -> str:
    return hashlib.sha256(f"{ip}{salt}".encode("utf-8")).hexdigest()


async def _log_download(
    session: AsyncSession,
    project_id: int,
    file_id: int,
    user_id: Optional[str],
    ip_hash: Optional[str],
) -> bool:
    """Insert a Download row if no matching row exists in the last 24h.

    Returns True when a new row was inserted, False when deduped.
    Identity for dedup: user_id if set, otherwise ip_hash.
    """
    cutoff = datetime.now(timezone.utc).replace(tzinfo=None) - timedelta(hours=24)
    query = select(Download.id).where(
        Download.project_id == project_id,
        Download.file_id == file_id,
        Download.downloaded_at >= cutoff,
    )
    if user_id:
        query = query.where(Download.user_id == user_id)
    elif ip_hash:
        query = query.where(
            Download.user_id.is_(None),
            Download.ip_hash == ip_hash,
        )
    else:
        # No identity at all — log it but it won't dedup
        pass

    existing = (await session.execute(query.limit(1))).scalar_one_or_none()
    if existing is not None:
        return False

    record = Download(
        project_id=project_id,
        file_id=file_id,
        user_id=user_id,
        ip_hash=ip_hash if not user_id else None,
    )
    session.add(record)
    await session.commit()
    # TODO(#106): Popular Project badge eval hook here — evaluate
    # download-count thresholds and award badges asynchronously.
    return True

def thumbnail_version(path: Optional[str]) -> Optional[str]:
    """A short tag that changes whenever a model gets a new thumbnail: the
    page puts it in the thumbnail URL, and it doubles as the ETag."""
    if not path:
        return None
    return hashlib.sha256(path.encode("utf-8")).hexdigest()[:16]


def file_response(record: ProjectFile, download_count: int = 0) -> FileResponse:
    """The API shape of a project file. ``file_type`` is worked out from the
    filename so legacy rows, which stored the extension, read as categories."""
    return FileResponse(
        id=record.id,
        filename=record.filename,
        file_size=record.file_size,
        file_type=file_category(record.filename),
        description=record.description,
        print_settings=record.print_settings,
        thumbnail_version=thumbnail_version(record.thumbnail_path),
        uploaded_at=record.uploaded_at,
        download_count=download_count,
    )


async def store_thumbnail(
    session: AsyncSession, record: ProjectFile, file: UploadFile
) -> FileResponse:
    """Save a browser-rendered PNG as ``record``'s thumbnail (issue #262),
    replacing any earlier one. Shared by the author and admin backfill routes."""
    if file_category(record.filename) != "model":
        raise HTTPException(status.HTTP_400_BAD_REQUEST, detail="Only 3D models have thumbnails")
    content = await file.read()
    if len(content) > MAX_THUMBNAIL_SIZE:
        raise HTTPException(status.HTTP_400_BAD_REQUEST, detail="Thumbnail too large")
    if not is_png(content):
        raise HTTPException(status.HTTP_400_BAD_REQUEST, detail="Thumbnail must be a PNG")
    stem = record.filename.rsplit(".", 1)[0]
    path = save_file(
        content, generate_filename(stem + ".png", record.project_id),
        record.project_id, "thumbnails",
    )
    if path is None:
        raise HTTPException(status.HTTP_500_INTERNAL_SERVER_ERROR, detail="Failed to save thumbnail")
    old = record.thumbnail_path
    record.thumbnail_path = path
    await session.commit()
    await session.refresh(record)
    if old:
        delete_file(old)
    count = int((await session.execute(
        select(func.count(Download.id)).where(Download.file_id == record.id)
    )).scalar_one())
    return file_response(record, count)


router = APIRouter(prefix="/api/projects/{project_id}/files", tags=["files"])


@router.get("", response_model=list[FileResponse])
async def list_files(
    project_id: int,
    session: AsyncSession = Depends(get_session),
    user: Optional[str] = Depends(get_optional_user),
) -> list[FileResponse]:
    await hide_drafts(session, project_id, user)
    items = (
        await session.scalars(
            select(ProjectFile)
            .where(ProjectFile.project_id == project_id)
            .order_by(ProjectFile.uploaded_at.desc())
        )
    ).all()
    # Per-file download counts — bulk fetch in one query to avoid n+1.
    counts: dict[int, int] = {}
    if items:
        rows = (
            await session.execute(
                select(Download.file_id, func.count(Download.id))
                .where(Download.file_id.in_([i.id for i in items]))
                .group_by(Download.file_id)
            )
        ).all()
        counts = {fid: int(c) for fid, c in rows}
    return [file_response(i, counts.get(i.id, 0)) for i in items]


@router.post("", response_model=FileResponse | ImageResponse, status_code=201)
async def upload_file(
    project_id: int,
    file: UploadFile,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> FileResponse | ImageResponse:
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

    # An image uploaded through the Files & Downloads section belongs in the
    # Images gallery, not the downloads list. Detect it by extension and
    # persist it as a ProjectImage (stored under projects/images/…) so it
    # surfaces in the right place. The dedup-free single-row insert mirrors
    # images.upload_image; the response is an ImageResponse so callers can
    # tell where the upload landed. The one-off backfill in
    # db.migrate_image_files_to_images relocates rows uploaded before this
    # routing existed.
    if is_image(file.filename):
        path = save_file(content, filename, project_id, "images")
        if path is None:
            raise HTTPException(status.HTTP_500_INTERNAL_SERVER_ERROR, detail="Failed to save file")
        # Append to the end of the gallery so an image dropped on the files
        # uploader doesn't displace the project's current cover (first image).
        max_sort = await session.scalar(
            select(func.max(ProjectImage.sort_order))
            .where(ProjectImage.project_id == project_id)
        )
        image = ProjectImage(
            project_id=project_id,
            filename=file.filename,
            file_path=path,
            sort_order=(max_sort + 1) if max_sort is not None else 0,
        )
        session.add(image)
        await session.commit()
        await session.refresh(image)
        return ImageResponse.model_validate(image, from_attributes=True)

    path = save_file(content, filename, project_id, "files")
    if path is None:
        raise HTTPException(status.HTTP_500_INTERNAL_SERVER_ERROR, detail="Failed to save file")

    record = ProjectFile(
        project_id=project_id,
        filename=file.filename,
        file_path=path,
        file_size=len(content),
        file_type=file_category(file.filename),
    )
    session.add(record)
    await session.commit()
    await session.refresh(record)
    return file_response(record)


@router.get("/{file_id}/download")
async def download_file(
    project_id: int,
    file_id: int,
    request: Request,
    preview: bool = False,
    user: Optional[str] = Depends(get_optional_user),
    session: AsyncSession = Depends(get_session),
) -> Response:
    """The file itself. ``?preview=1`` is the 3D viewer reading a model to
    show it, which isn't counted as a download."""
    record = await session.get(ProjectFile, file_id)
    if record is None or record.project_id != project_id:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    # Issue #260: a draft's files are the author's alone.
    draft = await hide_drafts(session, project_id, user)

    data = read_file(record.file_path)
    if data is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="File not found in storage")

    # Log the download. Never fail the file response if logging breaks —
    # the user came here to download, not to feed analytics.
    try:
        if not preview:
            settings = get_settings()
            ip_hash = None
            if not user:
                ip_hash = _hash_ip(_client_ip(request), settings.ip_hash_salt)
            await _log_download(session, project_id, file_id, user, ip_hash)
    except Exception:  # pragma: no cover — defensive guard
        logger.warning("Failed to log download for file %s", file_id, exc_info=True)

    headers = {"Content-Disposition": f'attachment; filename="{record.filename}"'}
    if draft:
        headers["Cache-Control"] = "private, no-store"
    return Response(content=data, media_type="application/octet-stream", headers=headers)


@router.put("/{file_id}", response_model=FileResponse)
async def update_file(
    project_id: int,
    file_id: int,
    body: FileUpdate,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> FileResponse:
    """Partial-update an uploaded file's metadata (issue #187).

    Owner-only. ``description`` and ``print_settings`` are mutable — the file itself is
    immutable post-upload (filename/size/type are stamped at write time).
    Partial-update semantics match ``LinkUpdate``: fields omitted from the
    body (``exclude_unset``) are left untouched; an explicit empty string
    clears the description; ``None`` is treated as "don't touch" (because
    Pydantic only sets fields the client actually included in the JSON).
    """
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    if project.author_username != user:
        raise HTTPException(status.HTTP_403_FORBIDDEN)

    record = await session.get(ProjectFile, file_id)
    if record is None or record.project_id != project_id:
        raise HTTPException(status.HTTP_404_NOT_FOUND)

    # ``exclude_unset`` so a missing field stays missing rather than being
    # patched to None — partial-update semantics. An explicit "" still
    # comes through and clears the description.
    payload = body.model_dump(exclude_unset=True)
    for field, value in payload.items():
        setattr(record, field, value)
    await session.commit()
    await session.refresh(record)

    download_count = int(
        (
            await session.execute(
                select(func.count(Download.id)).where(Download.file_id == record.id)
            )
        ).scalar_one()
    )
    return file_response(record, download_count)


@router.post("/{file_id}/thumbnail", response_model=FileResponse)
async def upload_thumbnail(
    project_id: int,
    file_id: int,
    file: UploadFile,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> FileResponse:
    """Store a model's thumbnail (issue #262). The editor renders it with
    Three.js after the upload, so the server never renders anything."""
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    if project.author_username != user:
        raise HTTPException(status.HTTP_403_FORBIDDEN)
    record = await session.get(ProjectFile, file_id)
    if record is None or record.project_id != project_id:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    return await store_thumbnail(session, record, file)


@router.get("/{file_id}/thumbnail")
async def view_thumbnail(
    project_id: int,
    file_id: int,
    user: Optional[str] = Depends(get_optional_user),
    session: AsyncSession = Depends(get_session),
) -> Response:
    record = await session.get(ProjectFile, file_id)
    if record is None or record.project_id != project_id or not record.thumbnail_path:
        raise HTTPException(status.HTTP_404_NOT_FOUND)
    draft = await hide_drafts(session, project_id, user)
    data = read_file(record.thumbnail_path)
    if data is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Thumbnail not found in storage")
    # Pages ask for ?v=<thumbnail_version>, so a replaced thumbnail is a new
    # URL and a long max-age is safe.
    etag = '"' + thumbnail_version(record.thumbnail_path) + '"'
    cache = "private, no-store" if draft else "public, max-age=86400"
    return Response(
        content=data, media_type="image/png",
        headers={"Cache-Control": cache, "ETag": etag},
    )


@router.delete("/{file_id}", status_code=204)
async def delete_file_endpoint(
    project_id: int,
    file_id: int,
    user: str = Depends(require_terms_accepted),
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
    if record.thumbnail_path:
        delete_file(record.thumbnail_path)
    await session.delete(record)
    await session.commit()
