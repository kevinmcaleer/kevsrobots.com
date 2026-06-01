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
from ..schemas import FileResponse, FileUpdate, ImageResponse
from ..storage import (
    delete_file,
    generate_filename,
    is_image,
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
    out: list[FileResponse] = []
    for i in items:
        out.append(
            FileResponse(
                id=i.id,
                filename=i.filename,
                file_size=i.file_size,
                file_type=i.file_type,
                description=i.description,
                uploaded_at=i.uploaded_at,
                download_count=counts.get(i.id, 0),
            )
        )
    return out


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
        image = ProjectImage(
            project_id=project_id,
            filename=file.filename,
            file_path=path,
        )
        session.add(image)
        await session.commit()
        await session.refresh(image)
        return ImageResponse.model_validate(image, from_attributes=True)

    path = save_file(content, filename, project_id, "files")
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
    request: Request,
    user: Optional[str] = Depends(get_optional_user),
    session: AsyncSession = Depends(get_session),
) -> Response:
    record = await session.get(ProjectFile, file_id)
    if record is None or record.project_id != project_id:
        raise HTTPException(status.HTTP_404_NOT_FOUND)

    data = read_file(record.file_path)
    if data is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="File not found in storage")

    # Log the download. Never fail the file response if logging breaks —
    # the user came here to download, not to feed analytics.
    try:
        settings = get_settings()
        ip_hash = None
        if not user:
            ip_hash = _hash_ip(_client_ip(request), settings.ip_hash_salt)
        await _log_download(session, project_id, file_id, user, ip_hash)
    except Exception:  # pragma: no cover — defensive guard
        logger.warning("Failed to log download for file %s", file_id, exc_info=True)

    return Response(
        content=data,
        media_type="application/octet-stream",
        headers={"Content-Disposition": f'attachment; filename="{record.filename}"'},
    )


@router.put("/{file_id}", response_model=FileResponse)
async def update_file(
    project_id: int,
    file_id: int,
    body: FileUpdate,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> FileResponse:
    """Partial-update an uploaded file's metadata (issue #187).

    Owner-only. Today only ``description`` is mutable — the file itself is
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
    return FileResponse(
        id=record.id,
        filename=record.filename,
        file_size=record.file_size,
        file_type=record.file_type,
        description=record.description,
        uploaded_at=record.uploaded_at,
        download_count=download_count,
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
    await session.delete(record)
    await session.commit()
