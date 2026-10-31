"""Admin-only parts endpoints (issue #122 Phase 2).

Also the review queue for parts people submit from the project editor
(``GET /submitted``, ``POST /{slug}/approve``, ``POST /{slug}/reject``).

Currently a single endpoint: force a synchronous supplier health check
for one part. Useful when the daily APScheduler job hasn't fired yet
and an admin wants to verify a fix landed without waiting 24 hours.
"""

from __future__ import annotations

import asyncio
import logging
import re
import tempfile
from pathlib import Path

from datetime import datetime, timezone
from typing import Literal

from fastapi import APIRouter, Depends, HTTPException, Query, status
from sqlalchemy import func, select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import require_admin
from ..db import get_session
from ..models import Part, PartPhoto, PartSupplier
from ..schemas import PartDetail, PartPhotoReview, PartReviewItem, PartSupplierResponse
from ..import_snakie_parts import download_github_library, import_library
from ..supplier_health import recheck_part_suppliers

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/admin/parts", tags=["admin-parts"])


_REF = re.compile(r"^[A-Za-z0-9._/-]{1,100}$")
_sync_lock = asyncio.Lock()


@router.post("/sync-snakie")
async def sync_snakie_library(
    ref: str = "master",
    _admin: str = Depends(require_admin),
    session: AsyncSession = Depends(get_session),
) -> dict:
    """Admin: import Snakie's Standard parts library from GitHub.

    The same import as ``python -m projects_api.import_snakie_parts --github``
    (Snakie is the master copy of the catalog). Parts are matched by slug,
    then name, then part number, so running it again only adds new parts
    and refreshes changed ones.
    """
    if not _REF.match(ref) or ".." in ref:
        raise HTTPException(status.HTTP_400_BAD_REQUEST, detail="Bad branch or tag name")
    if _sync_lock.locked():
        raise HTTPException(status.HTTP_409_CONFLICT, detail="A sync is already running")
    async with _sync_lock:
        with tempfile.TemporaryDirectory() as tmp:
            try:
                library = await asyncio.to_thread(download_github_library, ref, Path(tmp) / "library")
            except Exception as exc:  # network, 404, bad tarball
                logger.warning("Snakie library download failed (%s): %s", ref, exc)
                raise HTTPException(
                    status.HTTP_502_BAD_GATEWAY,
                    detail=f"Could not download the Snakie library ({ref}): {exc}",
                ) from exc
            report = await import_library(library, session)
    return {
        "ref": ref,
        "summary": report.summary(),
        "created": report.created,
        "updated": report.updated,
        "unchanged": report.unchanged,
        "skipped": report.skipped,
        "photo_failed": report.photo_failed,
    }


@router.post(
    "/{slug}/recheck-suppliers",
    response_model=list[PartSupplierResponse],
)
async def force_recheck_suppliers(
    slug: str,
    _admin: str = Depends(require_admin),
    session: AsyncSession = Depends(get_session),
) -> list[PartSupplierResponse]:
    """Admin: re-run the supplier health check synchronously for ``slug``.

    Bypasses the 24h scheduler — handy for ad-hoc verification after
    fixing a supplier URL.
    """
    part = await session.scalar(select(Part).where(Part.slug == slug))
    if part is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Part not found")

    await recheck_part_suppliers(session, part.id)

    suppliers = (
        await session.scalars(
            select(PartSupplier)
            .where(PartSupplier.part_id == part.id)
            .order_by(PartSupplier.id)
        )
    ).all()
    return [
        PartSupplierResponse(
            id=s.id,
            name=s.supplier_name,
            url=s.url,
            last_checked_at=s.last_checked_at,
            last_status=s.last_status,
            last_status_code=s.last_status_code,
            is_broken=bool(s.is_broken),
            consecutive_failures=int(s.consecutive_failures or 0),
        )
        for s in suppliers
    ]


@router.get("/photos/needs-review", response_model=list[PartPhotoReview])
async def photos_needing_review(
    _admin: str = Depends(require_admin),
    session: AsyncSession = Depends(get_session),
) -> list[PartPhotoReview]:
    """Issue #266: photos credited to a source with no licence on record,
    such as images imported from the Snakie library."""
    rows = (
        await session.execute(
            select(PartPhoto, Part.slug, Part.name)
            .join(Part, Part.id == PartPhoto.part_id)
            .where(PartPhoto.source.is_not(None), PartPhoto.licence.is_(None))
            .order_by(Part.name, PartPhoto.sort_order)
        )
    ).all()
    return [
        PartPhotoReview(
            id=photo.id, part_slug=slug, part_name=name, source=photo.source,
            url=photo.external_url or f"/api/parts/{slug}/photos/{photo.id}/view",
            is_external=bool(photo.external_url),
        )
        for photo, slug, name in rows
    ]


# --- submitted parts (the project editor's "New part" dialog) -----------


@router.get("/submitted", response_model=list[PartReviewItem])
async def submitted_parts(
    status_: Literal["pending", "rejected"] = Query("pending", alias="status"),
    _admin: str = Depends(require_admin),
    session: AsyncSession = Depends(get_session),
) -> list[PartReviewItem]:
    """Admin: parts waiting for review (or the ones turned down), oldest first."""
    from .parts import first_photo_cover_urls

    parts = (
        await session.scalars(
            select(Part).where(Part.status == status_).order_by(Part.created_at.asc(), Part.id.asc())
        )
    ).all()
    ids = [p.id for p in parts]
    covers = await first_photo_cover_urls(session, {p.id: p.slug for p in parts})
    photo_counts: dict[int, int] = {}
    links: dict[int, list[str]] = {}
    if ids:
        for pid, n in (
            await session.execute(
                select(PartPhoto.part_id, func.count(PartPhoto.id)).where(PartPhoto.part_id.in_(ids)).group_by(PartPhoto.part_id)
            )
        ).all():
            photo_counts[pid] = int(n)
        for sup in (
            await session.scalars(select(PartSupplier).where(PartSupplier.part_id.in_(ids)).order_by(PartSupplier.id))
        ).all():
            links.setdefault(sup.part_id, []).append(sup.url)
    return [
        PartReviewItem(
            id=p.id, slug=p.slug, name=p.name, status=p.status, created_by=p.created_by,
            created_at=p.created_at, description_md=p.description_md, category=p.category,
            cover_url=covers.get(p.id) or p.image_url, photo_count=photo_counts.get(p.id, 0),
            links=links.get(p.id, []) + ([p.datasheet_url] if p.datasheet_url else []),
            usage_count=p.usage_count or 0,
        )
        for p in parts
    ]


async def _set_review_status(session: AsyncSession, slug: str, new_status: str) -> PartDetail:
    from .parts import _part_detail

    part = await session.scalar(select(Part).where(Part.slug == slug))
    if part is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Part not found")
    if part.status not in ("pending", "rejected"):
        raise HTTPException(status.HTTP_409_CONFLICT, detail=f"The part is already in the catalog ({part.status})")
    part.status = new_status
    part.updated_at = datetime.now(timezone.utc).replace(tzinfo=None)
    await session.commit()
    await session.refresh(part)
    return await _part_detail(session, part)


@router.post("/{slug}/approve", response_model=PartDetail)
async def approve_part(
    slug: str,
    _admin: str = Depends(require_admin),
    session: AsyncSession = Depends(get_session),
) -> PartDetail:
    """Admin: put a submitted part in the catalog. It becomes a ``draft``,
    so everyone can find it and the usual verification rules apply."""
    return await _set_review_status(session, slug, "draft")


@router.post("/{slug}/reject", response_model=PartDetail)
async def reject_part(
    slug: str,
    _admin: str = Depends(require_admin),
    session: AsyncSession = Depends(get_session),
) -> PartDetail:
    """Admin: turn a submitted part down. It stays out of the catalog, but
    isn't deleted, so the projects that link it keep working (and it can
    still be approved later)."""
    return await _set_review_status(session, slug, "rejected")
