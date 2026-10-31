"""Admin-only parts endpoints (issue #122 Phase 2).

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

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import require_admin
from ..db import get_session
from ..models import Part, PartPhoto, PartSupplier
from ..schemas import PartPhotoReview, PartSupplierResponse
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
