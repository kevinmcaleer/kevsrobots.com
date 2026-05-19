"""Admin-only parts endpoints (issue #122 Phase 2).

Currently a single endpoint: force a synchronous supplier health check
for one part. Useful when the daily APScheduler job hasn't fired yet
and an admin wants to verify a fix landed without waiting 24 hours.
"""

from __future__ import annotations

import logging

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import require_admin
from ..db import get_session
from ..models import Part, PartSupplier
from ..schemas import PartSupplierResponse
from ..supplier_health import recheck_part_suppliers

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/admin/parts", tags=["admin-parts"])


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
