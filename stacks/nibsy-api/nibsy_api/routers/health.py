"""Health-check router."""

from __future__ import annotations

from fastapi import APIRouter, Depends
from sqlalchemy import func, select
from sqlalchemy.ext.asyncio import AsyncSession

from ..db import get_session
from ..models import NibsyContent, NibsyRecommendation
from ..schemas import HealthResponse

router = APIRouter(tags=["health"])


@router.get("/health", response_model=HealthResponse)
async def health(session: AsyncSession = Depends(get_session)) -> HealthResponse:
    """Liveness/readiness probe.

    Returns the row counts of the two tables we actually care about so the
    Docker healthcheck and any external monitor can tell the service has a
    populated database, not just that it can serve a 200.
    """

    content_count = await session.scalar(select(func.count()).select_from(NibsyContent))
    recommendation_count = await session.scalar(
        select(func.count()).select_from(NibsyRecommendation)
    )
    return HealthResponse(
        status="ok",
        content_count=int(content_count or 0),
        recommendation_count=int(recommendation_count or 0),
    )
