"""Administrative endpoints (manual ingest + regenerate triggers).

Every route in this router is gated behind :func:`require_admin` (#158).
Anonymous callers get 401; logged-in non-admins get 403. Mirrors the
gating pattern used by ``stacks/projects-api`` admin routes.

The APScheduler-driven jobs configured in ``main.lifespan`` are *not*
affected — they invoke the underlying functions directly without going
through the HTTP layer.
"""

from __future__ import annotations

from typing import Any, Optional

from fastapi import APIRouter, Body, Depends, HTTPException, Query, status
from sqlalchemy import func, select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import require_admin
from ..config import get_settings
from ..db import get_session
from ..generator import generate_recommendations
from ..categorise import export_for_categorisation, import_categorisation
from ..ingest import ingest_from_data_dir, ingest_from_remote
from ..models import NibsyContent, NibsyRecommendation
from ..schemas import AdminStatus, GenerationStats, IngestStats
from ..trending import compute_trending

router = APIRouter(prefix="/api/admin", tags=["admin"])


@router.get("/status", response_model=AdminStatus)
async def admin_status(
    session: AsyncSession = Depends(get_session),
    _user: str = Depends(require_admin),
) -> AdminStatus:
    """Cheap read-only summary for the admin portal (#158).

    Returns row counts for ``nibsy_content`` and ``nibsy_recommendations``
    plus the most recent ``generated_at`` timestamp across recommendations
    (best proxy for "when did the generator last run?"). Admin-gated to
    avoid leaking even count-shaped data to unauthenticated callers.
    """

    content_count = await session.scalar(
        select(func.count()).select_from(NibsyContent)
    )
    recommendation_count = await session.scalar(
        select(func.count()).select_from(NibsyRecommendation)
    )
    last_generated_at: Optional[Any] = await session.scalar(
        select(func.max(NibsyRecommendation.generated_at))
    )
    return AdminStatus(
        content_count=int(content_count or 0),
        recommendation_count=int(recommendation_count or 0),
        last_generated_at=last_generated_at,
    )


@router.post("/ingest", response_model=IngestStats)
async def trigger_ingest(
    session: AsyncSession = Depends(get_session),
    _user: str = Depends(require_admin),
) -> IngestStats:
    """Trigger a manual ingestion from `NIBSY_DATA_DIR`.

    The startup hook also runs ingestion automatically when the content
    table is empty; this endpoint is for re-runs after the YAML changes.
    Production deployment (#69) will eventually replace this with a
    scheduled job.
    """

    settings = get_settings()
    if settings.nibsy_data_dir is None:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="NIBSY_DATA_DIR is not configured",
        )
    return await ingest_from_data_dir(settings.nibsy_data_dir, session)


@router.post("/ingest-remote", response_model=IngestStats)
async def trigger_remote_ingest(
    session: AsyncSession = Depends(get_session),
    _user: str = Depends(require_admin),
) -> IngestStats:
    """Trigger a remote ingestion from the live site (#69)."""

    settings = get_settings()
    return await ingest_from_remote(settings.site_base_url, session)


@router.post("/recompute-trending")
async def trigger_trending(
    session: AsyncSession = Depends(get_session),
    _user: str = Depends(require_admin),
) -> dict:
    """Trigger a manual trending score recomputation (#72)."""

    settings = get_settings()
    return await compute_trending(session, settings.page_count_url)


@router.get("/categorise/export")
async def categorise_export(
    force: bool = Query(False, description="Re-export already categorised items"),
    session: AsyncSession = Depends(get_session),
    _user: str = Depends(require_admin),
) -> dict:
    """Export content items for manual AI categorisation (#75).

    Take the output to Claude (AI Max subscription), ask it to
    categorise each item, then POST the results back to /import.
    """

    items = await export_for_categorisation(session, force=force)
    return {
        "count": len(items),
        "prompt_hint": (
            "For each item, return JSON with: id, topics (list of specific "
            "topic tags like 'servo motors', 'PID control', 'MicroPython'), "
            "difficulty ('beginner'/'intermediate'/'advanced'), "
            "introduces (concepts taught), assumes (prerequisites). "
            "For courses in a sequence, add pathway: {name, order, total}."
        ),
        "items": items,
    }


@router.post("/categorise/import")
async def categorise_import(
    results: list[dict[str, Any]] = Body(...),
    session: AsyncSession = Depends(get_session),
    _user: str = Depends(require_admin),
) -> dict:
    """Import categorisation results from a manual AI pass (#75).

    Each item should have: id, topics, difficulty, introduces, assumes.
    Optionally: pathway ({name, order, total}) for course sequencing.
    """

    return await import_categorisation(session, results)


@router.post("/regenerate-recommendations", response_model=GenerationStats)
async def trigger_regenerate(
    session: AsyncSession = Depends(get_session),
    _user: str = Depends(require_admin),
) -> GenerationStats:
    """Trigger a manual recommendations regeneration (#74).

    The scheduler also runs this every `RECOMMENDATION_REFRESH_DAYS` days
    in the background; this endpoint is for ad-hoc refreshes after a
    content change or an algorithm tweak.
    """

    return await generate_recommendations(session)
