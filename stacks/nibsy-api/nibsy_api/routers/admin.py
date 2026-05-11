"""Administrative endpoints (manual ingest + regenerate triggers).

Both endpoints are unauthenticated for now. Auth lands with #69 (production
ingest pipeline) and #71 (widget integration) — the API will live on the
private Pi cluster network until then.
"""

from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession

from ..config import get_settings
from ..db import get_session
from ..generator import generate_recommendations
from ..ingest import ingest_from_data_dir, ingest_from_remote
from ..schemas import GenerationStats, IngestStats
from ..trending import compute_trending

router = APIRouter(prefix="/api/admin", tags=["admin"])


@router.post("/ingest", response_model=IngestStats)
async def trigger_ingest(
    session: AsyncSession = Depends(get_session),
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
) -> IngestStats:
    """Trigger a remote ingestion from the live site (#69)."""

    settings = get_settings()
    return await ingest_from_remote(settings.site_base_url, session)


@router.post("/recompute-trending")
async def trigger_trending(
    session: AsyncSession = Depends(get_session),
) -> dict:
    """Trigger a manual trending score recomputation (#72)."""

    settings = get_settings()
    return await compute_trending(session, settings.page_count_url)


@router.post("/regenerate-recommendations", response_model=GenerationStats)
async def trigger_regenerate(
    session: AsyncSession = Depends(get_session),
) -> GenerationStats:
    """Trigger a manual recommendations regeneration (#74).

    The scheduler also runs this every `RECOMMENDATION_REFRESH_DAYS` days
    in the background; this endpoint is for ad-hoc refreshes after a
    content change or an algorithm tweak.
    """

    return await generate_recommendations(session)
