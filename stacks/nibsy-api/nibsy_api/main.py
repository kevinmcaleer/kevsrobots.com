"""FastAPI application entrypoint.

Builds the app, wires routers, runs an auto-ingest on startup (if the
content table is empty and `NIBSY_DATA_DIR` is configured), then keeps the
catalogue fresh from the live site: once shortly after boot and once a day.
Recommendations themselves are computed per request, so there is nothing
to regenerate.
"""

from __future__ import annotations

import asyncio
import logging
from contextlib import asynccontextmanager
from typing import AsyncIterator

from apscheduler.schedulers.asyncio import AsyncIOScheduler
from apscheduler.triggers.cron import CronTrigger
from apscheduler.triggers.interval import IntervalTrigger
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from sqlalchemy import func, select

from .config import get_settings
from .db import create_all, get_sessionmaker
from .ingest import ingest_from_data_dir, ingest_from_remote
from .trending import compute_trending
from .models import NibsyContent
from .routers import admin, analytics, health, recommendations, stubs, tracking

logger = logging.getLogger(__name__)


async def _maybe_startup_ingest() -> None:
    """Run an ingest on boot if content is empty and a data dir is set."""

    settings = get_settings()
    data_dir = settings.nibsy_data_dir
    if data_dir is None or not data_dir.exists() or not data_dir.is_dir():
        logger.info("NIBSY_DATA_DIR not configured or missing — skipping startup ingest")
        return

    sessionmaker = get_sessionmaker()
    async with sessionmaker() as session:
        count = await session.scalar(
            select(func.count()).select_from(NibsyContent)
        )
        if count:
            logger.info(
                "nibsy_content already has %s rows — skipping startup ingest",
                count,
            )
            return
        logger.info("nibsy_content is empty — running startup ingest from %s", data_dir)
        stats = await ingest_from_data_dir(data_dir, session)
        logger.info("startup ingest finished: %s", stats.model_dump())


async def _scheduled_remote_ingest() -> None:
    """Refresh the catalogue from the live site (#69), then trending.

    A failed or empty refresh is logged at ERROR: a silently failing
    refresh is how the catalogue once sat frozen for six months.
    """

    settings = get_settings()
    sessionmaker = get_sessionmaker()
    async with sessionmaker() as session:
        try:
            stats = await ingest_from_remote(settings.site_base_url, session)
        except Exception:  # noqa: BLE001
            logger.exception("remote ingest failed")
            return
        if stats.errors:
            logger.error("remote ingest finished with errors: %s", stats.model_dump())
        else:
            logger.info("remote ingest finished: %s", stats.model_dump())
    await _scheduled_trending()


async def _scheduled_trending() -> None:
    """Hourly trending score recomputation (#72)."""

    settings = get_settings()
    sessionmaker = get_sessionmaker()
    async with sessionmaker() as session:
        try:
            stats = await compute_trending(session, settings.page_count_url)
            logger.info("scheduled trending finished: %s", stats)
        except Exception:  # noqa: BLE001
            logger.exception("scheduled trending failed")


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncIterator[None]:
    """Boot sequence: create tables, ingest, schedule."""

    await create_all()
    await _maybe_startup_ingest()

    settings = get_settings()
    scheduler: AsyncIOScheduler | None = None
    has_jobs = False
    startup_refresh: asyncio.Task | None = None

    if settings.ingest_schedule_hour >= 0:
        scheduler = AsyncIOScheduler()
        if settings.ingest_on_startup:
            # Don't make a deploy wait on the live site: refresh in the
            # background so the service is healthy immediately.
            startup_refresh = asyncio.create_task(_scheduled_remote_ingest())

    if scheduler and settings.ingest_schedule_hour >= 0:
        scheduler.add_job(
            _scheduled_remote_ingest,
            trigger=CronTrigger(hour=settings.ingest_schedule_hour, minute=0),
            id="nibsy-daily-ingest",
            name="Daily remote ingest from live site",
            max_instances=1,
            coalesce=True,
            replace_existing=True,
        )
        has_jobs = True
        logger.info(
            "scheduled: daily remote ingest at %02d:00 UTC from %s",
            settings.ingest_schedule_hour,
            settings.site_base_url,
        )

    if scheduler:
        scheduler.add_job(
            _scheduled_trending,
            trigger=IntervalTrigger(hours=1),
            id="nibsy-trending",
            name="Recompute trending scores",
            max_instances=1,
            coalesce=True,
            replace_existing=True,
        )
        has_jobs = True
        logger.info("scheduled: trending recompute every 1 hour")

    if scheduler and has_jobs:
        scheduler.start()
    elif not has_jobs:
        scheduler = None
        logger.info("no scheduled jobs configured")

    try:
        yield
    finally:
        if startup_refresh is not None and not startup_refresh.done():
            startup_refresh.cancel()
        if scheduler is not None:
            scheduler.shutdown(wait=False)
            logger.info("scheduler stopped")


def create_app() -> FastAPI:
    """Application factory."""

    app = FastAPI(
        title="Nibsy API",
        description="Recommendation microservice for kevsrobots.com",
        version="0.2.0",
        lifespan=lifespan,
    )
    settings = get_settings()
    # ``allow_credentials=True`` so the admin portal page (served from
    # www.kevsrobots.com) can include the Chatter cookie on its calls to
    # nibsy.kevsrobots.com (#158). The cookie is set on the parent domain
    # ``.kevsrobots.com`` and is required for ``require_admin``.
    app.add_middleware(
        CORSMiddleware,
        allow_origins=settings.cors_origins_list,
        allow_credentials=True,
        allow_methods=["GET", "POST"],
        allow_headers=["*"],
    )
    app.include_router(health.router)
    app.include_router(admin.router)
    app.include_router(recommendations.router)
    app.include_router(tracking.router)
    app.include_router(analytics.router)
    app.include_router(stubs.router)
    return app


app = create_app()
