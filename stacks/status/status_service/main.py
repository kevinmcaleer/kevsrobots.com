"""FastAPI app for the status service.

Wiring:
  * lifespan: init schema, schedule 15-min poll + daily vacuum
  * read-only public API under /api/*
  * dashboard at /
  * /health for our own liveness probe

The whole surface is public — no auth at all. Only safe data is
exposed: service display name, current state, current latency, recent
incident windows, uptime percentages. No URLs, no secrets, no usernames.
"""

from __future__ import annotations

import logging
import time
from contextlib import asynccontextmanager
from datetime import datetime, timezone
from pathlib import Path
from typing import AsyncIterator, Optional

from apscheduler.schedulers.asyncio import AsyncIOScheduler
from apscheduler.triggers.cron import CronTrigger
from apscheduler.triggers.interval import IntervalTrigger
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import PlainTextResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates

from .config import Settings, get_settings, scheduler_disabled
from .db import (
    cell_status_strings,
    init_schema,
    latest_per_service,
    vacuum_old_rows,
)
from .incidents import incidents_for_window, uptime_for_window
from .polling import overall_status, poll_once

logger = logging.getLogger(__name__)

_HERE = Path(__file__).resolve().parent
_TEMPLATES = Jinja2Templates(directory=str(_HERE / "templates"))
# Cache-bust the dashboard CSS link with the process start time so a
# container restart (i.e. a rebuild + redeploy) automatically invalidates
# the browser's cached stylesheet. Otherwise an old dashboard.css can
# linger in the browser even after the server is serving the new one.
_CACHE_BUST = str(int(time.time()))


async def _scheduled_poll() -> None:
    """APScheduler job: poll every service once."""

    settings = get_settings()
    try:
        results = await poll_once(settings, settings.services_list)
        logger.info("polled %d service(s)", len(results))
    except Exception:  # noqa: BLE001 — never let the scheduler die
        logger.exception("scheduled poll failed")


async def _scheduled_vacuum() -> None:
    """APScheduler job: delete rows older than retention and VACUUM."""

    settings = get_settings()
    try:
        deleted = await vacuum_old_rows(
            settings.status_db_path, retention_days=settings.retention_days
        )
        logger.info("vacuum deleted %d row(s)", deleted)
    except Exception:  # noqa: BLE001
        logger.exception("scheduled vacuum failed")


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncIterator[None]:
    """Boot sequence: ensure schema, then schedule jobs (unless disabled)."""

    settings = get_settings()
    # init_schema is idempotent — CREATE TABLE IF NOT EXISTS.
    # Wrap in try/except so a read-only or missing /data dir during
    # tests doesn't kill the app; tests override the path.
    try:
        await init_schema(settings.status_db_path)
    except Exception:  # noqa: BLE001
        logger.exception("init_schema failed for %s", settings.status_db_path)

    scheduler: Optional[AsyncIOScheduler] = None
    if not scheduler_disabled():
        scheduler = AsyncIOScheduler()
        scheduler.add_job(
            _scheduled_poll,
            trigger=IntervalTrigger(minutes=settings.poll_interval_minutes),
            id="status-poll",
            name="Poll all monitored services",
            max_instances=1,
            coalesce=True,
            replace_existing=True,
            # Fire the first poll IMMEDIATELY at startup so the dashboard
            # has real data within seconds instead of showing "unknown"
            # for the whole first 15-minute interval (every restart).
            next_run_time=datetime.now(timezone.utc),
        )
        scheduler.add_job(
            _scheduled_vacuum,
            trigger=CronTrigger(
                hour=settings.vacuum_hour_utc,
                minute=settings.vacuum_minute_utc,
            ),
            id="status-vacuum",
            name="Daily vacuum of old health-check rows",
            max_instances=1,
            coalesce=True,
            replace_existing=True,
        )
        scheduler.start()
        logger.info(
            "scheduled: poll every %dm, vacuum daily at %02d:%02d UTC",
            settings.poll_interval_minutes,
            settings.vacuum_hour_utc,
            settings.vacuum_minute_utc,
        )

    try:
        yield
    finally:
        if scheduler is not None:
            scheduler.shutdown(wait=False)
            logger.info("scheduler stopped")


def _serialise_service_entry(
    name: str, latest: Optional[dict]
) -> dict:
    """Project a DB row into the public-safe API shape."""

    if latest is None:
        return {
            "service": name,
            "status": "unknown",
            "last_checked_at": None,
            "latency_ms": None,
            "http_status": None,
            "message": "no data yet",
        }
    return {
        "service": name,
        "status": latest["status"],
        "last_checked_at": latest["checked_at"],
        "latency_ms": latest["latency_ms"],
        "http_status": latest["http_status"],
        "message": latest["error"] or "ok",
    }


def create_app() -> FastAPI:
    """Application factory."""

    app = FastAPI(
        title="kevsrobots status",
        description=(
            "Public status page + API for kevsrobots web services. "
            "See https://github.com/kevinmcaleer/kevsrobots.com/issues/203."
        ),
        version="0.1.0",
        lifespan=lifespan,
    )

    static_dir = _HERE / "static"
    if static_dir.exists():
        app.mount(
            "/static", StaticFiles(directory=str(static_dir)), name="static"
        )

    @app.get("/health", response_class=PlainTextResponse)
    async def _health() -> str:
        """Own liveness — plain ``ok``. Used by Docker HEALTHCHECK."""
        return "ok"

    @app.get("/api/status")
    async def api_status() -> dict:
        """Return overall + per-service current state."""

        settings = get_settings()
        names = settings.service_names
        latest = await latest_per_service(settings.status_db_path, names)
        per_service = {
            name: _serialise_service_entry(name, latest.get(name)) for name in names
        }
        # overall_status treats 'unknown' as non-green (amber), per spec
        # comment in polling.py.
        overall = overall_status(per_service)
        return {
            "overall": overall,
            "services": per_service,
        }

    @app.get("/api/status/{service}")
    async def api_status_one(service: str) -> dict:
        settings = get_settings()
        names = settings.service_names
        if service not in names:
            raise HTTPException(status_code=404, detail="unknown service")
        latest = await latest_per_service(settings.status_db_path, [service])
        return _serialise_service_entry(service, latest.get(service))

    @app.get("/api/incidents")
    async def api_incidents(
        days: int = 30, service: Optional[str] = None
    ) -> dict:
        settings = get_settings()
        if days < 1:
            days = 1
        if days > settings.retention_days:
            days = settings.retention_days
        if service and service not in settings.service_names:
            raise HTTPException(status_code=404, detail="unknown service")
        incidents = await incidents_for_window(
            settings.status_db_path, days=days, service=service
        )
        return {"days": days, "incidents": incidents}

    @app.get("/api/uptime")
    async def api_uptime(
        days: int = 30, service: Optional[str] = None
    ) -> dict:
        settings = get_settings()
        if days < 1:
            days = 1
        if days > settings.retention_days:
            days = settings.retention_days
        if service and service not in settings.service_names:
            raise HTTPException(status_code=404, detail="unknown service")
        per_service = await uptime_for_window(
            settings.status_db_path, days=days, service=service
        )
        # Make sure every configured service appears (with zeros) so the
        # dashboard doesn't omit silent services.
        for name in settings.service_names:
            if service and name != service:
                continue
            per_service.setdefault(
                name, {"total": 0, "green": 0, "uptime_pct": None}
            )
        return {"days": days, "services": per_service}

    @app.get("/api/timeline")
    async def api_timeline(days: int = 30) -> dict:
        """One compact status string per service for the dashboard timeline.

        Each char = one cell (default 15 min): ``g`` green, ``a`` amber,
        ``r`` red, ``_`` no data. Strings are oldest-on-the-left,
        ``cells-1`` is the current cell — so the rightmost cell updates
        every poll and everything shifts left."""
        settings = get_settings()
        if days < 1:
            days = 1
        if days > settings.retention_days:
            days = settings.retention_days
        cells_per_day = settings.timeline_cells_per_day
        cell_seconds = (24 * 60 * 60) // cells_per_day
        total_cells = days * cells_per_day
        services_data = await cell_status_strings(
            settings.status_db_path,
            services=settings.service_names,
            cell_seconds=cell_seconds,
            cells=total_cells,
        )
        return {
            "days": days,
            "cells": total_cells,
            "cell_seconds": cell_seconds,
            "services": services_data,
        }

    @app.get("/")
    async def dashboard(request: Request):
        """The public dashboard. All data fetched client-side."""

        settings = get_settings()
        # Starlette ≥0.29 changed TemplateResponse to take ``request`` as the
        # first positional arg; the old (name, context) shape silently
        # passes the context dict as ``name`` and crashes with
        # ``TypeError: unhashable type: 'dict'`` deep inside Jinja's cache.
        return _TEMPLATES.TemplateResponse(
            request,
            "index.html",
            {
                "services": settings.service_names,
                "timeline_cells_per_day": settings.timeline_cells_per_day,
                "retention_days": settings.retention_days,
                "cache_bust": _CACHE_BUST,
            },
        )

    return app


app = create_app()


def get_app_settings(_app: FastAPI) -> Settings:
    """Convenience for tests / external callers."""
    return get_settings()
