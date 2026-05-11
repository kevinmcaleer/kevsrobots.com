"""FastAPI application entrypoint.

Builds the app, wires routers, and runs an auto-ingest on startup if the
content table is empty and `NIBSY_DATA_DIR` is configured.
"""

from __future__ import annotations

import logging
from contextlib import asynccontextmanager
from typing import AsyncIterator

from fastapi import FastAPI
from sqlalchemy import func, select

from .config import get_settings
from .db import create_all, get_sessionmaker
from .ingest import ingest_from_data_dir
from .models import NibsyContent
from .routers import admin, health, stubs

logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncIterator[None]:
    """Create tables and optionally run a first-time ingest."""

    await create_all()

    settings = get_settings()
    data_dir = settings.nibsy_data_dir
    if data_dir is not None and data_dir.exists() and data_dir.is_dir():
        sessionmaker = get_sessionmaker()
        async with sessionmaker() as session:
            count = await session.scalar(select(func.count()).select_from(NibsyContent))
            if not count:
                logger.info("nibsy_content is empty — running startup ingest from %s", data_dir)
                stats = await ingest_from_data_dir(data_dir, session)
                logger.info("startup ingest finished: %s", stats.model_dump())
            else:
                logger.info("nibsy_content already has %s rows — skipping startup ingest", count)
    else:
        logger.info("NIBSY_DATA_DIR not configured or missing — skipping startup ingest")

    yield


def create_app() -> FastAPI:
    """Application factory."""

    app = FastAPI(
        title="Nibsy API",
        description="Recommendation microservice for kevsrobots.com",
        version="0.1.0",
        lifespan=lifespan,
    )
    app.include_router(health.router)
    app.include_router(admin.router)
    app.include_router(stubs.router)
    return app


app = create_app()
