"""Async SQLAlchemy engine, session factory, and declarative Base.

Peer services in this repo use sync SQLAlchemy (see `pagecount/`), but the
Nibsy API is greenfield so we adopt the async stack from the outset to keep
the FastAPI handlers non-blocking.
"""

from __future__ import annotations

from typing import AsyncIterator

from sqlalchemy.ext.asyncio import (
    AsyncEngine,
    AsyncSession,
    async_sessionmaker,
    create_async_engine,
)
from sqlalchemy.orm import DeclarativeBase

from .config import get_settings


class Base(DeclarativeBase):
    """Declarative base for all ORM models."""


_engine: AsyncEngine | None = None
_sessionmaker: async_sessionmaker[AsyncSession] | None = None


def get_engine() -> AsyncEngine:
    """Lazily build and cache the global async engine."""

    global _engine
    if _engine is None:
        settings = get_settings()
        _engine = create_async_engine(settings.database_url, future=True)
    return _engine


def get_sessionmaker() -> async_sessionmaker[AsyncSession]:
    """Lazily build and cache the global async sessionmaker."""

    global _sessionmaker
    if _sessionmaker is None:
        _sessionmaker = async_sessionmaker(
            bind=get_engine(),
            expire_on_commit=False,
            class_=AsyncSession,
        )
    return _sessionmaker


def reset_engine() -> None:
    """Drop the cached engine/sessionmaker.

    Used by tests to swap database URLs between fixtures.
    """

    global _engine, _sessionmaker
    _engine = None
    _sessionmaker = None


async def get_session() -> AsyncIterator[AsyncSession]:
    """FastAPI dependency yielding an `AsyncSession` per request."""

    sessionmaker = get_sessionmaker()
    async with sessionmaker() as session:
        yield session


async def create_all() -> None:
    """Create every registered table. Mirrors peer services that skip Alembic."""

    # Importing models registers them on Base.metadata. The import is local
    # to avoid a circular dependency at module load time.
    from . import models  # noqa: F401

    engine = get_engine()
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)
