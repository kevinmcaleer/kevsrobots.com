"""Shared pytest fixtures.

Each test gets a fresh in-memory aiosqlite database so they cannot leak
state into each other. We monkeypatch the settings before constructing the
engine, then build the FastAPI app and override `get_session` to use the
test sessionmaker.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import AsyncIterator

import pytest
import pytest_asyncio
from httpx import ASGITransport, AsyncClient
from sqlalchemy.ext.asyncio import (
    AsyncSession,
    async_sessionmaker,
    create_async_engine,
)
from sqlalchemy.pool import StaticPool

# Make `nibsy_api` importable when pytest is invoked from this directory.
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

FIXTURES_DIR = ROOT / "tests" / "fixtures" / "_data"


@pytest.fixture(autouse=True)
def _env(monkeypatch: pytest.MonkeyPatch) -> None:
    """Point every test at the fixtures dir and an isolated DB URL."""

    monkeypatch.setenv("NIBSY_DATA_DIR", str(FIXTURES_DIR))
    # Unique in-memory DB per worker; the fixture below overrides the engine.
    monkeypatch.setenv("DATABASE_URL", "sqlite+aiosqlite:///:memory:")
    # Admin auth (#158): give existing tests a known admin so the
    # auth-gated /api/admin/* routes still resolve. The test-secret here
    # matches the helper below; ``test_admin_auth.py`` re-asserts the
    # gating logic explicitly.
    monkeypatch.setenv("JWT_SECRET", "test-secret")
    monkeypatch.setenv("ADMIN_USERNAMES", "kev")


def make_admin_header(username: str = "kev") -> dict[str, str]:
    """Sign a JWT for ``username`` so tests can hit /api/admin/* routes.

    Mirrors ``stacks/projects-api/tests/conftest.make_auth_header``. The
    signing key is fixed to ``test-secret`` (the value the ``_env``
    fixture writes into ``JWT_SECRET``) so signature verification works
    end-to-end inside the async client.
    """

    from jose import jwt

    token = jwt.encode({"sub": username}, "test-secret", algorithm="HS256")
    return {"Authorization": f"Bearer {token}"}


@pytest_asyncio.fixture
async def engine():
    """Fresh in-memory engine per test."""

    eng = create_async_engine(
        "sqlite+aiosqlite:///:memory:",
        future=True,
        poolclass=StaticPool,
        connect_args={"check_same_thread": False},
    )
    # Import after env is set so any module-level config picks it up.
    from nibsy_api import db as db_module
    from nibsy_api import models  # noqa: F401 — registers tables on Base.metadata

    async with eng.begin() as conn:
        await conn.run_sync(db_module.Base.metadata.create_all)

    yield eng

    await eng.dispose()


@pytest_asyncio.fixture
async def sessionmaker_(engine) -> async_sessionmaker[AsyncSession]:
    return async_sessionmaker(bind=engine, expire_on_commit=False, class_=AsyncSession)


@pytest_asyncio.fixture
async def session(sessionmaker_) -> AsyncIterator[AsyncSession]:
    async with sessionmaker_() as s:
        yield s


@pytest_asyncio.fixture
async def client(engine, sessionmaker_) -> AsyncIterator[AsyncClient]:
    """ASGI test client with `get_session` and `create_all` overridden."""

    from nibsy_api import db as db_module
    from nibsy_api.main import create_app

    async def _override_session() -> AsyncIterator[AsyncSession]:
        async with sessionmaker_() as s:
            yield s

    async def _noop_create_all() -> None:
        # Tables are already created by the `engine` fixture.
        return None

    # Patch create_all so lifespan doesn't try to talk to the global engine.
    original_create_all = db_module.create_all
    db_module.create_all = _noop_create_all
    # Patch the sessionmaker the lifespan uses so startup ingest writes to
    # the same in-memory DB our overridden dependency reads from.
    original_get_sessionmaker = db_module.get_sessionmaker
    db_module.get_sessionmaker = lambda: sessionmaker_

    app = create_app()
    app.dependency_overrides[db_module.get_session] = _override_session

    transport = ASGITransport(app=app)
    async with AsyncClient(transport=transport, base_url="http://test") as ac:
        async with app.router.lifespan_context(app):
            # Explicitly ingest fixtures so tests see real content data.
            # The lifespan's _maybe_startup_ingest may not populate data
            # reliably across SQLite in-memory connection boundaries.
            from nibsy_api.ingest import ingest_from_data_dir
            from nibsy_api.generator import generate_recommendations

            async with sessionmaker_() as s:
                from sqlalchemy import func, select
                from nibsy_api.models import NibsyContent

                count = await s.scalar(
                    select(func.count()).select_from(NibsyContent)
                )
                if not count:
                    await ingest_from_data_dir(FIXTURES_DIR, s)
                    await generate_recommendations(s)

            yield ac

    db_module.create_all = original_create_all
    db_module.get_sessionmaker = original_get_sessionmaker
