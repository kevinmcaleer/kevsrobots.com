"""Shared pytest fixtures."""

from __future__ import annotations

import os
import sys
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

ROOT = __import__("pathlib").Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))


@pytest.fixture(autouse=True)
def _env(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("DATABASE_URL", "sqlite+aiosqlite:///:memory:")
    monkeypatch.setenv("JWT_SECRET", "test-secret")


@pytest_asyncio.fixture
async def engine():
    eng = create_async_engine(
        "sqlite+aiosqlite:///:memory:",
        future=True,
        poolclass=StaticPool,
        connect_args={"check_same_thread": False},
    )
    from projects_api import db as db_module
    from projects_api import models  # noqa: F401

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
    from projects_api import auth as auth_module
    from projects_api import db as db_module
    from projects_api import main as main_module
    from projects_api.main import create_app

    async def _override_session() -> AsyncIterator[AsyncSession]:
        async with sessionmaker_() as s:
            yield s

    async def _noop_create_all() -> None:
        return None

    original_create_all = db_module.create_all
    db_module.create_all = _noop_create_all

    # Issue #106: route the lifespan seeder at the test sessionmaker so
    # the badge catalog lands in the in-memory DB the test fixture set up
    # (instead of the global engine the seeder would otherwise grab).
    original_get_sessionmaker = db_module.get_sessionmaker
    db_module.get_sessionmaker = lambda: sessionmaker_
    main_module.get_sessionmaker = lambda: sessionmaker_

    app = create_app()
    app.dependency_overrides[db_module.get_session] = _override_session

    # Terms-gate (terms-gate): bypass the T&Cs acceptance check in the
    # default test client so the ~290 existing tests don't need to first
    # POST to /api/users/me/accept-terms. ``test_terms.py`` opts back IN
    # to the real gate by deleting these overrides on its own client.
    app.dependency_overrides[auth_module.require_terms_accepted] = (
        auth_module.get_current_user
    )
    app.dependency_overrides[auth_module.require_terms_accepted_aged] = (
        auth_module.get_current_user_aged
    )

    # feedback_reporter (#206) bundles the T&Cs gate with an anonymous
    # X-Snakie-Key path. Bypass its terms gate for the default client (like the
    # require_terms_accepted override above) while still exercising the REAL app-key
    # check, so authed-feedback tests pass and the anonymous-key tests are honest.
    from fastapi import Cookie, Depends, Header
    from projects_api.routers import feedback as feedback_module

    async def _feedback_reporter_no_terms(
        x_snakie_key: str | None = Header(default=None, alias="X-Snakie-Key"),
        access_token: str | None = Cookie(default=None),
        authorization: str | None = Header(default=None),
        session: AsyncSession = Depends(db_module.get_session),
    ) -> str:
        if feedback_module._is_snakie_app(x_snakie_key):
            return feedback_module.ANON_REPORTER
        return await auth_module.get_current_user(access_token, authorization, session)

    app.dependency_overrides[feedback_module.feedback_reporter] = (
        _feedback_reporter_no_terms
    )

    transport = ASGITransport(app=app)
    async with AsyncClient(transport=transport, base_url="http://test") as ac:
        async with app.router.lifespan_context(app):
            yield ac

    db_module.create_all = original_create_all
    db_module.get_sessionmaker = original_get_sessionmaker
    main_module.get_sessionmaker = original_get_sessionmaker


def make_auth_header(username: str = "testuser") -> dict:
    """Generate a valid JWT auth header for testing."""
    from jose import jwt

    token = jwt.encode({"sub": username}, "test-secret", algorithm="HS256")
    return {"Authorization": f"Bearer {token}"}
