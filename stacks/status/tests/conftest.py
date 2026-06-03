"""Shared pytest fixtures.

Each test gets its own temp-file SQLite DB so they cannot leak state
into each other. We disable the APScheduler scheduler globally via
``STATUS_DISABLE_SCHEDULER=1`` so the test suite is hermetic.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import AsyncIterator

import pytest
import pytest_asyncio
from httpx import ASGITransport, AsyncClient

# Make ``status_service`` importable when pytest is invoked from this dir.
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))


@pytest.fixture(autouse=True)
def _env(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    """Point every test at an isolated temp DB + a tiny service list.

    Also kills the scheduler so APScheduler never starts inside tests.
    """

    monkeypatch.setenv("STATUS_DB_PATH", str(tmp_path / "status.db"))
    monkeypatch.setenv(
        "STATUS_SERVICES",
        "search=http://example.invalid/search,"
        "chatter=http://example.invalid/chatter,"
        "nibsy=http://example.invalid/nibsy",
    )
    monkeypatch.setenv("STATUS_DISABLE_SCHEDULER", "1")
    monkeypatch.setenv("POLL_INTERVAL_MINUTES", "15")
    monkeypatch.setenv("RETENTION_DAYS", "30")


@pytest_asyncio.fixture
async def db_path(_env, tmp_path: Path) -> AsyncIterator[str]:
    """A pre-initialised SQLite path for tests that want to write rows."""

    from status_service.db import init_schema

    path = os.environ["STATUS_DB_PATH"]
    await init_schema(path)
    yield path


@pytest_asyncio.fixture
async def client(_env) -> AsyncIterator[AsyncClient]:
    """ASGI test client. The lifespan runs init_schema for us."""

    from status_service.main import create_app

    app = create_app()
    transport = ASGITransport(app=app)
    async with AsyncClient(transport=transport, base_url="http://test") as ac:
        async with app.router.lifespan_context(app):
            yield ac
