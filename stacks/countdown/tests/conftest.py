from __future__ import annotations

import sys
from pathlib import Path
from typing import AsyncIterator

import pytest_asyncio
from httpx import ASGITransport, AsyncClient

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))


@pytest_asyncio.fixture
async def client() -> AsyncIterator[AsyncClient]:
    from countdown_service.main import create_app

    app = create_app()
    transport = ASGITransport(app=app)
    async with AsyncClient(transport=transport, base_url="http://test") as value:
        yield value
