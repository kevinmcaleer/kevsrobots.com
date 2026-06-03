"""Liveness endpoint smoke test."""

from __future__ import annotations

import pytest


@pytest.mark.asyncio
async def test_health_ok(client) -> None:
    """``GET /health`` returns 200 with plain-text ``ok``."""

    response = await client.get("/health")
    assert response.status_code == 200
    assert response.text == "ok"
