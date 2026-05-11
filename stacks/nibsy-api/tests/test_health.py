"""Health endpoint smoke tests."""

from __future__ import annotations

import pytest


@pytest.mark.asyncio
async def test_health_ok(client) -> None:
    """`GET /health` returns 200 with the expected JSON shape."""

    response = await client.get("/health")
    assert response.status_code == 200
    body = response.json()
    assert body["status"] == "ok"
    assert "content_count" in body
    assert "recommendation_count" in body
    assert isinstance(body["content_count"], int)
    assert isinstance(body["recommendation_count"], int)
