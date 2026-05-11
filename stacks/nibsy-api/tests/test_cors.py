"""CORS middleware tests."""

from __future__ import annotations

import pytest


@pytest.mark.asyncio
async def test_preflight_allowed_origin(client) -> None:
    response = await client.options(
        "/health",
        headers={
            "Origin": "http://localhost:4000",
            "Access-Control-Request-Method": "GET",
        },
    )
    assert response.status_code == 200
    assert response.headers["access-control-allow-origin"] == "http://localhost:4000"


@pytest.mark.asyncio
async def test_preflight_disallowed_origin(client) -> None:
    response = await client.options(
        "/health",
        headers={
            "Origin": "http://evil.example.com",
            "Access-Control-Request-Method": "GET",
        },
    )
    assert "access-control-allow-origin" not in response.headers
