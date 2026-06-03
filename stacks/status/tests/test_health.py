"""Liveness endpoint smoke test."""

from __future__ import annotations

import pytest


@pytest.mark.asyncio
async def test_health_ok(client) -> None:
    """``GET /health`` returns 200 with plain-text ``ok``."""

    response = await client.get("/health")
    assert response.status_code == 200
    assert response.text == "ok"


@pytest.mark.asyncio
async def test_dashboard_renders(client) -> None:
    """``GET /`` renders the dashboard template. Regression: Starlette ≥0.29
    requires ``TemplateResponse(request, name, context)`` — the old shape
    silently treats the context dict as ``name`` and 500s with
    ``TypeError: unhashable type: 'dict'`` deep in Jinja."""

    response = await client.get("/")
    assert response.status_code == 200, response.text
    body = response.text
    assert "<html" in body.lower()
    # The dashboard JS hits the API; the markup at minimum mentions one
    # of the configured services so we know the context reached Jinja.
    assert "status" in body.lower()
