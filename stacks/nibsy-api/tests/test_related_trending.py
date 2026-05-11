"""Related and trending endpoint tests (#67, #72)."""

from __future__ import annotations

import pytest


@pytest.mark.asyncio
async def test_related_returns_results(client) -> None:
    response = await client.get("/api/related/1?limit=3")
    if response.status_code == 404:
        pytest.skip("content_id=1 not present in fixtures")
    assert response.status_code == 200
    body = response.json()
    assert "related" in body
    assert isinstance(body["related"], list)
    for item in body["related"]:
        assert item["id"] != 1


@pytest.mark.asyncio
async def test_related_not_found(client) -> None:
    response = await client.get("/api/related/999999")
    assert response.status_code == 404


@pytest.mark.asyncio
async def test_trending_empty(client) -> None:
    response = await client.get("/api/trending?limit=5")
    assert response.status_code == 200
    assert response.json()["trending"] == []


@pytest.mark.asyncio
async def test_trending_with_clicks(client) -> None:
    for _ in range(5):
        await client.post(
            "/api/track/click",
            json={"content_url": "/learn/example/00_intro.html", "source_page": "/"},
        )
    response = await client.get("/api/trending?limit=5")
    assert response.status_code == 200
    trending = response.json()["trending"]
    # Falls back to click-only data since no precomputed scores exist.
    if trending:
        assert trending[0]["score"] >= 5


@pytest.mark.asyncio
async def test_trending_content_type_filter(client) -> None:
    response = await client.get("/api/trending?limit=5&content_type=course")
    assert response.status_code == 200
    assert isinstance(response.json()["trending"], list)


@pytest.mark.asyncio
async def test_recompute_trending(client) -> None:
    response = await client.post("/api/admin/recompute-trending")
    assert response.status_code == 200
    body = response.json()
    assert "computed" in body
    assert "duration_ms" in body
