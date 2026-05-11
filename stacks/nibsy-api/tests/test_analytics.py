"""Analytics endpoint tests (#68)."""

from __future__ import annotations

import pytest


@pytest.mark.asyncio
async def test_nibsy_stats_empty(client) -> None:
    response = await client.get("/api/analytics/nibsy-stats")
    assert response.status_code == 200
    body = response.json()
    assert body["total_clicks_today"] == 0
    assert body["total_impressions_today"] == 0
    assert body["overall_ctr"] == 0


@pytest.mark.asyncio
async def test_top_clicked_empty(client) -> None:
    response = await client.get("/api/analytics/top-clicked")
    assert response.status_code == 200
    assert response.json()["top_clicked"] == []


@pytest.mark.asyncio
async def test_top_clicked_with_data(client) -> None:
    for _ in range(3):
        resp = await client.post(
            "/api/track/click",
            json={
                "content_url": "/learn/example/00_intro.html",
                "source_page": "/",
            },
        )
        assert resp.status_code == 200
    response = await client.get("/api/analytics/top-clicked?period=30d&limit=5")
    assert response.status_code == 200
    items = response.json()["top_clicked"]
    assert len(items) >= 1
    assert items[0]["clicks"] == 3


@pytest.mark.asyncio
async def test_nibsy_stats_with_clicks(client) -> None:
    await client.post(
        "/api/track/click",
        json={"content_url": "/learn/docker/", "source_page": "/"},
    )
    await client.post(
        "/api/track/impression",
        json={"content_ids": [1, 2], "source_page": "/"},
    )
    response = await client.get("/api/analytics/nibsy-stats")
    body = response.json()
    assert body["total_clicks_today"] >= 1
    assert body["total_impressions_today"] >= 2
