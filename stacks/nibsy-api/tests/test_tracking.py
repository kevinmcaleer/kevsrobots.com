"""Click and impression tracking endpoint tests (#68)."""

from __future__ import annotations

import pytest


@pytest.mark.asyncio
async def test_track_click(client) -> None:
    response = await client.post(
        "/api/track/click",
        json={
            "content_url": "/learn/micropython/",
            "source_page": "/blog/pico-project.html",
            "content_id": 1,
            "session_id": "test-session",
        },
    )
    assert response.status_code == 200
    assert response.json()["status"] == "ok"


@pytest.mark.asyncio
async def test_track_click_minimal(client) -> None:
    response = await client.post(
        "/api/track/click",
        json={
            "content_url": "/learn/micropython/",
            "source_page": "/blog/pico-project.html",
        },
    )
    assert response.status_code == 200


@pytest.mark.asyncio
async def test_track_impression(client) -> None:
    response = await client.post(
        "/api/track/impression",
        json={
            "content_ids": [1, 2, 3],
            "source_page": "/blog/pico-project.html",
            "session_id": "test-session",
        },
    )
    assert response.status_code == 200
    body = response.json()
    assert body["status"] == "ok"
    assert body["recorded"] == 3


@pytest.mark.asyncio
async def test_track_impression_empty(client) -> None:
    response = await client.post(
        "/api/track/impression",
        json={
            "content_ids": [],
            "source_page": "/blog/pico-project.html",
        },
    )
    assert response.status_code == 200
    assert response.json()["recorded"] == 0
