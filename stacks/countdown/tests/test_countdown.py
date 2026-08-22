from __future__ import annotations

import pytest


@pytest.mark.asyncio
async def test_health(client) -> None:
    response = await client.get("/health")

    assert response.status_code == 200
    assert response.text == "ok"


@pytest.mark.asyncio
async def test_creator_page_renders(client) -> None:
    response = await client.get("/")

    assert response.status_code == 200
    assert "Make every second count." in response.text
    assert "Share or embed" in response.text
    assert "frame-ancestors *" in response.headers["content-security-policy"]


@pytest.mark.asyncio
async def test_shared_countdown_is_rendered_safely(client) -> None:
    response = await client.get(
        "/",
        params={
            "at": "2030-01-02T15:04:00+00:00",
            "description": 'Robot launch "day"',
            "image": "https://example.com/robot.jpg",
        },
    )

    assert response.status_code == 200
    assert 'data-at="2030-01-02T15:04:00+00:00"' in response.text
    assert "Robot launch &#34;day&#34;" in response.text
    assert "https://example.com/robot.jpg" in response.text
    assert response.headers["cache-control"] == "no-store, max-age=0"


@pytest.mark.asyncio
async def test_date_only_countdown_mode_is_rendered(client) -> None:
    response = await client.get(
        "/",
        params={
            "at": "2030-01-02T00:00:00+00:00",
            "date_only": "1",
        },
    )

    assert response.status_code == 200
    assert 'data-date-only="true"' in response.text


@pytest.mark.asyncio
async def test_invalid_timestamp_returns_creator_with_error(client) -> None:
    response = await client.get("/", params={"at": "tomorrow"})

    assert response.status_code == 200
    assert 'data-at=""' in response.text
    assert "shared countdown date is invalid" in response.text


@pytest.mark.asyncio
async def test_unsafe_image_scheme_is_rejected(client) -> None:
    response = await client.get(
        "/",
        params={
            "at": "2030-01-02T15:04:00+00:00",
            "image": "javascript:alert(1)",
        },
    )

    assert response.status_code == 200
    assert 'data-image=""' in response.text
    assert "must use a valid http:// or https:// URL" in response.text
