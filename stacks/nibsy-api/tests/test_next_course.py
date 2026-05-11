"""Next-course recommendation endpoint tests (#76)."""

from __future__ import annotations

import pytest


@pytest.mark.asyncio
async def test_next_course_by_url(client) -> None:
    response = await client.get(
        "/api/recommendations/next-course?current=/learn/example/00_intro.html"
    )
    assert response.status_code == 200
    body = response.json()
    assert body["source"] == "related"
    assert isinstance(body["next_courses"], list)


@pytest.mark.asyncio
async def test_next_course_by_id(client) -> None:
    response = await client.get("/api/recommendations/next-course?current=1")
    if response.status_code == 404:
        pytest.skip("content_id=1 is not a course in fixtures")
    assert response.status_code == 200
    body = response.json()
    assert body["source"] in ("related", "pathway")


@pytest.mark.asyncio
async def test_next_course_not_found(client) -> None:
    response = await client.get(
        "/api/recommendations/next-course?current=/nonexistent/course"
    )
    assert response.status_code == 404


@pytest.mark.asyncio
async def test_next_course_excludes_self(client) -> None:
    response = await client.get(
        "/api/recommendations/next-course?current=/learn/example/00_intro.html"
    )
    assert response.status_code == 200
    for course in response.json().get("next_courses", []):
        assert course["url"] != "/learn/example/00_intro.html"
