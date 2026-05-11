"""Ingestion tests against the fixtures directory."""

from __future__ import annotations

from pathlib import Path

import pytest
from sqlalchemy import select

from nibsy_api.ingest import ingest_from_data_dir
from nibsy_api.models import NibsyContent

FIXTURES = Path(__file__).resolve().parent / "fixtures" / "_data"


@pytest.mark.asyncio
async def test_ingest_populates_content(session) -> None:
    """A first ingest should add rows for every fixture entry."""

    stats = await ingest_from_data_dir(FIXTURES, session)

    assert stats.errors == 0, stats.error_details
    # 3 courses + 3 posts + 3 youtube + 3 robots + 2 reviews (one is skipped) + 3 glossary = 17
    assert stats.added == 17
    assert stats.updated >= 2  # popular_videos merges into 2 of the 3 youtube rows

    rows = (await session.scalars(select(NibsyContent))).all()
    urls = {r.url for r in rows}

    assert "/learn/example/00_intro.html" in urls
    assert "/2025-01-01-example.md" in urls
    assert "https://youtube.com/watch?v=TEST_VIDEO_ID_1" in urls
    assert "/projects/example-robot/" in urls
    assert "https://example.com/products/example" in urls
    assert "/glossary/test-term-one" in urls


@pytest.mark.asyncio
async def test_ingest_idempotent(session) -> None:
    """Re-running ingest with no source changes should report all unchanged."""

    first = await ingest_from_data_dir(FIXTURES, session)
    assert first.errors == 0

    second = await ingest_from_data_dir(FIXTURES, session)
    assert second.errors == 0
    assert second.added == 0
    # All youtube rows already have popular_videos metadata applied, so they
    # come back as unchanged on the second pass too.
    assert second.updated == 0
    assert second.unchanged > 0


@pytest.mark.asyncio
async def test_popular_videos_merges_metadata(session) -> None:
    """popular_videos.yaml updates existing video metadata, doesn't create rows."""

    await ingest_from_data_dir(FIXTURES, session)

    # The synthesised YouTube URL is the upsert key.
    row = await session.scalar(
        select(NibsyContent).where(
            NibsyContent.url == "https://youtube.com/watch?v=TEST_VIDEO_ID_1"
        )
    )
    assert row is not None
    meta = row.content_metadata or {}
    assert meta.get("views") == 12345
    assert meta.get("popular") is True
    assert meta.get("video_id") == "TEST_VIDEO_ID_1"

    # The popular_videos.yaml entry with id TEST_VIDEO_ID_MISSING must NOT
    # have spawned a new content row.
    missing = await session.scalar(
        select(NibsyContent).where(
            NibsyContent.url == "https://youtube.com/watch?v=TEST_VIDEO_ID_MISSING"
        )
    )
    assert missing is None


@pytest.mark.asyncio
async def test_reviews_placeholder_skipped(session) -> None:
    """Reviews with `excerpt: null` or `title: title` should be skipped."""

    await ingest_from_data_dir(FIXTURES, session)
    placeholder = await session.scalar(
        select(NibsyContent).where(
            NibsyContent.url == "https://example.com/products/placeholder"
        )
    )
    assert placeholder is None


@pytest.mark.asyncio
async def test_former_stubs_are_implemented(client) -> None:
    """Routes that were 501 stubs should now return real responses."""

    for method, path, expected in [
        ("GET", "/api/trending", 200),
        ("POST", "/api/track/click", 422),  # missing body → validation error
        ("POST", "/api/track/impression", 422),
    ]:
        response = await client.request(method, path)
        assert response.status_code == expected, f"{method} {path} returned {response.status_code}"
