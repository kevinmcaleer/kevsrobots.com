"""Tests for ingesting the site catalogue (nibsy-catalogue.json)."""

from __future__ import annotations

import pytest
from sqlalchemy import select

from nibsy_api.ingest import ingest_catalogue
from nibsy_api.models import NibsyContent
from nibsy_api.trending import _fetch_all_page_views


def _post(slug: str, **extra) -> dict:
    return {"type": "post", "title": slug, "url": f"/blog/{slug}.html", "tags": ["Pico"], "date": "2026-01-01", **extra}


@pytest.mark.asyncio
async def test_catalogue_rows_keep_tags_and_real_urls(session) -> None:
    stats = await ingest_catalogue({"items": [_post("a", cover="/c.jpg")]}, session)
    assert stats.added == 1
    row = await session.scalar(select(NibsyContent))
    assert row.url == "/blog/a.html"
    assert row.tags == ["pico"]
    assert row.content_metadata == {"cover": "/c.jpg"}


@pytest.mark.asyncio
async def test_first_url_wins_and_bad_items_are_skipped(session) -> None:
    data = {
        "items": [
            _post("a"),
            {"type": "robot", "title": "A robot", "url": "/blog/a.html", "tags": []},
            {"type": "glossary", "title": "Nope", "url": "/glossary/nope"},
            {"type": "post", "title": "", "url": "/blog/untitled.html"},
        ]
    }
    await ingest_catalogue(data, session)
    rows = (await session.scalars(select(NibsyContent))).all()
    assert [(r.url, r.content_type) for r in rows] == [("/blog/a.html", "post")]


@pytest.mark.asyncio
async def test_pages_missing_from_the_catalogue_are_removed(session) -> None:
    await ingest_catalogue({"items": [_post("a"), _post("b"), _post("c")]}, session)
    stats = await ingest_catalogue({"items": [_post("a"), _post("b")]}, session)
    assert stats.removed == 1
    urls = set((await session.scalars(select(NibsyContent.url))).all())
    assert urls == {"/blog/a.html", "/blog/b.html"}


@pytest.mark.asyncio
async def test_truncated_catalogue_does_not_wipe_the_table(session) -> None:
    await ingest_catalogue({"items": [_post(s) for s in "abcde"]}, session)
    stats = await ingest_catalogue({"items": [_post("a")]}, session)
    assert stats.removed == 0
    assert stats.error_details
    assert len((await session.scalars(select(NibsyContent))).all()) == 5


@pytest.mark.asyncio
async def test_empty_catalogue_is_an_error(session) -> None:
    stats = await ingest_catalogue({"items": []}, session)
    assert stats.errors == 1


def test_page_views_are_folded_onto_one_key_per_page(tmp_path) -> None:
    import sqlite3
    from datetime import datetime, timedelta

    db = tmp_path / "pagecount.db"
    conn = sqlite3.connect(db)
    conn.execute("CREATE TABLE page_visits (page_url TEXT, timestamp TIMESTAMP)")
    recent = str(datetime.now() - timedelta(hours=30))
    old = str(datetime.now() - timedelta(days=400))
    conn.executemany(
        "INSERT INTO page_visits VALUES (?, ?)",
        [
            ("https://www.kevsrobots.com/blog/a.html", recent),
            ("https://kevsrobots.com/blog/a.html?utm_source=x", recent),
            ("https://www.kevsrobots.com/blog/a", recent),
            ("https://www.kevsrobots.com/blog/a.html", old),
            ("https://www.kevsrobots.com/learn/", recent),
        ],
    )
    conn.commit()
    conn.close()

    counts = _fetch_all_page_views(f"sqlite:///{db}")
    assert counts == {"/blog/a": 3, "/learn": 1}
