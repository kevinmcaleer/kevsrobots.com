"""Tests for the live `GET /api/recommendations` endpoint."""

from __future__ import annotations

from datetime import datetime

import pytest

from nibsy_api.ingest import ingest_catalogue
from nibsy_api.models import NibsyContent, NibsyTrending
from nibsy_api.recommender import CatalogueIndex, normalise_url, recommend_for_page

NOW = datetime(2026, 11, 20)


def _item(type_: str, title: str, url: str, tags: list[str], date: str = "2026-01-01", description: str = "") -> dict:
    return {
        "type": type_,
        "title": title,
        "url": url,
        "description": description,
        "tags": tags,
        "date": date,
    }


CATALOGUE = {
    "version": 1,
    "items": [
        _item("post", "Pico OLED clock", "/blog/pico-oled-clock.html", ["micropython", "pico", "oled"]),
        _item("post", "Adding OLED displays with MicroPython", "/blog/micropython-oled.html", ["micropython", "oled"]),
        _item("post", "Docker Swarm on a Pi cluster", "/blog/swarm-cluster.html", ["docker", "raspberry pi", "clusters"]),
        _item("post", "Podman vs Docker", "/blog/podman-vs-docker.html", ["docker", "containers"]),
        _item("post", "Laser cut robot chassis", "/blog/laser-chassis.html", ["laser cutting"]),
        _item("course", "Learn MicroPython", "/learn/micropython/00_intro.html", ["micropython", "pico"]),
        _item("course", "Docker for makers", "/learn/docker/00_intro.html", ["docker"]),
        _item("course", "Build a SMARS robot", "/learn/smars/00_intro.html", ["arduino"]),
        _item("review", "Pimoroni Pico Display", "/reviews/pico-display.html", []),
        _item("video", "MicroPython OLED tutorial for the Pico", "https://youtube.com/watch?v=aaa", []),
        _item("video", "Docker on Raspberry Pi explained", "https://youtube.com/watch?v=bbb", []),
        _item("video", "Laser cutting a robot", "https://youtube.com/watch?v=ccc", []),
    ],
}


async def _seed(session) -> None:
    stats = await ingest_catalogue(CATALOGUE, session)
    assert stats.errors == 0


async def _urls(client, page: str, **params) -> list[str]:
    response = await client.get("/api/recommendations", params={"page": page, **params})
    assert response.status_code == 200
    return [r["url"] for r in response.json()["recommendations"]]


@pytest.mark.asyncio
async def test_different_pages_get_different_recommendations(client, session) -> None:
    """The original bug: every blog post returned the same list."""

    await _seed(session)
    oled = await _urls(client, "/blog/pico-oled-clock.html", limit=3)
    docker = await _urls(client, "/blog/swarm-cluster.html", limit=3)

    assert oled != docker
    assert "/blog/micropython-oled.html" in oled
    assert "/blog/podman-vs-docker.html" in docker
    assert "/blog/pico-oled-clock.html" not in oled, "a page must not recommend itself"


@pytest.mark.asyncio
async def test_response_shape_and_reason(client, session) -> None:
    await _seed(session)
    response = await client.get(
        "/api/recommendations", params={"page": "/blog/pico-oled-clock.html"}
    )
    body = response.json()
    assert body["basis"] == "page"
    assert body["generator_version"] == "live-v1"
    item = body["recommendations"][0]
    for field in ("content_id", "url", "title", "type", "reason", "score"):
        assert field in item
    assert item["reason"].startswith("More on ")


@pytest.mark.asyncio
async def test_untagged_video_is_matched_by_title(client, session) -> None:
    """Videos have no tags; the title has to carry them."""

    await _seed(session)
    urls = await _urls(client, "/blog/micropython-oled.html", limit=4)
    assert "https://youtube.com/watch?v=aaa" in urls
    assert "https://youtube.com/watch?v=bbb" not in urls


@pytest.mark.asyncio
async def test_lesson_page_uses_its_course(client, session) -> None:
    await _seed(session)
    response = await client.get(
        "/api/recommendations",
        params={"page": "/learn/docker/04_volumes.html", "limit": 3},
    )
    body = response.json()
    assert body["basis"] == "course"
    urls = [r["url"] for r in body["recommendations"]]
    assert "/learn/docker/00_intro.html" not in urls
    assert any("docker" in u for u in urls)


@pytest.mark.asyncio
async def test_url_spellings_resolve_to_the_same_page(client, session) -> None:
    await _seed(session)
    canonical = await _urls(client, "/blog/pico-oled-clock.html")
    assert await _urls(client, "/blog/pico-oled-clock") == canonical
    assert await _urls(client, "/blog/pico-oled-clock.html?utm=x#top") == canonical


@pytest.mark.asyncio
async def test_unknown_page_falls_back_to_its_section(client, session) -> None:
    await _seed(session)
    response = await client.get(
        "/api/recommendations", params={"page": "/learn/", "limit": 3}
    )
    body = response.json()
    assert body["basis"] == "fallback"
    assert len(body["recommendations"]) == 3
    assert {r["type"] for r in body["recommendations"]} == {"course"}


@pytest.mark.asyncio
async def test_fallback_prefers_trending(client, session) -> None:
    await _seed(session)
    rows = (await session.execute(NibsyContent.__table__.select())).all()
    by_url = {r.url: r.id for r in rows}
    session.add(NibsyTrending(content_id=by_url["/blog/laser-chassis.html"], trending_score=99))
    await session.commit()

    response = await client.get("/api/recommendations", params={"page": "/", "limit": 6})
    recs = response.json()["recommendations"]
    assert all(r["reason"] == "Popular right now" for r in recs)


@pytest.mark.asyncio
async def test_respects_limit_and_exclude(client, session) -> None:
    await _seed(session)
    response = await client.get(
        "/api/recommendations", params={"page": "/blog/pico-oled-clock.html", "limit": 2}
    )
    recs = response.json()["recommendations"]
    assert len(recs) == 2

    excluded = recs[0]["content_id"]
    response = await client.get(
        "/api/recommendations",
        params={"page": "/blog/pico-oled-clock.html", "exclude": f"{excluded},junk"},
    )
    assert excluded not in {r["content_id"] for r in response.json()["recommendations"]}


@pytest.mark.asyncio
async def test_new_content_is_recommended_without_a_regeneration(client, session) -> None:
    """Nothing is precomputed, so a re-ingest is visible on the next request."""

    await _seed(session)
    before = await _urls(client, "/blog/laser-chassis.html", limit=6)
    assert "/blog/laser-engraver.html" not in before

    bigger = {
        **CATALOGUE,
        "items": [
            *CATALOGUE["items"],
            _item("post", "Laser engraver upgrade", "/blog/laser-engraver.html", ["laser cutting"]),
        ],
    }
    await ingest_catalogue(bigger, session)
    after = await _urls(client, "/blog/laser-chassis.html", limit=6)
    assert "/blog/laser-engraver.html" in after


def test_normalise_url() -> None:
    assert normalise_url("https://www.kevsrobots.com/blog/x.html?a=1#b") == "/blog/x"
    assert normalise_url("/blog/x") == "/blog/x"
    assert normalise_url("/learn/") == "/learn"
    assert normalise_url("/index.html") == "/"
    assert normalise_url("https://youtube.com/watch?v=abc") == "https://youtube.com/watch?v=abc"


def test_multi_word_tags_match_however_they_are_written() -> None:
    rows = [
        NibsyContent(id=1, content_type="post", title="A", url="/a", tags=["raspberry pi"]),
        NibsyContent(id=2, content_type="post", title="B", url="/b", tags=["raspberrypi"]),
        NibsyContent(id=3, content_type="post", title="Raspberry Pi tips", url="/c", tags=[]),
        NibsyContent(id=4, content_type="post", title="Arduino", url="/d", tags=["arduino"]),
    ]
    picks, _ = recommend_for_page(CatalogueIndex(rows), "/a", 3, set(), {}, NOW)
    related = [p.item.url for p in picks if p.reason.startswith("More on")]
    assert set(related) == {"/b", "/c"}
