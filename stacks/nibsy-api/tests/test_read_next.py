"""The "If you liked this" section on blog posts: `POST /api/recommendations`."""

from __future__ import annotations

import time

import pytest
from sqlalchemy import func, select

from nibsy_api.ingest import ingest_catalogue
from nibsy_api.models import NibsyPageView

from .conftest import make_admin_header


def _item(type_, title, url, tags, **extra):
    return {"type": type_, "title": title, "url": url, "description": f"About {title}", "tags": tags, "date": "2026-01-01", **extra}


CATALOGUE = {
    "version": 1,
    "items": [
        _item("post", "Pico OLED clock", "/blog/pico-oled-clock.html", ["micropython", "pico", "oled"], cover="/assets/img/clock.jpg", videos=["vid123"]),
        _item("post", "Adding OLED displays with MicroPython", "/blog/micropython-oled.html", ["micropython", "oled"]),
        _item("post", "MicroPython on the Pico W", "/blog/pico-w.html", ["micropython", "pico"]),
        _item("post", "Pico servo driver", "/blog/pico-servo.html", ["pico", "servo"]),
        _item("post", "Docker Swarm on a Pi cluster", "/blog/swarm-cluster.html", ["docker", "raspberry pi", "clusters"]),
        _item("post", "Podman vs Docker", "/blog/podman-vs-docker.html", ["docker", "containers"]),
        _item("post", "Kubernetes at home", "/blog/k3s.html", ["docker", "clusters", "kubernetes"]),
        _item("post", "Laser cut robot chassis", "/blog/laser-chassis.html", ["laser cutting"]),
        _item("course", "Learn MicroPython", "/learn/micropython/00_intro.html", ["micropython", "pico"], cover="/learn/micropython/cover.jpg", level="beginner"),
        _item("course", "Docker for makers", "/learn/docker/00_intro.html", ["docker", "containers"]),
        _item("course", "Build a SMARS robot", "/learn/smars/00_intro.html", ["arduino"]),
        _item("video", "MicroPython OLED tutorial for the Pico", "https://youtube.com/watch?v=aaa", ["micropython", "oled"], video_id="aaa"),
    ],
}


async def _seed(session) -> None:
    stats = await ingest_catalogue(CATALOGUE, session)
    assert stats.errors == 0


async def _post(client, headers=None, **body) -> dict:
    response = await client.post("/api/recommendations", json=body, headers=headers or {})
    assert response.status_code == 200, response.text
    assert response.headers["cache-control"] == "private, no-store"
    return response.json()


@pytest.mark.asyncio
async def test_posts_only_with_a_related_course(client, session) -> None:
    await _seed(session)
    body = await _post(client, page="/blog/micropython-oled.html", limit=3, types=["post"], courses=1)

    recs = body["recommendations"]
    assert len(recs) == 3
    assert {r["type"] for r in recs} == {"post"}
    assert "/blog/micropython-oled.html" not in [r["url"] for r in recs]
    assert [c["url"] for c in body["courses"]] == ["/learn/micropython/00_intro.html"]
    assert body["courses"][0]["level"] == "beginner"


@pytest.mark.asyncio
async def test_cards_carry_cover_description_and_video(client, session) -> None:
    await _seed(session)
    body = await _post(client, page="/blog/micropython-oled.html", limit=3, types=["post"])
    clock = next(r for r in body["recommendations"] if r["url"] == "/blog/pico-oled-clock.html")
    assert clock["cover"] == "/assets/img/clock.jpg"
    assert clock["video_id"] == "vid123"
    assert clock["description"] == "About Pico OLED clock"
    assert clock["date"] == "2026-01-01"


@pytest.mark.asyncio
async def test_browser_history_steers_and_is_not_suggested_again(client, session) -> None:
    await _seed(session)
    # A page with no topic of its own: only the reader's history can steer it.
    now_ms = int(time.time() * 1000)
    seen = [
        {"page": "/blog/swarm-cluster.html", "at": now_ms},
        {"page": "https://www.kevsrobots.com/blog/podman-vs-docker.html", "at": now_ms - 86_400_000},
    ]
    body = await _post(client, page="/blog/", limit=3, types=["post"], seen=seen, courses=1)

    urls = [r["url"] for r in body["recommendations"]]
    assert urls[0] == "/blog/k3s.html"
    assert "/blog/swarm-cluster.html" not in urls, "already read"
    assert "/blog/podman-vs-docker.html" not in urls, "already read"
    assert body["personalised"] is True
    assert body["recommendations"][0]["reason"].startswith("Because you read")
    assert body["courses"][0]["url"] == "/learn/docker/00_intro.html"


@pytest.mark.asyncio
async def test_browser_history_is_never_stored(client, session) -> None:
    await _seed(session)
    await _post(client, page="/blog/pico-w.html", types=["post"], seen=[{"page": "/blog/k3s.html"}])
    await _post(client, headers=make_admin_header("alice"), page="/blog/pico-w.html", seen=[{"page": "/blog/k3s.html"}])
    assert await session.scalar(select(func.count()).select_from(NibsyPageView)) == 0


@pytest.mark.asyncio
async def test_expired_and_unknown_history_is_ignored(client, session) -> None:
    await _seed(session)
    old = int((time.time() - 200 * 86_400) * 1000)
    body = await _post(
        client,
        page="/blog/swarm-cluster.html",
        limit=3,
        types=["post"],
        seen=[
            {"page": "/blog/k3s.html", "at": old},  # past the 90-day window
            {"page": "/not/a/page.html"},  # not in the catalogue
            {"page": "/blog/laser-chassis.html", "at": 10**20},  # nonsense time
        ],
    )
    urls = [r["url"] for r in body["recommendations"]]
    assert "/blog/k3s.html" in urls, "an expired view must not hide a good match"
    assert body["personalised"] is False


@pytest.mark.asyncio
async def test_logged_in_history_and_browser_history_combine(client, session) -> None:
    await _seed(session)
    alice = make_admin_header("alice")
    await client.post("/api/track/view", json={"page": "/blog/swarm-cluster.html"}, headers=alice)
    body = await _post(client, headers=alice, page="/blog/", limit=3, types=["post"], seen=[{"page": "/blog/podman-vs-docker.html"}])
    urls = [r["url"] for r in body["recommendations"]]
    assert body["logged_in"] is True
    assert "/blog/swarm-cluster.html" not in urls
    assert "/blog/podman-vs-docker.html" not in urls
    assert urls[0] == "/blog/k3s.html"


@pytest.mark.asyncio
async def test_fallback_respects_types(client, session) -> None:
    await _seed(session)
    body = await _post(client, page="/somewhere/else.html", limit=3, types=["post"])
    assert len(body["recommendations"]) == 3
    assert {r["type"] for r in body["recommendations"]} == {"post"}


@pytest.mark.asyncio
async def test_rejects_oversized_history(client, session) -> None:
    await _seed(session)
    response = await client.post(
        "/api/recommendations",
        json={"page": "/blog/k3s.html", "seen": [{"page": "/blog/x.html"}] * 101},
    )
    assert response.status_code == 422


@pytest.mark.asyncio
async def test_get_form_is_unchanged_for_the_widget(client, session) -> None:
    await _seed(session)
    response = await client.get("/api/recommendations", params={"page": "/blog/micropython-oled.html", "limit": 6})
    body = response.json()
    assert any(r["type"] == "video" for r in body["recommendations"])
    assert body["courses"] == []
