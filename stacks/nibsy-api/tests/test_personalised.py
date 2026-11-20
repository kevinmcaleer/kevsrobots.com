"""Personalised recommendations from a logged-in user's viewing history."""

from __future__ import annotations

from datetime import datetime, timedelta, timezone

import pytest
from sqlalchemy import select

from nibsy_api.history import purge_old_views
from nibsy_api.ingest import ingest_catalogue
from nibsy_api.models import NibsyContent, NibsyPageView
from nibsy_api.recommender import CatalogueIndex, View, build_profile

from .conftest import make_admin_header
from .test_recommendations_endpoint import CATALOGUE

def NOW() -> datetime:
    return datetime.now(timezone.utc).replace(tzinfo=None)


ALICE = make_admin_header("alice")
BOB = make_admin_header("bob")


async def _view(client, page: str, headers=ALICE) -> dict:
    response = await client.post("/api/track/view", json={"page": page}, headers=headers)
    assert response.status_code == 200
    return response.json()


async def _recs(client, page: str, headers=None, limit: int = 6) -> dict:
    response = await client.get(
        "/api/recommendations", params={"page": page, "limit": limit}, headers=headers or {}
    )
    assert response.status_code == 200
    return response.json()


@pytest.mark.asyncio
async def test_anonymous_views_are_never_stored(client, session) -> None:
    await ingest_catalogue(CATALOGUE, session)
    body = await _view(client, "/blog/swarm-cluster.html", headers={})
    assert body["recorded"] is False
    assert (await session.scalars(select(NibsyPageView))).all() == []

    recs = await _recs(client, "/")
    assert recs["logged_in"] is False
    assert recs["personalised"] is False


@pytest.mark.asyncio
async def test_history_shapes_recommendations_on_a_page_with_no_topic(client, session) -> None:
    """On the home page an anonymous visitor gets trending; Alice gets Docker."""

    await ingest_catalogue(CATALOGUE, session)
    await _view(client, "/blog/swarm-cluster.html")

    recs = await _recs(client, "/", ALICE, limit=3)
    assert recs["logged_in"] is True
    assert recs["personalised"] is True
    urls = [r["url"] for r in recs["recommendations"]]
    assert "/blog/podman-vs-docker.html" in urls
    assert "/blog/swarm-cluster.html" not in urls, "already read"
    personal = [r for r in recs["recommendations"] if r["url"] == "/blog/podman-vs-docker.html"]
    assert personal[0]["reason"] == "Because you read Docker Swarm on a Pi cluster"


@pytest.mark.asyncio
async def test_users_do_not_see_each_others_history(client, session) -> None:
    await ingest_catalogue(CATALOGUE, session)
    await _view(client, "/blog/swarm-cluster.html", ALICE)
    await _view(client, "/blog/pico-oled-clock.html", BOB)

    alice = [r["url"] for r in (await _recs(client, "/", ALICE, 3))["recommendations"]]
    bob = [r["url"] for r in (await _recs(client, "/", BOB, 3))["recommendations"]]
    assert "/blog/podman-vs-docker.html" in alice
    assert "/blog/micropython-oled.html" in bob
    assert "/blog/podman-vs-docker.html" not in bob


@pytest.mark.asyncio
async def test_current_page_still_leads(client, session) -> None:
    """History steers, but a Docker reader on an OLED post still gets OLED first."""

    await ingest_catalogue(CATALOGUE, session)
    await _view(client, "/blog/swarm-cluster.html")
    recs = (await _recs(client, "/blog/pico-oled-clock.html", ALICE))["recommendations"]
    assert recs[0]["reason"].startswith("More on "), "top pick comes from the page"
    assert "docker" not in recs[0]["url"]
    assert any("docker" in r["url"] or r["url"].endswith("=bbb") for r in recs)


@pytest.mark.asyncio
async def test_lesson_view_counts_as_its_course_and_repeat_views_add_up(client, session) -> None:
    await ingest_catalogue(CATALOGUE, session)
    assert (await _view(client, "/learn/docker/03_images.html"))["recorded"] is True
    await _view(client, "/learn/docker/04_volumes.html")
    assert (await _view(client, "/not/a/real/page"))["recorded"] is False

    rows = (await session.scalars(select(NibsyPageView))).all()
    assert len(rows) == 1
    assert rows[0].view_count == 2
    course = await session.get(NibsyContent, rows[0].content_id)
    assert course.url == "/learn/docker/00_intro.html"


@pytest.mark.asyncio
async def test_user_can_see_and_clear_their_history(client, session) -> None:
    await ingest_catalogue(CATALOGUE, session)
    await _view(client, "/blog/swarm-cluster.html", ALICE)
    await _view(client, "/blog/pico-oled-clock.html", BOB)

    assert (await client.get("/api/me/history")).status_code == 401
    listed = (await client.get("/api/me/history", headers=ALICE)).json()["history"]
    assert [h["url"] for h in listed] == ["/blog/swarm-cluster.html"]

    cleared = await client.delete("/api/me/history", headers=ALICE)
    assert cleared.json() == {"deleted": 1}
    assert (await _recs(client, "/", ALICE))["personalised"] is False
    # Bob's history is untouched.
    assert len((await client.get("/api/me/history", headers=BOB)).json()["history"]) == 1


@pytest.mark.asyncio
async def test_old_views_are_purged_and_ignored(client, session) -> None:
    await ingest_catalogue(CATALOGUE, session)
    await _view(client, "/blog/swarm-cluster.html")
    row = await session.scalar(select(NibsyPageView))
    row.last_viewed_at = NOW() - timedelta(days=120)
    await session.commit()

    assert (await _recs(client, "/", ALICE))["personalised"] is False
    assert await purge_old_views(session, NOW()) == 1


def test_recent_views_outweigh_old_ones() -> None:
    now = datetime(2026, 11, 20)
    rows = [
        NibsyContent(id=1, content_type="post", title="Docker", url="/a", tags=["docker"]),
        NibsyContent(id=2, content_type="post", title="Lasers", url="/b", tags=["laser"]),
    ]
    index = CatalogueIndex(rows)
    profile = build_profile(
        index,
        [View(1, now - timedelta(days=60)), View(2, now - timedelta(days=1))],
        now,
    )
    assert profile["laser"] > 3 * profile["docker"]
