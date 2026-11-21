"""Finished courses on a learner's public profile, opt-in (#638)."""

from __future__ import annotations

import time

import pytest

from nibsy_api.ingest import ingest_catalogue

from .conftest import make_admin_header
from .test_learn import CATALOGUE

ALICE = make_admin_header("alice")


async def _finish(client, course: str, headers=ALICE) -> None:
    now = int(time.time() * 1000)
    response = await client.post(
        "/api/me/progress",
        json={"courses": [{"course": course, "visited": ["00_intro"], "updated": now, "completed": now}]},
        headers=headers,
    )
    assert response.status_code == 200


@pytest.mark.asyncio
async def test_finished_courses_are_private_until_turned_on(client, session) -> None:
    await ingest_catalogue(CATALOGUE, session)
    await _finish(client, "/learn/servos/")
    # Started but not finished: never listed.
    await client.post(
        "/api/me/progress", json={"courses": [{"course": "/learn/rust/", "visited": ["00_intro"]}]}, headers=ALICE
    )

    assert (await client.get("/api/me/learning-settings", headers=ALICE)).json()["show_courses"] is False
    assert (await client.get("/api/learners/alice/courses")).json()["courses"] == []

    response = await client.post("/api/me/learning-settings", json={"show_courses": True}, headers=ALICE)
    assert response.json()["show_courses"] is True
    courses = (await client.get("/api/learners/alice/courses")).json()["courses"]
    assert [c["url"] for c in courses] == ["/learn/servos/00_intro.html"]
    assert courses[0]["cover"] == "/learn/servos/cover.jpg"

    await client.post("/api/me/learning-settings", json={"show_courses": False}, headers=ALICE)
    assert (await client.get("/api/learners/alice/courses")).json()["courses"] == []


@pytest.mark.asyncio
async def test_settings_need_sign_in_and_a_boolean(client) -> None:
    assert (await client.post("/api/me/learning-settings", json={"show_courses": True})).status_code == 401
    bad = await client.post("/api/me/learning-settings", json={"show_courses": "yes"}, headers=ALICE)
    assert bad.status_code == 422
    assert (await client.get("/api/learners/nobody/courses")).json()["courses"] == []
