"""Learner course progress synced from lesson pages (#633)."""

from __future__ import annotations

import time

import pytest

from .conftest import make_admin_header

ALICE = make_admin_header("alice")
BOB = make_admin_header("bob")
COURSE = "/learn/micropython/"


def _now_ms(offset_s: float = 0) -> int:
    return int((time.time() + offset_s) * 1000)


async def _sync(client, courses, headers=ALICE):
    response = await client.post("/api/me/progress", json={"courses": courses}, headers=headers)
    assert response.status_code == 200, response.text
    return response.json()["courses"]


@pytest.mark.asyncio
async def test_progress_needs_sign_in(client) -> None:
    assert (await client.get("/api/me/progress")).status_code == 401
    assert (await client.post("/api/me/progress", json={"courses": []})).status_code == 401


@pytest.mark.asyncio
async def test_browser_record_is_stored_in_its_own_shape(client) -> None:
    stored = await _sync(
        client,
        [
            {
                "course": COURSE,
                # The browser keeps visited as an object; "index" is not a lesson.
                "visited": {"00_intro": True, "01_leds": True, "index": True},
                "answers": {"q1": True, "q2": False},
                "last": "01_leds",
                "updated": _now_ms(-60),
            }
        ],
    )
    assert len(stored) == 1
    rec = stored[0]
    assert rec["course"] == COURSE
    assert rec["visited"] == ["00_intro", "01_leds"]
    assert rec["answers"] == {"q1": True, "q2": False}
    assert rec["last"] == "01_leds"
    assert rec["completed"] is None
    body = (await client.get("/api/me/progress", headers=ALICE)).json()
    assert body["courses"] == stored
    # Bob sees none of it.
    assert (await client.get("/api/me/progress", headers=BOB)).json()["courses"] == []


@pytest.mark.asyncio
async def test_merging_two_devices_loses_nothing(client) -> None:
    await _sync(
        client,
        [{"course": COURSE, "visited": ["00_intro", "01_leds"], "answers": {"q1": False},
          "last": "01_leds", "updated": _now_ms(-600)}],
    )
    stored = await _sync(
        client,
        # A lesson URL resolves to its course; the later device's answer to
        # q1 does not replace the first, but its new answer to q2 is kept.
        [{"course": "/learn/micropython/03_buttons.html", "visited": ["00_intro", "03_buttons"],
          "answers": {"q1": True, "q2": True}, "last": "03_buttons", "updated": _now_ms(-10),
          "completed": _now_ms(-10)}],
    )
    rec = stored[0]
    assert rec["visited"] == ["00_intro", "01_leds", "03_buttons"]
    assert rec["answers"] == {"q1": False, "q2": True}
    assert rec["last"] == "03_buttons"
    assert rec["completed"] is not None

    # An older record doesn't move "last" back.
    stored = await _sync(
        client, [{"course": COURSE, "visited": ["02_pwm"], "last": "02_pwm", "updated": _now_ms(-3600)}]
    )
    assert stored[0]["last"] == "03_buttons"
    assert "02_pwm" in stored[0]["visited"]


@pytest.mark.asyncio
async def test_bad_records_are_ignored(client) -> None:
    stored = await _sync(
        client,
        [
            {"course": "/blog/not-a-course.html", "visited": ["a"]},
            {"course": "/learn/../etc/", "visited": ["a"]},
            {"course": "/learn/learning_pathways/robotics.html", "visited": ["a"]},
            {"course": "/learn/rust/", "visited": ["<script>", "00_intro"], "updated": _now_ms(10**7)},
            {"course": "/learn/empty/", "visited": [], "answers": {}},
            "nonsense",
        ],
    )
    assert [r["course"] for r in stored] == ["/learn/rust/"]
    assert stored[0]["visited"] == ["00_intro"]
    # A clock from the future is clamped to now.
    assert stored[0]["updated"] <= _now_ms(5)
    response = await client.post("/api/me/progress", json={"courses": "x"}, headers=ALICE)
    assert response.status_code == 422


@pytest.mark.asyncio
async def test_reset_one_course_or_all(client) -> None:
    await _sync(
        client,
        [{"course": COURSE, "visited": ["00_intro"]}, {"course": "/learn/rust/", "visited": ["00_intro"]}],
    )
    response = await client.delete("/api/me/progress", params={"course": COURSE + "02_x.html"}, headers=ALICE)
    assert response.json() == {"deleted": 1}
    remaining = (await client.get("/api/me/progress", headers=ALICE)).json()["courses"]
    assert [r["course"] for r in remaining] == ["/learn/rust/"]
    assert (await client.delete("/api/me/progress", headers=ALICE)).json() == {"deleted": 1}
    assert (await client.get("/api/me/progress", headers=ALICE)).json()["courses"] == []
