"""Made with Snakie / BetterCAD, the following feed and pinned projects
(homepage showcase and profile overhaul)."""

from __future__ import annotations

import pytest

from projects_api.models import ProjectImport, ProjectWiring

from .conftest import make_auth_header


async def _project(client, owner: str, title: str, status: str = "completed") -> int:
    headers = make_auth_header(owner)
    r = await client.post("/api/projects", json={"title": title}, headers=headers)
    assert r.status_code == 201, r.text
    pid = r.json()["id"]
    if status != "wip":
        r = await client.put(f"/api/projects/{pid}", json={"status": status}, headers=headers)
        assert r.status_code == 200, r.text
    return pid


async def _snakie(session, pid: int) -> None:
    session.add(ProjectWiring(project_id=pid, data={"board": "pico"}))
    await session.commit()


async def _bettercad(session, pid: int, status: str = "applied") -> None:
    session.add(ProjectImport(
        project_id=pid, source="bettercad", status=status, title="Robot arm",
        steps=1, pictures=0, files=0, archive_path="x.zip", archive_size=1,
    ))
    await session.commit()


# --- made_with ------------------------------------------------------------


@pytest.mark.asyncio
async def test_cards_say_which_apps_made_them(client, session) -> None:
    both = await _project(client, "alice", "Robot arm")
    snakie = await _project(client, "alice", "Line follower")
    plain = await _project(client, "alice", "Soldering tips")
    pending = await _project(client, "alice", "Walker")
    await _snakie(session, both)
    await _bettercad(session, both)
    await _snakie(session, snakie)
    await _bettercad(session, pending, status="pending")

    cards = {p["id"]: p["made_with"] for p in (await client.get("/api/projects")).json()}
    assert cards[both] == ["snakie", "bettercad"]
    assert cards[snakie] == ["snakie"]
    assert cards[plain] == []
    assert cards[pending] == []  # not imported yet

    detail = (await client.get(f"/api/projects/{both}")).json()
    assert detail["made_with"] == ["snakie", "bettercad"]


@pytest.mark.asyncio
async def test_gallery_filters_by_app(client, session) -> None:
    a = await _project(client, "alice", "Robot arm")
    b = await _project(client, "alice", "Line follower")
    await _project(client, "alice", "Soldering tips")
    await _bettercad(session, a)
    await _snakie(session, b)

    snakie = [p["id"] for p in (await client.get("/api/projects?made_with=snakie")).json()]
    bettercad = [p["id"] for p in (await client.get("/api/projects?made_with=bettercad")).json()]
    assert snakie == [b]
    assert bettercad == [a]
    assert (await client.get("/api/projects?made_with=fusion")).status_code == 422


@pytest.mark.asyncio
async def test_featured_filters_by_app(client, session) -> None:
    a = await _project(client, "alice", "Robot arm")
    b = await _project(client, "alice", "Line follower")
    for pid in (a, b):
        r = await client.post(
            f"/api/admin/projects/{pid}/feature", json={}, headers=make_auth_header("kev")
        )
        assert r.status_code == 200, r.text
    await _snakie(session, b)

    everything = (await client.get("/api/projects/featured")).json()
    assert {p["id"]: p["made_with"] for p in everything} == {a: [], b: ["snakie"]}
    only = (await client.get("/api/projects/featured?made_with=snakie")).json()
    assert [p["id"] for p in only] == [b]


# --- following feed -----------------------------------------------------------


@pytest.mark.asyncio
async def test_feed_shows_people_you_follow(client) -> None:
    await _project(client, "bob", "Bob's robot")
    await _project(client, "carol", "Carol's robot")
    await client.post("/api/users/bob/follow", headers=make_auth_header("alice"))

    r = await client.get("/api/users/me/feed", headers=make_auth_header("alice"))
    assert r.status_code == 200, r.text
    items = r.json()["items"]
    assert items and {i["username"] for i in items} == {"bob"}
    assert any(i["subject_title"] == "Bob's robot" for i in items)


@pytest.mark.asyncio
async def test_feed_needs_sign_in(client) -> None:
    assert (await client.get("/api/users/me/feed")).status_code == 401


@pytest.mark.asyncio
async def test_feed_is_empty_when_following_nobody(client) -> None:
    await _project(client, "bob", "Bob's robot")
    r = await client.get("/api/users/me/feed", headers=make_auth_header("alice"))
    assert r.json()["items"] == []


# --- pinned projects and builds_with ---------------------------------------------


@pytest.mark.asyncio
async def test_pin_projects_in_order(client, session) -> None:
    a = await _project(client, "alice", "Robot arm")
    b = await _project(client, "alice", "Walker")
    await _snakie(session, b)
    headers = make_auth_header("alice")

    r = await client.put("/api/users/me/profile", json={"pinned_project_ids": [b, a, b]}, headers=headers)
    assert r.status_code == 200, r.text
    body = r.json()
    assert body["pinned_project_ids"] == [b, a]
    assert [p["id"] for p in body["pinned"]] == [b, a]
    assert body["pinned"][0]["made_with"] == ["snakie"]

    # Other fields are left alone by a later edit that doesn't send pins.
    r = await client.put("/api/users/me/profile", json={"bio": "Hi"}, headers=headers)
    assert r.json()["pinned_project_ids"] == [b, a]

    r = await client.put("/api/users/me/profile", json={"pinned_project_ids": []}, headers=headers)
    assert r.json()["pinned"] == []


@pytest.mark.asyncio
async def test_cannot_pin_someone_elses_or_a_draft(client) -> None:
    theirs = await _project(client, "bob", "Bob's robot")
    draft = await _project(client, "alice", "Secret", status="draft")
    headers = make_auth_header("alice")
    for pid in (theirs, draft):
        r = await client.put("/api/users/me/profile", json={"pinned_project_ids": [pid]}, headers=headers)
        assert r.status_code == 400, r.text


@pytest.mark.asyncio
async def test_cannot_pin_more_than_three(client) -> None:
    ids = [await _project(client, "alice", f"Project {i}") for i in range(4)]
    r = await client.put(
        "/api/users/me/profile", json={"pinned_project_ids": ids}, headers=make_auth_header("alice")
    )
    assert r.status_code == 422


@pytest.mark.asyncio
async def test_archived_pin_drops_out_of_profile(client) -> None:
    a = await _project(client, "alice", "Robot arm")
    headers = make_auth_header("alice")
    await client.put("/api/users/me/profile", json={"pinned_project_ids": [a]}, headers=headers)
    await client.put(f"/api/projects/{a}", json={"status": "archived"}, headers=headers)
    body = (await client.get("/api/users/alice/profile")).json()
    assert body["pinned_project_ids"] == [a]
    assert body["pinned"] == []


@pytest.mark.asyncio
async def test_profile_counts_builds_with(client, session) -> None:
    a = await _project(client, "alice", "Robot arm")
    b = await _project(client, "alice", "Walker")
    await _project(client, "alice", "Soldering")
    d = await _project(client, "alice", "Secret", status="draft")
    await _snakie(session, a)
    await _snakie(session, b)
    await _bettercad(session, b)
    await _snakie(session, d)  # drafts don't count

    body = (await client.get("/api/users/alice/profile")).json()
    assert body["builds_with"] == {"snakie": 2, "bettercad": 1}
