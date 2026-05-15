"""Tests for project remix / fork attribution (issue #108)."""

from __future__ import annotations

import pytest

from .conftest import make_auth_header


async def _create_project(client, headers, **fields) -> dict:
    payload = {"title": "Original Robot", "short_description": "A great bot"}
    payload.update(fields)
    resp = await client.post("/api/projects", json=payload, headers=headers)
    assert resp.status_code == 201, resp.text
    return resp.json()


@pytest.mark.asyncio
async def test_create_remix_links_to_original(client) -> None:
    alice = make_auth_header("alice")
    bob = make_auth_header("bob")

    original = await _create_project(client, alice, title="Original Robot")

    resp = await client.post(
        f"/api/projects/{original['id']}/remix",
        json={"remix_description": "Swapped the motor driver for a TB6612"},
        headers=bob,
    )
    assert resp.status_code == 201, resp.text
    remix = resp.json()

    assert remix["author_username"] == "bob"
    assert remix["title"].startswith("Remix of ")
    assert remix["remixed_from"]["id"] == original["id"]
    assert remix["remixed_from"]["title"] == "Original Robot"
    assert remix["remixed_from"]["author_username"] == "alice"
    assert remix["is_remix"] is True
    assert remix["remix_description"].startswith("Swapped the motor")
    assert remix["status"] == "wip"


@pytest.mark.asyncio
async def test_remix_appears_in_originals_remix_list(client) -> None:
    alice = make_auth_header("alice")
    bob = make_auth_header("bob")

    original = await _create_project(client, alice, title="Original Robot")
    remix_resp = await client.post(
        f"/api/projects/{original['id']}/remix",
        json={"remix_description": "Added an OLED display for battery info"},
        headers=bob,
    )
    remix = remix_resp.json()

    # Direct remixes list contains the new project.
    listing = await client.get(f"/api/projects/{original['id']}/remixes")
    assert listing.status_code == 200
    ids = [r["id"] for r in listing.json()]
    assert remix["id"] in ids

    # The original now reports a remix count of 1.
    refreshed = await client.get(f"/api/projects/{original['id']}")
    body = refreshed.json()
    assert body["remixes_count"] == 1
    assert body["is_remix"] is False


@pytest.mark.asyncio
async def test_remix_chain_returns_full_ancestry(client) -> None:
    alice = make_auth_header("alice")
    bob = make_auth_header("bob")
    carol = make_auth_header("carol")

    a = await _create_project(client, alice, title="Project A")

    b_resp = await client.post(
        f"/api/projects/{a['id']}/remix",
        json={"remix_description": "B is A but with smaller wheels"},
        headers=bob,
    )
    b = b_resp.json()

    c_resp = await client.post(
        f"/api/projects/{b['id']}/remix",
        json={"remix_description": "C is B but with a Pi Pico controller"},
        headers=carol,
    )
    c = c_resp.json()

    chain = await client.get(f"/api/projects/{c['id']}/remix-chain")
    assert chain.status_code == 200
    ids = [p["id"] for p in chain.json()]
    assert ids == [a["id"], b["id"], c["id"]]


@pytest.mark.asyncio
async def test_put_cannot_clear_remix_attribution(client) -> None:
    alice = make_auth_header("alice")
    bob = make_auth_header("bob")

    original = await _create_project(client, alice, title="Original Robot")
    remix_resp = await client.post(
        f"/api/projects/{original['id']}/remix",
        json={"remix_description": "Refactored the chassis for laser cutting"},
        headers=bob,
    )
    remix_id = remix_resp.json()["id"]

    # Attempt to clear the parent link.
    clear_attempt = await client.put(
        f"/api/projects/{remix_id}",
        json={"remixed_from_id": None},
        headers=bob,
    )
    assert clear_attempt.status_code == 400
    assert "permanent" in clear_attempt.json()["detail"].lower()

    # Attempt to change the description after the fact.
    change_attempt = await client.put(
        f"/api/projects/{remix_id}",
        json={"remix_description": "actually I changed nothing"},
        headers=bob,
    )
    assert change_attempt.status_code == 400

    # Sanity: normal updates still work and attribution survives.
    ok = await client.put(
        f"/api/projects/{remix_id}",
        json={"title": "My Improved Remix"},
        headers=bob,
    )
    assert ok.status_code == 200
    assert ok.json()["remixed_from"]["id"] == original["id"]
    assert ok.json()["remix_description"].startswith("Refactored")


@pytest.mark.asyncio
async def test_remix_requires_description(client) -> None:
    alice = make_auth_header("alice")
    bob = make_auth_header("bob")
    original = await _create_project(client, alice, title="Original Robot")

    missing = await client.post(
        f"/api/projects/{original['id']}/remix",
        json={},
        headers=bob,
    )
    assert missing.status_code == 422

    too_short = await client.post(
        f"/api/projects/{original['id']}/remix",
        json={"remix_description": "tiny"},
        headers=bob,
    )
    assert too_short.status_code == 422


@pytest.mark.asyncio
async def test_remix_requires_auth(client) -> None:
    alice = make_auth_header("alice")
    original = await _create_project(client, alice, title="Original Robot")
    resp = await client.post(
        f"/api/projects/{original['id']}/remix",
        json={"remix_description": "Anonymous would-be remixer"},
    )
    assert resp.status_code == 401


@pytest.mark.asyncio
async def test_remix_of_unknown_project_404s(client) -> None:
    bob = make_auth_header("bob")
    resp = await client.post(
        "/api/projects/99999/remix",
        json={"remix_description": "Trying to fork a ghost"},
        headers=bob,
    )
    assert resp.status_code == 404


@pytest.mark.asyncio
async def test_remix_with_custom_title(client) -> None:
    alice = make_auth_header("alice")
    bob = make_auth_header("bob")
    original = await _create_project(client, alice, title="Original Robot")
    resp = await client.post(
        f"/api/projects/{original['id']}/remix",
        json={
            "title": "Bob's Beefed-Up Bot",
            "remix_description": "Doubled the battery capacity",
        },
        headers=bob,
    )
    assert resp.status_code == 201
    assert resp.json()["title"] == "Bob's Beefed-Up Bot"


@pytest.mark.asyncio
async def test_list_projects_marks_remixes(client) -> None:
    alice = make_auth_header("alice")
    bob = make_auth_header("bob")
    original = await _create_project(client, alice, title="Original Robot")
    remix_resp = await client.post(
        f"/api/projects/{original['id']}/remix",
        json={"remix_description": "Added neopixel underlights"},
        headers=bob,
    )
    remix_id = remix_resp.json()["id"]

    listing = await client.get("/api/projects")
    by_id = {p["id"]: p for p in listing.json()}
    assert by_id[remix_id]["is_remix"] is True
    assert by_id[original["id"]]["is_remix"] is False
