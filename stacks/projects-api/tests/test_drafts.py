"""Draft projects are private to their author (issue #260)."""

from __future__ import annotations

import pytest

from .conftest import make_auth_header


async def _create(client, headers, **fields) -> dict:
    payload = {"title": "Secret Robot", "short_description": "Not ready yet", "status": "draft"}
    payload.update(fields)
    resp = await client.post("/api/projects", json=payload, headers=headers)
    assert resp.status_code == 201, resp.text
    return resp.json()


async def _publish(client, headers, project_id: int) -> None:
    resp = await client.put(
        f"/api/projects/{project_id}", json={"status": "completed"}, headers=headers
    )
    assert resp.status_code == 200, resp.text


@pytest.mark.asyncio
async def test_project_can_be_created_as_draft(client) -> None:
    project = await _create(client, make_auth_header("alice"))
    assert project["status"] == "draft"


@pytest.mark.asyncio
async def test_create_defaults_to_public_wip(client) -> None:
    resp = await client.post(
        "/api/projects", json={"title": "Open Robot"}, headers=make_auth_header("alice")
    )
    assert resp.json()["status"] == "wip"
    assert (await client.get(f"/api/projects/{resp.json()['id']}")).status_code == 200


@pytest.mark.asyncio
async def test_draft_is_404_for_others_but_visible_to_owner(client) -> None:
    alice = make_auth_header("alice")
    project = await _create(client, alice)
    pid = project["id"]

    assert (await client.get(f"/api/projects/{pid}", headers=alice)).status_code == 200
    assert (await client.get(f"/api/projects/{pid}")).status_code == 404
    bob = make_auth_header("bob")
    assert (await client.get(f"/api/projects/{pid}", headers=bob)).status_code == 404

    by_slug = f"/api/projects/by-slug/alice/{project['slug']}"
    assert (await client.get(by_slug, headers=alice)).status_code == 200
    assert (await client.get(by_slug)).status_code == 404

    for sub in ("images", "files", "bom", "links", "journal", "videos", "makes"):
        assert (await client.get(f"/api/projects/{pid}/{sub}")).status_code == 404, sub
        assert (
            await client.get(f"/api/projects/{pid}/{sub}", headers=alice)
        ).status_code == 200, sub
    assert (
        await client.get(f"/api/projects/alice/{project['slug']}/images")
    ).status_code == 404


@pytest.mark.asyncio
async def test_draft_absent_from_hub_and_search_until_published(client) -> None:
    alice = make_auth_header("alice")
    project = await _create(client, alice, title="Hidden Walker")

    def ids(resp):
        return [p["id"] for p in resp.json()]

    assert project["id"] not in ids(await client.get("/api/projects"))
    assert project["id"] not in ids(await client.get("/api/projects?search=Hidden"))
    assert project["id"] not in ids(await client.get("/api/projects?author=alice"))
    # The owner's own profile listing includes their drafts.
    assert project["id"] in ids(
        await client.get("/api/projects?author=alice", headers=alice)
    )
    # And My Projects always does.
    assert project["id"] in ids(await client.get("/api/projects/my/list", headers=alice))

    await _publish(client, alice, project["id"])
    assert project["id"] in ids(await client.get("/api/projects"))
    assert (await client.get(f"/api/projects/{project['id']}")).status_code == 200


@pytest.mark.asyncio
async def test_draft_logs_no_activity_until_published(client) -> None:
    alice = make_auth_header("alice")
    project = await _create(client, alice)
    await client.put(
        f"/api/projects/{project['id']}", json={"title": "Secret Robot Mk2"}, headers=alice
    )
    feed = (await client.get("/api/users/alice/activity")).json()
    assert feed["items"] == []

    await _publish(client, alice, project["id"])
    feed = (await client.get("/api/users/alice/activity")).json()
    assert [i["kind"] for i in feed["items"]] == ["project_published"]


@pytest.mark.asyncio
async def test_others_cannot_remix_a_draft(client) -> None:
    alice = make_auth_header("alice")
    project = await _create(client, alice)
    resp = await client.post(
        f"/api/projects/{project['id']}/remix",
        json={"remix_description": "Trying to fork a draft"},
        headers=make_auth_header("bob"),
    )
    assert resp.status_code == 404


@pytest.mark.asyncio
async def test_social_card_hides_drafts(client) -> None:
    alice = make_auth_header("alice")
    project = await _create(client, alice)
    resp = await client.get(f"/og/alice/{project['slug']}")
    assert resp.status_code == 404


@pytest.mark.asyncio
async def test_draft_image_and_file_urls_are_private(client) -> None:
    alice = make_auth_header("alice")
    project = await _create(client, alice)
    pid = project["id"]
    img = await client.post(
        f"/api/projects/{pid}/images",
        files={"file": ("photo.png", b"\x89PNG\r\n\x1a\n" + b"\x00" * 64, "image/png")},
        headers=alice,
    )
    assert img.status_code == 201, img.text
    f = await client.post(
        f"/api/projects/{pid}/files",
        files={"file": ("code.py", b"print('hi')", "text/plain")},
        headers=alice,
    )
    assert f.status_code == 201, f.text
    view = f"/api/projects/{pid}/images/{img.json()['id']}/view"
    download = f"/api/projects/{pid}/files/{f.json()['id']}/download"

    for url in (view, download):
        assert (await client.get(url)).status_code == 404, url
        assert (await client.get(url, headers=make_auth_header("bob"))).status_code == 404, url
        own = await client.get(url, headers=alice)
        assert own.status_code == 200, url
        assert own.headers["cache-control"] == "private, no-store"

    await _publish(client, alice, pid)
    for url in (view, download):
        resp = await client.get(url)
        assert resp.status_code == 200, url
        assert "cache-control" not in resp.headers
