"""Tests for the staff-picks endpoints (issue #115)."""

from __future__ import annotations

import pytest

from .conftest import make_auth_header

ADMIN_USER = "kev"


async def _create_project(client, *, owner: str = "maker", title: str = "A Build For Picks") -> int:
    resp = await client.post(
        "/api/projects",
        json={"title": title, "short_description": "desc"},
        headers=make_auth_header(owner),
    )
    assert resp.status_code == 201, resp.text
    return resp.json()["id"]


async def _create_pick(client, *, title: str = "Test Pick", is_published: bool = False) -> int:
    resp = await client.post(
        "/api/admin/staff-picks",
        json={"title": title, "description": "desc", "is_published": is_published},
        headers=make_auth_header(ADMIN_USER),
    )
    assert resp.status_code == 201, resp.text
    return resp.json()["id"]


@pytest.mark.asyncio
async def test_create_pick_starts_unpublished(client):
    resp = await client.post(
        "/api/admin/staff-picks",
        json={"title": "May 2026 Picks"},
        headers=make_auth_header(ADMIN_USER),
    )
    assert resp.status_code == 201
    data = resp.json()
    assert data["title"] == "May 2026 Picks"
    assert data["is_published"] is False
    assert data["created_by"] == ADMIN_USER
    assert data["item_count"] == 0


@pytest.mark.asyncio
async def test_create_pick_requires_admin(client):
    resp = await client.post(
        "/api/admin/staff-picks",
        json={"title": "Sneaky Pick"},
        headers=make_auth_header("randomuser"),
    )
    assert resp.status_code == 403


@pytest.mark.asyncio
async def test_update_pick_publishes_and_changes_title(client):
    pick_id = await _create_pick(client, title="Draft Pick")
    resp = await client.put(
        f"/api/admin/staff-picks/{pick_id}",
        json={"title": "Published Pick", "is_published": True},
        headers=make_auth_header(ADMIN_USER),
    )
    assert resp.status_code == 200
    data = resp.json()
    assert data["title"] == "Published Pick"
    assert data["is_published"] is True


@pytest.mark.asyncio
async def test_add_item_to_pick(client):
    pick_id = await _create_pick(client)
    project_id = await _create_project(client)
    resp = await client.post(
        f"/api/admin/staff-picks/{pick_id}/items",
        json={"project_id": project_id, "editor_note": "Outstanding cable mgmt"},
        headers=make_auth_header(ADMIN_USER),
    )
    assert resp.status_code == 201, resp.text
    data = resp.json()
    assert data["project_id"] == project_id
    assert data["editor_note"] == "Outstanding cable mgmt"
    assert data["order_index"] == 0
    assert data["project"]["id"] == project_id


@pytest.mark.asyncio
async def test_add_duplicate_item_rejected(client):
    pick_id = await _create_pick(client)
    project_id = await _create_project(client)
    r1 = await client.post(
        f"/api/admin/staff-picks/{pick_id}/items",
        json={"project_id": project_id},
        headers=make_auth_header(ADMIN_USER),
    )
    assert r1.status_code == 201
    r2 = await client.post(
        f"/api/admin/staff-picks/{pick_id}/items",
        json={"project_id": project_id},
        headers=make_auth_header(ADMIN_USER),
    )
    assert r2.status_code == 409


@pytest.mark.asyncio
async def test_reorder_item_via_patch(client):
    pick_id = await _create_pick(client)
    p1 = await _create_project(client, title="First Project")
    p2 = await _create_project(client, title="Second Project")

    item1 = (await client.post(
        f"/api/admin/staff-picks/{pick_id}/items",
        json={"project_id": p1},
        headers=make_auth_header(ADMIN_USER),
    )).json()
    item2 = (await client.post(
        f"/api/admin/staff-picks/{pick_id}/items",
        json={"project_id": p2},
        headers=make_auth_header(ADMIN_USER),
    )).json()
    assert item1["order_index"] == 0
    assert item2["order_index"] == 1

    # Move item2 to the front.
    r = await client.patch(
        f"/api/admin/staff-picks/{pick_id}/items/{item2['id']}",
        json={"order_index": -1, "editor_note": "Bumped to top"},
        headers=make_auth_header(ADMIN_USER),
    )
    assert r.status_code == 422  # ge=0 rejects negatives — try again with 0
    r = await client.patch(
        f"/api/admin/staff-picks/{pick_id}/items/{item2['id']}",
        json={"order_index": 0, "editor_note": "Bumped to top"},
        headers=make_auth_header(ADMIN_USER),
    )
    assert r.status_code == 200
    assert r.json()["order_index"] == 0
    assert r.json()["editor_note"] == "Bumped to top"


@pytest.mark.asyncio
async def test_delete_item(client):
    pick_id = await _create_pick(client)
    project_id = await _create_project(client)
    item = (await client.post(
        f"/api/admin/staff-picks/{pick_id}/items",
        json={"project_id": project_id},
        headers=make_auth_header(ADMIN_USER),
    )).json()
    r = await client.delete(
        f"/api/admin/staff-picks/{pick_id}/items/{item['id']}",
        headers=make_auth_header(ADMIN_USER),
    )
    assert r.status_code == 204


@pytest.mark.asyncio
async def test_list_picks_public_only_shows_published(client):
    draft_id = await _create_pick(client, title="Draft", is_published=False)
    pub_id = await _create_pick(client, title="Published", is_published=True)

    # Anonymous public call should see only the published one.
    resp = await client.get("/api/staff-picks")
    assert resp.status_code == 200
    ids = [p["id"] for p in resp.json()]
    assert pub_id in ids
    assert draft_id not in ids


@pytest.mark.asyncio
async def test_list_picks_admin_sees_drafts(client):
    draft_id = await _create_pick(client, title="Draft", is_published=False)
    pub_id = await _create_pick(client, title="Published", is_published=True)

    # Admin asking for unpublished sees the draft only.
    resp = await client.get(
        "/api/staff-picks?published=false",
        headers=make_auth_header(ADMIN_USER),
    )
    assert resp.status_code == 200
    ids = [p["id"] for p in resp.json()]
    assert draft_id in ids
    assert pub_id not in ids


@pytest.mark.asyncio
async def test_non_admin_asking_for_drafts_gets_empty_list(client):
    await _create_pick(client, title="Draft", is_published=False)
    # Anonymous + published=false → empty (silent), not 403.
    resp = await client.get("/api/staff-picks?published=false")
    assert resp.status_code == 200
    assert resp.json() == []


@pytest.mark.asyncio
async def test_get_unpublished_pick_hidden_from_public(client):
    pick_id = await _create_pick(client, is_published=False)
    # Anonymous should 404 (not 403) so we don't leak draft existence.
    resp = await client.get(f"/api/staff-picks/{pick_id}")
    assert resp.status_code == 404


@pytest.mark.asyncio
async def test_get_unpublished_pick_visible_to_admin(client):
    pick_id = await _create_pick(client, is_published=False)
    resp = await client.get(
        f"/api/staff-picks/{pick_id}",
        headers=make_auth_header(ADMIN_USER),
    )
    assert resp.status_code == 200
    assert resp.json()["id"] == pick_id


@pytest.mark.asyncio
async def test_get_pick_returns_items_in_order(client):
    pick_id = await _create_pick(client, is_published=True)
    p1 = await _create_project(client, title="Alpha Build")
    p2 = await _create_project(client, title="Beta Build")
    p3 = await _create_project(client, title="Gamma Build")

    for pid in (p1, p2, p3):
        await client.post(
            f"/api/admin/staff-picks/{pick_id}/items",
            json={"project_id": pid},
            headers=make_auth_header(ADMIN_USER),
        )

    resp = await client.get(f"/api/staff-picks/{pick_id}")
    assert resp.status_code == 200
    data = resp.json()
    assert data["item_count"] == 3
    assert [it["project_id"] for it in data["items"]] == [p1, p2, p3]
    assert data["items"][0]["project"]["title"] == "Alpha Build"


@pytest.mark.asyncio
async def test_admin_endpoints_reject_non_admin(client):
    project_id = await _create_project(client)
    pick_id = await _create_pick(client)

    # Each admin route should 403 for randoms.
    r = await client.put(
        f"/api/admin/staff-picks/{pick_id}",
        json={"title": "Hijack"},
        headers=make_auth_header("rando"),
    )
    assert r.status_code == 403

    r = await client.post(
        f"/api/admin/staff-picks/{pick_id}/items",
        json={"project_id": project_id},
        headers=make_auth_header("rando"),
    )
    assert r.status_code == 403

    r = await client.delete(
        f"/api/admin/staff-picks/{pick_id}",
        headers=make_auth_header("rando"),
    )
    assert r.status_code == 403


@pytest.mark.asyncio
async def test_delete_pick_cascades_items(client):
    pick_id = await _create_pick(client)
    project_id = await _create_project(client)
    await client.post(
        f"/api/admin/staff-picks/{pick_id}/items",
        json={"project_id": project_id},
        headers=make_auth_header(ADMIN_USER),
    )
    r = await client.delete(
        f"/api/admin/staff-picks/{pick_id}",
        headers=make_auth_header(ADMIN_USER),
    )
    assert r.status_code == 204
    # And the pick is gone.
    r = await client.get(
        f"/api/staff-picks/{pick_id}",
        headers=make_auth_header(ADMIN_USER),
    )
    assert r.status_code == 404


@pytest.mark.asyncio
async def test_editor_note_length_capped(client):
    pick_id = await _create_pick(client)
    project_id = await _create_project(client)
    resp = await client.post(
        f"/api/admin/staff-picks/{pick_id}/items",
        json={"project_id": project_id, "editor_note": "x" * 301},
        headers=make_auth_header(ADMIN_USER),
    )
    assert resp.status_code == 422
