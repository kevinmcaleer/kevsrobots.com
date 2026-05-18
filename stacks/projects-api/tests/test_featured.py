"""Tests for the featured-projects endpoints (issue #115)."""

from __future__ import annotations

import pytest

from .conftest import make_auth_header


# Match the default in projects_api.config.Settings.admin_usernames.
ADMIN_USER = "kev"


async def _create_project(client, *, owner: str = "maker", title: str = "A Build To Feature") -> int:
    resp = await client.post(
        "/api/projects",
        json={"title": title, "short_description": "desc"},
        headers=make_auth_header(owner),
    )
    assert resp.status_code == 201, resp.text
    return resp.json()["id"]


@pytest.mark.asyncio
async def test_feature_endpoint_sets_all_four_columns(client):
    project_id = await _create_project(client)
    resp = await client.post(
        f"/api/admin/projects/{project_id}/feature",
        json={"note": "Lovely cable management"},
        headers=make_auth_header(ADMIN_USER),
    )
    assert resp.status_code == 200, resp.text
    data = resp.json()
    assert data["is_featured"] is True
    assert data["featured_note"] == "Lovely cable management"
    assert data["featured_by"] == ADMIN_USER
    assert data["featured_at"] is not None


@pytest.mark.asyncio
async def test_feature_endpoint_is_idempotent(client):
    project_id = await _create_project(client)
    # Feature once with note A
    r1 = await client.post(
        f"/api/admin/projects/{project_id}/feature",
        json={"note": "First pick"},
        headers=make_auth_header(ADMIN_USER),
    )
    assert r1.status_code == 200
    # Feature again with note B — should overwrite, not 5xx.
    r2 = await client.post(
        f"/api/admin/projects/{project_id}/feature",
        json={"note": "Updated reason"},
        headers=make_auth_header(ADMIN_USER),
    )
    assert r2.status_code == 200
    assert r2.json()["featured_note"] == "Updated reason"


@pytest.mark.asyncio
async def test_unfeature_clears_all_four_columns(client):
    project_id = await _create_project(client)
    await client.post(
        f"/api/admin/projects/{project_id}/feature",
        json={"note": "x"},
        headers=make_auth_header(ADMIN_USER),
    )
    resp = await client.delete(
        f"/api/admin/projects/{project_id}/feature",
        headers=make_auth_header(ADMIN_USER),
    )
    assert resp.status_code == 200
    data = resp.json()
    assert data["is_featured"] is False
    assert data["featured_note"] is None
    assert data["featured_at"] is None
    assert data["featured_by"] is None


@pytest.mark.asyncio
async def test_unfeature_is_idempotent(client):
    project_id = await _create_project(client)
    # Unfeature a project that was never featured — must not 500.
    resp = await client.delete(
        f"/api/admin/projects/{project_id}/feature",
        headers=make_auth_header(ADMIN_USER),
    )
    assert resp.status_code == 200
    assert resp.json()["is_featured"] is False


@pytest.mark.asyncio
async def test_feature_requires_admin(client):
    project_id = await _create_project(client)
    resp = await client.post(
        f"/api/admin/projects/{project_id}/feature",
        json={"note": "x"},
        headers=make_auth_header("randomuser"),
    )
    assert resp.status_code == 403


@pytest.mark.asyncio
async def test_feature_requires_auth(client):
    project_id = await _create_project(client)
    resp = await client.post(
        f"/api/admin/projects/{project_id}/feature",
        json={"note": "x"},
    )
    assert resp.status_code == 401


@pytest.mark.asyncio
async def test_feature_unknown_project_404(client):
    resp = await client.post(
        "/api/admin/projects/99999/feature",
        json={"note": "x"},
        headers=make_auth_header(ADMIN_USER),
    )
    assert resp.status_code == 404


@pytest.mark.asyncio
async def test_feature_note_length_validated(client):
    project_id = await _create_project(client)
    # 201 chars — pydantic should reject before SQLAlchemy does.
    resp = await client.post(
        f"/api/admin/projects/{project_id}/feature",
        json={"note": "x" * 201},
        headers=make_auth_header(ADMIN_USER),
    )
    assert resp.status_code == 422


@pytest.mark.asyncio
async def test_list_featured_returns_only_featured(client):
    p_feat = await _create_project(client, title="Featured One")
    p_other = await _create_project(client, title="Plain Build")  # noqa: F841

    await client.post(
        f"/api/admin/projects/{p_feat}/feature",
        json={"note": "Pick of the week"},
        headers=make_auth_header(ADMIN_USER),
    )

    resp = await client.get("/api/projects/featured")
    assert resp.status_code == 200
    data = resp.json()
    ids = [p["id"] for p in data]
    assert p_feat in ids
    assert p_other not in ids
    # featured_note carried through to the public listing
    assert any(p["featured_note"] == "Pick of the week" for p in data)


@pytest.mark.asyncio
async def test_list_featured_excludes_archived_and_blocked(client):
    """Featured + archived → hidden. Featured + blocked → hidden."""
    # Set up two featured projects, then archive one and block the other.
    p_archived = await _create_project(client, owner="a1", title="Archived Featured")
    p_blocked = await _create_project(client, owner="b1", title="Blocked Featured")
    p_visible = await _create_project(client, owner="v1", title="Plain Featured")

    for pid in (p_archived, p_blocked, p_visible):
        r = await client.post(
            f"/api/admin/projects/{pid}/feature",
            json={"note": "x"},
            headers=make_auth_header(ADMIN_USER),
        )
        assert r.status_code == 200, r.text

    # Archive the first via the owner's PUT.
    r = await client.put(
        f"/api/projects/{p_archived}",
        json={"status": "archived"},
        headers=make_auth_header("a1"),
    )
    assert r.status_code == 200, r.text

    # Block the second via the admin route.
    r = await client.put(
        f"/api/admin/projects/{p_blocked}/block",
        headers=make_auth_header(ADMIN_USER),
    )
    assert r.status_code == 200, r.text

    resp = await client.get("/api/projects/featured")
    assert resp.status_code == 200
    ids = [p["id"] for p in resp.json()]
    assert p_visible in ids
    assert p_archived not in ids
    assert p_blocked not in ids


@pytest.mark.asyncio
async def test_list_featured_orders_newest_feature_first(client):
    p1 = await _create_project(client, title="Older Pick")
    p2 = await _create_project(client, title="Newer Pick")

    await client.post(
        f"/api/admin/projects/{p1}/feature",
        json={"note": "a"},
        headers=make_auth_header(ADMIN_USER),
    )
    await client.post(
        f"/api/admin/projects/{p2}/feature",
        json={"note": "b"},
        headers=make_auth_header(ADMIN_USER),
    )

    resp = await client.get("/api/projects/featured")
    ids = [p["id"] for p in resp.json()]
    # p2 was featured most recently — it should come first.
    assert ids.index(p2) < ids.index(p1)


@pytest.mark.asyncio
async def test_featured_flag_surfaces_on_listing_and_detail(client):
    pid = await _create_project(client, title="Surface Test")
    await client.post(
        f"/api/admin/projects/{pid}/feature",
        json={"note": "Surface"},
        headers=make_auth_header(ADMIN_USER),
    )

    # Detail
    r = await client.get(f"/api/projects/{pid}")
    assert r.status_code == 200
    body = r.json()
    assert body["is_featured"] is True
    assert body["featured_note"] == "Surface"

    # Listing
    r = await client.get("/api/projects")
    assert r.status_code == 200
    items = {p["id"]: p for p in r.json()}
    assert items[pid]["is_featured"] is True
    assert items[pid]["featured_note"] == "Surface"
