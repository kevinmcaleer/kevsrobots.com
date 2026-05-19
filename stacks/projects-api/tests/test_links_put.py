"""Tests for the new ``PUT /api/projects/{id}/links/{link_id}`` route
added by issue #171 (inline-edit autosave for Related Links)."""

from __future__ import annotations

import pytest
from .conftest import make_auth_header


@pytest.fixture
async def project_id(client) -> int:
    headers = make_auth_header()
    resp = await client.post(
        "/api/projects",
        json={"title": "Links PUT Test Project"},
        headers=headers,
    )
    return resp.json()["id"]


@pytest.fixture
async def link_id(client, project_id: int) -> int:
    headers = make_auth_header()
    resp = await client.post(
        f"/api/projects/{project_id}/links",
        json={
            "title": "Original Title",
            "url": "https://example.com/before",
            "link_type": "article",
        },
        headers=headers,
    )
    assert resp.status_code == 201
    return resp.json()["id"]


@pytest.mark.asyncio
async def test_link_put_updates_all_fields(client, project_id, link_id) -> None:
    headers = make_auth_header()
    resp = await client.put(
        f"/api/projects/{project_id}/links/{link_id}",
        json={
            "title": "New Title",
            "url": "https://example.com/after",
            "link_type": "tutorial",
        },
        headers=headers,
    )
    assert resp.status_code == 200
    body = resp.json()
    assert body["title"] == "New Title"
    assert body["url"] == "https://example.com/after"
    assert body["link_type"] == "tutorial"


@pytest.mark.asyncio
async def test_link_put_partial_update_title_only(
    client, project_id, link_id
) -> None:
    """Sending only the title leaves url + link_type untouched."""
    headers = make_auth_header()
    resp = await client.put(
        f"/api/projects/{project_id}/links/{link_id}",
        json={"title": "Just retitled"},
        headers=headers,
    )
    assert resp.status_code == 200
    body = resp.json()
    assert body["title"] == "Just retitled"
    # url + link_type kept their original values.
    assert body["url"] == "https://example.com/before"
    assert body["link_type"] == "article"


@pytest.mark.asyncio
async def test_link_put_partial_update_url_only(
    client, project_id, link_id
) -> None:
    headers = make_auth_header()
    resp = await client.put(
        f"/api/projects/{project_id}/links/{link_id}",
        json={"url": "https://example.com/url-only"},
        headers=headers,
    )
    assert resp.status_code == 200
    body = resp.json()
    assert body["url"] == "https://example.com/url-only"
    assert body["title"] == "Original Title"
    assert body["link_type"] == "article"


@pytest.mark.asyncio
async def test_link_put_partial_update_type_only(
    client, project_id, link_id
) -> None:
    headers = make_auth_header()
    resp = await client.put(
        f"/api/projects/{project_id}/links/{link_id}",
        json={"link_type": "video"},
        headers=headers,
    )
    assert resp.status_code == 200
    assert resp.json()["link_type"] == "video"


@pytest.mark.asyncio
async def test_link_put_rejects_bad_type(client, project_id, link_id) -> None:
    headers = make_auth_header()
    resp = await client.put(
        f"/api/projects/{project_id}/links/{link_id}",
        json={"link_type": "not-a-real-type"},
        headers=headers,
    )
    assert resp.status_code == 422


@pytest.mark.asyncio
async def test_link_put_owner_only(client, project_id, link_id) -> None:
    other = make_auth_header(username="someoneelse")
    resp = await client.put(
        f"/api/projects/{project_id}/links/{link_id}",
        json={"title": "hijacked"},
        headers=other,
    )
    assert resp.status_code == 403


@pytest.mark.asyncio
async def test_link_put_404_when_link_missing(client, project_id) -> None:
    headers = make_auth_header()
    resp = await client.put(
        f"/api/projects/{project_id}/links/99999",
        json={"title": "ghost"},
        headers=headers,
    )
    assert resp.status_code == 404


@pytest.mark.asyncio
async def test_link_put_survives_refresh(client, project_id, link_id) -> None:
    """Round-trip: PUT then re-GET the list and confirm the edit landed."""
    headers = make_auth_header()
    await client.put(
        f"/api/projects/{project_id}/links/{link_id}",
        json={"title": "Persisted", "link_type": "documentation"},
        headers=headers,
    )
    resp = await client.get(f"/api/projects/{project_id}/links")
    assert resp.status_code == 200
    rows = [r for r in resp.json() if r["id"] == link_id]
    assert len(rows) == 1
    assert rows[0]["title"] == "Persisted"
    assert rows[0]["link_type"] == "documentation"
