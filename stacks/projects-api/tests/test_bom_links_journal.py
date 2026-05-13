"""BOM, links, and journal tests."""

from __future__ import annotations

import pytest
from .conftest import make_auth_header


@pytest.fixture
async def project_id(client) -> int:
    headers = make_auth_header()
    resp = await client.post(
        "/api/projects",
        json={"title": "BOM Test Project"},
        headers=headers,
    )
    return resp.json()["id"]


@pytest.mark.asyncio
async def test_bom_crud(client, project_id) -> None:
    headers = make_auth_header()
    # Add
    resp = await client.post(
        f"/api/projects/{project_id}/bom",
        json={"name": "Servo Motor", "quantity": 4, "unit_cost": 8.50},
        headers=headers,
    )
    assert resp.status_code == 201
    item_id = resp.json()["id"]
    # List
    resp = await client.get(f"/api/projects/{project_id}/bom")
    assert len(resp.json()) == 1
    assert resp.json()[0]["name"] == "Servo Motor"
    # Update
    resp = await client.put(
        f"/api/projects/{project_id}/bom/{item_id}",
        json={"name": "STS3215 Servo", "quantity": 6, "unit_cost": 9.00},
        headers=headers,
    )
    assert resp.json()["name"] == "STS3215 Servo"
    # Delete
    resp = await client.delete(f"/api/projects/{project_id}/bom/{item_id}", headers=headers)
    assert resp.status_code == 204


@pytest.mark.asyncio
async def test_links_crud(client, project_id) -> None:
    headers = make_auth_header()
    resp = await client.post(
        f"/api/projects/{project_id}/links",
        json={"title": "Servo Tutorial", "url": "https://example.com", "link_type": "tutorial"},
        headers=headers,
    )
    assert resp.status_code == 201
    link_id = resp.json()["id"]
    resp = await client.get(f"/api/projects/{project_id}/links")
    assert len(resp.json()) == 1
    resp = await client.delete(f"/api/projects/{project_id}/links/{link_id}", headers=headers)
    assert resp.status_code == 204


@pytest.mark.asyncio
async def test_journal_crud(client, project_id) -> None:
    headers = make_auth_header()
    resp = await client.post(
        f"/api/projects/{project_id}/journal",
        json={"title": "Day 1: Wiring", "content_md": "Connected the servos.", "status": "in_progress"},
        headers=headers,
    )
    assert resp.status_code == 201
    entry_id = resp.json()["id"]
    resp = await client.get(f"/api/projects/{project_id}/journal")
    assert len(resp.json()) == 1
    resp = await client.put(
        f"/api/projects/{project_id}/journal/{entry_id}",
        json={"title": "Day 1: Wiring complete", "content_md": "All servos connected.", "status": "completed"},
        headers=headers,
    )
    assert resp.json()["status"] == "completed"
    resp = await client.delete(f"/api/projects/{project_id}/journal/{entry_id}", headers=headers)
    assert resp.status_code == 204
