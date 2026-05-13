"""Project CRUD tests."""

from __future__ import annotations

import pytest
from .conftest import make_auth_header


@pytest.mark.asyncio
async def test_health(client) -> None:
    response = await client.get("/health")
    assert response.status_code == 200
    assert response.json()["status"] == "ok"


@pytest.mark.asyncio
async def test_create_project(client) -> None:
    response = await client.post(
        "/api/projects",
        json={"title": "My Test Robot", "short_description": "A cool robot"},
        headers=make_auth_header(),
    )
    assert response.status_code == 201
    body = response.json()
    assert body["title"] == "My Test Robot"
    assert body["status"] == "draft"
    assert body["author_username"] == "testuser"


@pytest.mark.asyncio
async def test_create_project_unauthenticated(client) -> None:
    response = await client.post(
        "/api/projects",
        json={"title": "Should Fail Project"},
    )
    assert response.status_code == 401


@pytest.mark.asyncio
async def test_list_projects_empty(client) -> None:
    response = await client.get("/api/projects")
    assert response.status_code == 200
    assert response.json() == []


@pytest.mark.asyncio
async def test_list_projects_published_only(client) -> None:
    headers = make_auth_header()
    await client.post(
        "/api/projects",
        json={"title": "Draft Project"},
        headers=headers,
    )
    response = await client.get("/api/projects")
    assert len(response.json()) == 0  # drafts not visible


@pytest.mark.asyncio
async def test_publish_and_list(client) -> None:
    headers = make_auth_header()
    create = await client.post(
        "/api/projects",
        json={"title": "Published Project", "tags": ["robotics", "python"]},
        headers=headers,
    )
    pid = create.json()["id"]
    await client.put(
        f"/api/projects/{pid}",
        json={"status": "published"},
        headers=headers,
    )
    response = await client.get("/api/projects")
    items = response.json()
    assert len(items) == 1
    assert items[0]["title"] == "Published Project"
    assert "robotics" in items[0]["tags"]


@pytest.mark.asyncio
async def test_get_project(client) -> None:
    headers = make_auth_header()
    create = await client.post(
        "/api/projects",
        json={"title": "Detail Test Project"},
        headers=headers,
    )
    pid = create.json()["id"]
    await client.put(f"/api/projects/{pid}", json={"status": "published"}, headers=headers)
    response = await client.get(f"/api/projects/{pid}")
    assert response.status_code == 200
    assert response.json()["title"] == "Detail Test Project"


@pytest.mark.asyncio
async def test_draft_not_visible_to_others(client) -> None:
    headers = make_auth_header()
    create = await client.post(
        "/api/projects",
        json={"title": "Secret Draft"},
        headers=headers,
    )
    pid = create.json()["id"]
    response = await client.get(f"/api/projects/{pid}")
    assert response.status_code == 404


@pytest.mark.asyncio
async def test_update_project(client) -> None:
    headers = make_auth_header()
    create = await client.post(
        "/api/projects",
        json={"title": "Original Title"},
        headers=headers,
    )
    pid = create.json()["id"]
    response = await client.put(
        f"/api/projects/{pid}",
        json={"title": "Updated Title"},
        headers=headers,
    )
    assert response.status_code == 200
    assert response.json()["title"] == "Updated Title"


@pytest.mark.asyncio
async def test_update_not_owner(client) -> None:
    owner = make_auth_header("owner")
    other = make_auth_header("other")
    create = await client.post(
        "/api/projects",
        json={"title": "Owner Project"},
        headers=owner,
    )
    pid = create.json()["id"]
    response = await client.put(
        f"/api/projects/{pid}",
        json={"title": "Hacked"},
        headers=other,
    )
    assert response.status_code == 403


@pytest.mark.asyncio
async def test_delete_project(client) -> None:
    headers = make_auth_header()
    create = await client.post(
        "/api/projects",
        json={"title": "Delete Me"},
        headers=headers,
    )
    pid = create.json()["id"]
    response = await client.delete(f"/api/projects/{pid}", headers=headers)
    assert response.status_code == 204


@pytest.mark.asyncio
async def test_my_projects(client) -> None:
    headers = make_auth_header()
    await client.post("/api/projects", json={"title": "My Project"}, headers=headers)
    response = await client.get("/api/projects/my/list", headers=headers)
    assert response.status_code == 200
    assert len(response.json()) == 1
