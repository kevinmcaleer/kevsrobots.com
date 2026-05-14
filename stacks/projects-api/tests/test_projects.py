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
    assert body["status"] == "wip"
    assert body["author_username"] == "testuser"


@pytest.mark.asyncio
async def test_create_project_unauthenticated(client) -> None:
    response = await client.post(
        "/api/projects",
        json={"title": "Should Fail Project"},
    )
    assert response.status_code == 401


@pytest.mark.asyncio
async def test_list_projects_shows_wip(client) -> None:
    headers = make_auth_header()
    await client.post(
        "/api/projects",
        json={"title": "WIP Project"},
        headers=headers,
    )
    response = await client.get("/api/projects")
    assert len(response.json()) == 1  # WIP projects are public


@pytest.mark.asyncio
async def test_archived_not_listed(client) -> None:
    headers = make_auth_header()
    create = await client.post(
        "/api/projects",
        json={"title": "Archive Me"},
        headers=headers,
    )
    pid = create.json()["id"]
    await client.put(
        f"/api/projects/{pid}",
        json={"status": "archived"},
        headers=headers,
    )
    response = await client.get("/api/projects")
    ids = [p["id"] for p in response.json()]
    assert pid not in ids


@pytest.mark.asyncio
async def test_complete_and_list(client) -> None:
    headers = make_auth_header()
    create = await client.post(
        "/api/projects",
        json={"title": "Completed Project", "tags": ["robotics", "python"]},
        headers=headers,
    )
    pid = create.json()["id"]
    await client.put(
        f"/api/projects/{pid}",
        json={"status": "completed"},
        headers=headers,
    )
    response = await client.get("/api/projects")
    items = response.json()
    assert any(p["title"] == "Completed Project" for p in items)
    completed = [p for p in items if p["id"] == pid][0]
    assert completed["status"] == "completed"
    assert "robotics" in completed["tags"]


@pytest.mark.asyncio
async def test_get_project(client) -> None:
    headers = make_auth_header()
    create = await client.post(
        "/api/projects",
        json={"title": "Detail Test Project"},
        headers=headers,
    )
    pid = create.json()["id"]
    response = await client.get(f"/api/projects/{pid}")
    assert response.status_code == 200
    assert response.json()["title"] == "Detail Test Project"


@pytest.mark.asyncio
async def test_wip_visible_to_everyone(client) -> None:
    headers = make_auth_header()
    create = await client.post(
        "/api/projects",
        json={"title": "Public WIP"},
        headers=headers,
    )
    pid = create.json()["id"]
    # Access without auth
    response = await client.get(f"/api/projects/{pid}")
    assert response.status_code == 200


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
