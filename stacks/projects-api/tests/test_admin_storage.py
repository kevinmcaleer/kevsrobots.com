"""The admin storage page's routes: bytes per project against the limit."""

from __future__ import annotations

from datetime import datetime

import pytest

from projects_api.models import ProjectFile, ProjectImage, ProjectVersion
from .conftest import make_auth_header


@pytest.fixture
def admin(monkeypatch) -> dict:
    monkeypatch.setenv("ADMIN_USERNAMES", "admin")
    return make_auth_header("admin")


async def _project(client, title: str) -> int:
    resp = await client.post(
        "/api/projects", json={"title": title, "status": "draft"}, headers=make_auth_header(),
    )
    assert resp.status_code in (200, 201), resp.text
    return resp.json()["id"]


@pytest.fixture
async def rover(client, session) -> dict:
    pid = await _project(client, "Rover")
    live = ProjectFile(project_id=pid, filename="rover.model.json", file_path="local:a",
                       file_size=3000, file_type="other")
    old = ProjectFile(project_id=pid, filename="rover.model.json", file_path="local:b",
                      file_size=5000, file_type="other", removed_at=datetime(2026, 11, 1))
    session.add_all([live, old, ProjectImage(project_id=pid, filename="a.png", file_path="local:c")])
    await session.flush()
    session.add(ProjectVersion(project_id=pid, number=1, kind="published",
                               snapshot={"files": [{"id": old.id}]}))
    await session.commit()
    return {"id": pid, "live": live.id, "old": old.id}


@pytest.mark.asyncio
async def test_routes_are_admin_only(client, rover, admin) -> None:
    assert (await client.get("/api/admin/storage/projects", headers=make_auth_header())).status_code == 403
    url = f"/api/admin/storage/projects/{rover['id']}/files"
    assert (await client.get(url, headers=make_auth_header())).status_code == 403


@pytest.mark.asyncio
async def test_lists_bytes_split_by_removed_and_biggest_first(client, rover, admin) -> None:
    empty = await _project(client, "Empty")
    body = (await client.get("/api/admin/storage/projects", headers=admin)).json()
    assert body["limit_bytes"] == 50 * 1024 * 1024
    first, second = body["projects"]
    assert first["project_id"] == rover["id"]
    assert (first["file_count"], first["file_bytes"]) == (1, 3000)
    assert (first["removed_count"], first["removed_bytes"]) == (1, 5000)
    assert first["counted_bytes"] == 8000
    assert (first["image_count"], first["version_count"]) == (1, 1)
    assert (first["largest_filename"], first["largest_bytes"]) == ("rover.model.json", 5000)
    assert second["project_id"] == empty
    assert second["counted_bytes"] == 0 and second["largest_filename"] is None


@pytest.mark.asyncio
async def test_files_include_removed_ones_and_the_versions_using_them(client, rover, admin) -> None:
    files = (await client.get(f"/api/admin/storage/projects/{rover['id']}/files", headers=admin)).json()
    assert [f["id"] for f in files] == [rover["old"], rover["live"]]
    assert files[0]["removed_at"] is not None and files[0]["versions"] == [1]
    assert files[1]["removed_at"] is None and files[1]["versions"] == []
    assert (await client.get("/api/admin/storage/projects/999/files", headers=admin)).status_code == 404
