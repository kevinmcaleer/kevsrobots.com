"""Admin storage page: each project's size, sortable (/admin/projects/storage/)."""

from __future__ import annotations

from datetime import datetime

import pytest
from sqlalchemy import update

from .conftest import make_auth_header


@pytest.mark.asyncio
async def test_project_storage_sizes(client, monkeypatch) -> None:
    from projects_api.db import get_sessionmaker
    from projects_api.models import ProjectFile

    monkeypatch.setenv("ADMIN_USERNAMES", "admin")
    admin = make_auth_header("admin")
    headers = make_auth_header()
    big = (await client.post("/api/projects", json={"title": "Big project"}, headers=headers)).json()["id"]
    small = (await client.post("/api/projects", json={"title": "Small project"}, headers=headers)).json()["id"]
    up = lambda pid, name, n: client.post(
        f"/api/projects/{pid}/files", files={"file": (name, b"x" * n, "text/plain")}, headers=headers
    )
    await up(big, "robot.model.json", 300)
    old = (await up(big, "old.py", 100)).json()["id"]
    await up(small, "a.py", 10)
    async with get_sessionmaker()() as s:
        await s.execute(update(ProjectFile).where(ProjectFile.id == old).values(removed_at=datetime(2026, 1, 1)))
        await s.commit()

    assert (await client.get("/api/admin/storage/projects", headers=headers)).status_code == 403
    body = (await client.get("/api/admin/storage/projects", headers=admin)).json()
    assert body["limit"] > 0
    rows = {p["project_id"]: p for p in body["projects"]}
    assert body["projects"][0]["project_id"] == big
    assert rows[big]["used"] == 300 and rows[big]["kept"] == 100
    assert rows[big]["files"] == 1 and rows[big]["kept_files"] == 1
    assert rows[big]["largest_file"] == "robot.model.json"
    assert rows[small]["used"] == 10

    files = (await client.get(f"/api/admin/storage/projects/{big}/files", headers=admin)).json()
    assert [f["filename"] for f in files] == ["robot.model.json", "old.py"]
    assert files[1]["removed_at"] is not None
    assert (await client.get("/api/admin/storage/projects/9999/files", headers=admin)).status_code == 404
