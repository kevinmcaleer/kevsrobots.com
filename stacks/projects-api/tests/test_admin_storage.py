"""Admin storage page: each project's size, sortable (/admin/projects/storage/)."""

from __future__ import annotations

import os
from datetime import datetime

import pytest
from sqlalchemy import update

from .conftest import make_auth_header


@pytest.mark.asyncio
async def test_project_storage_sizes(client, monkeypatch) -> None:
    from projects_api.db import get_sessionmaker
    from projects_api.models import ProjectFile, ProjectVersion

    monkeypatch.setenv("ADMIN_USERNAMES", "admin")
    admin = make_auth_header("admin")
    headers = make_auth_header()
    big = (await client.post("/api/projects", json={"title": "Big project"}, headers=headers)).json()["id"]
    small = (await client.post("/api/projects", json={"title": "Small project"}, headers=headers)).json()["id"]
    up = lambda pid, name, n: client.post(
        f"/api/projects/{pid}/files", files={"file": (name, os.urandom(n), "text/plain")}, headers=headers
    )
    await up(big, "robot.model.json", 300)
    old = (await up(big, "old.py", 100)).json()["id"]
    await up(small, "a.py", 10)
    async with get_sessionmaker()() as s:
        await s.execute(update(ProjectFile).where(ProjectFile.id == old).values(removed_at=datetime(2026, 1, 1)))
        s.add(ProjectVersion(project_id=big, number=1, kind="published", snapshot={"files": [{"id": old}]}))
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
    assert files[1]["removed_at"] is not None and files[1]["versions"] == [1]
    assert files[0]["versions"] == []
    assert rows[big]["versions"] == 1
    assert (await client.get("/api/admin/storage/projects/9999/files", headers=admin)).status_code == 404


STL = b"\x00" * 80 + (200).to_bytes(4, "little") + (b"\x00" * 12 + b"\x01\x02\x03\x04" * 9 + b"\x00\x00") * 200


@pytest.mark.asyncio
async def test_models_are_stored_gzipped_and_download_unchanged(client) -> None:
    headers = make_auth_header()
    pid = (await client.post("/api/projects", json={"title": "Gzip project"}, headers=headers)).json()["id"]
    up = await client.post(f"/api/projects/{pid}/files", files={"file": ("part.stl", STL, "model/stl")}, headers=headers)
    assert up.status_code == 201
    assert up.json()["file_size"] == len(STL)
    got = await client.get(f"/api/projects/{pid}/files/{up.json()['id']}/download", headers=headers)
    assert got.content == STL

    from projects_api.db import get_sessionmaker
    from projects_api.models import ProjectFile

    async with get_sessionmaker()() as s:
        row = await s.get(ProjectFile, up.json()["id"])
        assert row.file_path.endswith(".stored.gz")
        assert row.stored_size < len(STL) / 2


@pytest.mark.asyncio
async def test_compress_existing_files(client, monkeypatch) -> None:
    from projects_api.db import get_sessionmaker
    from projects_api.models import ProjectFile
    from projects_api.storage import save_file

    monkeypatch.setenv("ADMIN_USERNAMES", "admin")
    admin = make_auth_header("admin")
    headers = make_auth_header()
    pid = (await client.post("/api/projects", json={"title": "Old project"}, headers=headers)).json()["id"]
    path = save_file(STL, "old.stl", pid, "files")
    async with get_sessionmaker()() as s:
        row = ProjectFile(project_id=pid, filename="old.stl", file_path=path, file_size=len(STL), file_type="model")
        pdf = ProjectFile(project_id=pid, filename="a.pdf", file_path=save_file(b"%PDF", "a.pdf", pid, "files"), file_size=4, file_type="document")
        s.add_all([row, pdf])
        await s.commit()
        fid = row.id

    report = (await client.get("/api/admin/storage/projects", headers=admin)).json()
    mine = next(p for p in report["projects"] if p["project_id"] == pid)
    assert mine["compressible"] == 1 and mine["used"] == len(STL) + 4

    assert (await client.post(f"/api/admin/storage/projects/{pid}/compress", headers=headers)).status_code == 403
    res = (await client.post(f"/api/admin/storage/projects/{pid}/compress", headers=admin)).json()
    assert res["compressed"] == 1 and res["skipped"] == 1 and res["failed"] == 0 and res["saved"] > len(STL) / 2

    got = await client.get(f"/api/projects/{pid}/files/{fid}/download", headers=headers)
    assert got.content == STL
    report = (await client.get("/api/admin/storage/projects", headers=admin)).json()
    mine = next(p for p in report["projects"] if p["project_id"] == pid)
    assert mine["compressible"] == 0 and mine["used"] < len(STL) / 2
