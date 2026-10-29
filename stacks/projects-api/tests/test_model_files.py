"""3D models in project files: categories, print settings and
browser-rendered thumbnails (issues #261 and #262)."""

from __future__ import annotations

import pytest
from sqlalchemy import select

from projects_api.models import Download, ProjectFile
from .conftest import make_auth_header

PNG = b"\x89PNG\r\n\x1a\n" + b"\x00" * 32
STL = b"solid cube\nendsolid cube\n"


@pytest.fixture
async def project_id(client) -> int:
    resp = await client.post(
        "/api/projects", json={"title": "Model Test Project", "status": "wip"},
        headers=make_auth_header(),
    )
    return resp.json()["id"]


async def _upload(client, project_id, name, content=b"x") -> dict:
    resp = await client.post(
        f"/api/projects/{project_id}/files",
        files={"file": (name, content, "application/octet-stream")},
        headers=make_auth_header(),
    )
    assert resp.status_code == 201, resp.text
    return resp.json()


@pytest.mark.asyncio
@pytest.mark.parametrize("name,category", [
    ("bracket.stl", "model"), ("case.3mf", "model"), ("part.STEP", "model"),
    ("print.gcode", "model"), ("main.py", "code"), ("sketch.ino", "code"),
    ("notes.md", "document"), ("plate.dxf", "document"), ("config.yaml", "document"),
    ("source.zip", "archive"),
])
async def test_upload_sets_category(client, project_id, name, category) -> None:
    assert (await _upload(client, project_id, name))["file_type"] == category


@pytest.mark.asyncio
async def test_legacy_extension_rows_read_as_categories(client, session, project_id) -> None:
    session.add(ProjectFile(
        project_id=project_id, filename="old.stl", file_path="local:x",
        file_size=1, file_type="stl",
    ))
    await session.commit()
    files = (await client.get(f"/api/projects/{project_id}/files")).json()
    assert files[0]["file_type"] == "model"
    assert files[0]["thumbnail_version"] is None


@pytest.mark.asyncio
async def test_print_settings_round_trip(client, project_id) -> None:
    f = await _upload(client, project_id, "bracket.stl", STL)
    resp = await client.put(
        f"/api/projects/{project_id}/files/{f['id']}",
        json={"print_settings": "0.2 mm layers, 20% infill"},
        headers=make_auth_header(),
    )
    assert resp.status_code == 200
    assert resp.json()["print_settings"] == "0.2 mm layers, 20% infill"
    assert resp.json()["description"] is None


@pytest.mark.asyncio
async def test_thumbnail_upload_and_serve(client, project_id) -> None:
    f = await _upload(client, project_id, "bracket.stl", STL)
    resp = await client.post(
        f"/api/projects/{project_id}/files/{f['id']}/thumbnail",
        files={"file": ("thumb.png", PNG, "image/png")},
        headers=make_auth_header(),
    )
    assert resp.status_code == 200, resp.text
    version = resp.json()["thumbnail_version"]
    assert version

    resp = await client.get(f"/api/projects/{project_id}/files/{f['id']}/thumbnail?v={version}")
    assert resp.status_code == 200
    assert resp.content == PNG
    assert resp.headers["content-type"] == "image/png"
    assert resp.headers["cache-control"] == "public, max-age=86400"
    assert resp.headers["etag"] == f'"{version}"'

    # A new thumbnail is a new version.
    resp = await client.post(
        f"/api/projects/{project_id}/files/{f['id']}/thumbnail",
        files={"file": ("thumb.png", PNG + b"1", "image/png")},
        headers=make_auth_header(),
    )
    assert resp.json()["thumbnail_version"] != version


@pytest.mark.asyncio
async def test_thumbnail_rules(client, project_id) -> None:
    model = await _upload(client, project_id, "bracket.stl", STL)
    code = await _upload(client, project_id, "main.py")
    url = f"/api/projects/{project_id}/files/{{}}/thumbnail"
    png = {"file": ("thumb.png", PNG, "image/png")}

    assert (await client.post(url.format(code["id"]), files=png,
                              headers=make_auth_header())).status_code == 400
    assert (await client.post(url.format(model["id"]),
                              files={"file": ("t.png", b"GIF89a", "image/png")},
                              headers=make_auth_header())).status_code == 400
    assert (await client.post(url.format(model["id"]), files=png,
                              headers=make_auth_header("someone_else"))).status_code == 403
    assert (await client.post(url.format(model["id"]), files=png)).status_code == 401
    # No thumbnail yet.
    assert (await client.get(url.format(model["id"]))).status_code == 404


@pytest.mark.asyncio
async def test_draft_thumbnail_is_private(client) -> None:
    pid = (await client.post(
        "/api/projects", json={"title": "Draft Models", "status": "draft"},
        headers=make_auth_header(),
    )).json()["id"]
    f = await _upload(client, pid, "bracket.stl", STL)
    await client.post(
        f"/api/projects/{pid}/files/{f['id']}/thumbnail",
        files={"file": ("thumb.png", PNG, "image/png")}, headers=make_auth_header(),
    )
    url = f"/api/projects/{pid}/files/{f['id']}/thumbnail"
    assert (await client.get(url)).status_code == 404
    resp = await client.get(url, headers=make_auth_header())
    assert resp.status_code == 200
    assert resp.headers["cache-control"] == "private, no-store"


@pytest.mark.asyncio
async def test_preview_download_is_not_counted(client, session, project_id) -> None:
    f = await _upload(client, project_id, "bracket.stl", STL)
    url = f"/api/projects/{project_id}/files/{f['id']}/download"
    assert (await client.get(url + "?preview=1")).content == STL
    assert (await session.scalars(select(Download))).all() == []
    await client.get(url)
    assert len((await session.scalars(select(Download))).all()) == 1


@pytest.mark.asyncio
async def test_admin_backfill(client, project_id, monkeypatch) -> None:
    monkeypatch.setenv("ADMIN_USERNAMES", "admin")
    admin = make_auth_header("admin")
    model = await _upload(client, project_id, "bracket.stl", STL)
    await _upload(client, project_id, "main.py")

    assert (await client.get("/api/admin/files/missing-thumbnails",
                             headers=make_auth_header())).status_code == 403
    missing = (await client.get("/api/admin/files/missing-thumbnails", headers=admin)).json()
    assert [m["file_id"] for m in missing] == [model["id"]]
    assert missing[0]["project_title"] == "Model Test Project"

    resp = await client.get(f"/api/admin/files/{model['id']}/model", headers=admin)
    assert resp.content == STL
    resp = await client.post(
        f"/api/admin/files/{model['id']}/thumbnail",
        files={"file": ("thumb.png", PNG, "image/png")}, headers=admin,
    )
    assert resp.status_code == 200
    assert resp.json()["thumbnail_version"]
    assert (await client.get("/api/admin/files/missing-thumbnails", headers=admin)).json() == []


@pytest.mark.asyncio
async def test_delete_removes_thumbnail(client, project_id) -> None:
    f = await _upload(client, project_id, "bracket.stl", STL)
    await client.post(
        f"/api/projects/{project_id}/files/{f['id']}/thumbnail",
        files={"file": ("thumb.png", PNG, "image/png")}, headers=make_auth_header(),
    )
    resp = await client.delete(f"/api/projects/{project_id}/files/{f['id']}",
                               headers=make_auth_header())
    assert resp.status_code == 204
    assert (await client.get(f"/api/projects/{project_id}/files")).json() == []
