"""Snakie's Share sheet: a new draft from the app, and the robot model sent
as project files (routers/app_send.py)."""

from __future__ import annotations

import pytest

from .conftest import make_auth_header
from .test_wiring_send import _terms_ok, app_header  # noqa: F401  (autouse fixture)

URDF = b'<robot name="openduck"><link name="base"><visual><geometry><mesh filename="package://openduck/stl/body.stl"/></geometry></visual></link></robot>'
STL = b"solid body\nendsolid body\n"
PNG = b"\x89PNG\r\n\x1a\n" + b"\x00" * 32


async def _new(client, **extra) -> dict:
    resp = await client.post("/api/projects/app/new", json={"title": "OpenDuck mini", **extra}, headers=app_header())
    assert resp.status_code == 201, resp.text
    return resp.json()


@pytest.mark.asyncio
async def test_new_project_is_a_private_draft(client) -> None:
    body = await _new(client, code_repo_url="https://github.com/maker/openduck")
    assert body["status"] == "draft"
    assert body["editor_url"].endswith(f"editor.html?id={body['id']}")
    project = (await client.get(f"/api/projects/{body['id']}", headers=make_auth_header())).json()
    assert project["title"] == "OpenDuck mini"
    assert project["code_repo_url"] == "https://github.com/maker/openduck"
    # Nobody else can see a draft.
    assert (await client.get(f"/api/projects/{body['id']}")).status_code == 404
    mine = (await client.get("/api/projects/my/list", headers=app_header())).json()
    assert [p["id"] for p in mine] == [body["id"]]


@pytest.mark.asyncio
async def test_new_project_needs_projects_write(client) -> None:
    resp = await client.post(
        "/api/projects/app/new", json={"title": "OpenDuck mini"}, headers=app_header("profile projects:read")
    )
    assert resp.status_code == 403


@pytest.mark.asyncio
async def test_new_project_refuses_a_non_github_link(client) -> None:
    resp = await client.post(
        "/api/projects/app/new",
        json={"title": "OpenDuck mini", "code_repo_url": "https://example.com/x"},
        headers=app_header(),
    )
    assert resp.status_code == 422


def _robot_files(stl: bytes = STL):
    return [
        ("files", ("openduck.urdf", URDF, "application/xml")),
        ("files", ("body.stl", stl, "model/stl")),
    ]


@pytest.mark.asyncio
async def test_send_robot_adds_model_files_and_cover(client) -> None:
    pid = (await _new(client))["id"]
    resp = await client.put(
        f"/api/projects/{pid}/robot/send",
        files=_robot_files() + [("cover", ("cover.png", PNG, "image/png"))],
        headers=app_header(),
    )
    assert resp.status_code == 200, resp.text
    body = resp.json()
    assert sorted(f["filename"] for f in body["files"]) == ["body.stl", "openduck.urdf"]
    assert all(not f["replaced"] for f in body["files"]) and body["cover"] is True
    files = (await client.get(f"/api/projects/{pid}/files", headers=make_auth_header())).json()
    assert sorted(f["filename"] for f in files) == ["body.stl", "openduck.urdf"]
    images = (await client.get(f"/api/projects/{pid}/images", headers=make_auth_header())).json()
    assert [i["filename"] for i in images] == ["snakie-cover.png"]


@pytest.mark.asyncio
async def test_sending_again_replaces_by_name(client) -> None:
    pid = (await _new(client))["id"]
    url = f"/api/projects/{pid}/robot/send"
    first = await client.put(url, files=_robot_files() + [("cover", ("a.png", PNG, "image/png"))], headers=app_header())
    assert first.status_code == 200
    second = await client.put(
        url, files=_robot_files(b"solid body2\nendsolid body2\n") + [("cover", ("b.png", PNG, "image/png"))],
        headers=app_header(),
    )
    assert second.status_code == 200, second.text
    assert all(f["replaced"] for f in second.json()["files"])
    files = (await client.get(f"/api/projects/{pid}/files", headers=make_auth_header())).json()
    assert sorted(f["filename"] for f in files) == ["body.stl", "openduck.urdf"]
    images = (await client.get(f"/api/projects/{pid}/images", headers=make_auth_header())).json()
    assert len(images) == 1


@pytest.mark.asyncio
async def test_send_robot_refuses_code(client) -> None:
    pid = (await _new(client))["id"]
    resp = await client.put(
        f"/api/projects/{pid}/robot/send",
        files=[("files", ("main.py", b"print('hi')", "text/x-python"))],
        headers=app_header(),
    )
    assert resp.status_code == 400
    assert "main.py" in resp.json()["detail"]


@pytest.mark.asyncio
async def test_send_robot_only_to_your_own_project(client) -> None:
    pid = (await _new(client))["id"]
    resp = await client.put(
        f"/api/projects/{pid}/robot/send", files=_robot_files(), headers=app_header(username="someoneelse")
    )
    assert resp.status_code == 403


@pytest.mark.asyncio
async def test_send_robot_needs_projects_write(client) -> None:
    pid = (await _new(client))["id"]
    resp = await client.put(
        f"/api/projects/{pid}/robot/send", files=_robot_files(), headers=app_header("profile projects:read")
    )
    assert resp.status_code == 403
