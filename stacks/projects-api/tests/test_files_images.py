"""File and image upload tests."""

from __future__ import annotations

import io

import pytest
from .conftest import make_auth_header


@pytest.fixture
async def project_id(client) -> int:
    headers = make_auth_header()
    resp = await client.post(
        "/api/projects",
        json={"title": "Upload Test Project"},
        headers=headers,
    )
    return resp.json()["id"]


@pytest.mark.asyncio
async def test_upload_file(client, project_id) -> None:
    headers = make_auth_header()
    resp = await client.post(
        f"/api/projects/{project_id}/files",
        files={"file": ("code.py", b"print('hello')", "text/plain")},
        headers=headers,
    )
    assert resp.status_code == 201
    body = resp.json()
    assert body["filename"] == "code.py"
    assert body["file_type"] == "py"
    assert body["file_size"] == len(b"print('hello')")


@pytest.mark.asyncio
async def test_upload_file_unauthenticated(client, project_id) -> None:
    resp = await client.post(
        f"/api/projects/{project_id}/files",
        files={"file": ("code.py", b"print('hello')", "text/plain")},
    )
    assert resp.status_code == 401


@pytest.mark.asyncio
async def test_upload_file_not_owner(client, project_id) -> None:
    headers = make_auth_header("other_user")
    resp = await client.post(
        f"/api/projects/{project_id}/files",
        files={"file": ("code.py", b"print('hello')", "text/plain")},
        headers=headers,
    )
    assert resp.status_code == 403


@pytest.mark.asyncio
async def test_upload_disallowed_extension(client, project_id) -> None:
    headers = make_auth_header()
    resp = await client.post(
        f"/api/projects/{project_id}/files",
        files={"file": ("virus.exe", b"bad stuff", "application/octet-stream")},
        headers=headers,
    )
    assert resp.status_code == 400
    assert "not allowed" in resp.json()["detail"]


@pytest.mark.asyncio
async def test_list_files(client, project_id) -> None:
    headers = make_auth_header()
    await client.post(
        f"/api/projects/{project_id}/files",
        files={"file": ("test.py", b"x = 1", "text/plain")},
        headers=headers,
    )
    resp = await client.get(f"/api/projects/{project_id}/files")
    assert resp.status_code == 200
    assert len(resp.json()) == 1


@pytest.mark.asyncio
async def test_download_file(client, project_id) -> None:
    headers = make_auth_header()
    upload = await client.post(
        f"/api/projects/{project_id}/files",
        files={"file": ("hello.py", b"print('hi')", "text/plain")},
        headers=headers,
    )
    file_id = upload.json()["id"]
    resp = await client.get(f"/api/projects/{project_id}/files/{file_id}/download")
    assert resp.status_code == 200
    assert resp.content == b"print('hi')"


@pytest.mark.asyncio
async def test_delete_file(client, project_id) -> None:
    headers = make_auth_header()
    upload = await client.post(
        f"/api/projects/{project_id}/files",
        files={"file": ("del.py", b"pass", "text/plain")},
        headers=headers,
    )
    file_id = upload.json()["id"]
    resp = await client.delete(f"/api/projects/{project_id}/files/{file_id}", headers=headers)
    assert resp.status_code == 204


@pytest.mark.asyncio
async def test_upload_image(client, project_id) -> None:
    headers = make_auth_header()
    # 1x1 PNG
    png = (
        b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00\x00\x01"
        b"\x00\x00\x00\x01\x08\x02\x00\x00\x00\x90wS\xde\x00"
        b"\x00\x00\x0cIDATx\x9cc\xf8\x0f\x00\x00\x01\x01\x00"
        b"\x05\x18\xd8N\x00\x00\x00\x00IEND\xaeB`\x82"
    )
    resp = await client.post(
        f"/api/projects/{project_id}/images",
        files={"file": ("photo.png", png, "image/png")},
        headers=headers,
    )
    assert resp.status_code == 201
    assert resp.json()["filename"] == "photo.png"


@pytest.mark.asyncio
async def test_list_images(client, project_id) -> None:
    resp = await client.get(f"/api/projects/{project_id}/images")
    assert resp.status_code == 200
    assert isinstance(resp.json(), list)
