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


# --- Per-file descriptions (issue #187) ----------------------------------
#
# Partial-update semantics chosen here:
#   * ``description: null`` from the client means "field omitted" (we don't
#     touch the existing value). Pydantic only puts fields the client
#     actually included in ``model_dump(exclude_unset=True)``.
#   * ``description: ""`` (explicit empty string) clears the description
#     to the empty string rather than NULL. The frontend treats both null
#     and "" as "no description" so the user-visible behaviour is
#     identical, but the backend keeps the distinction so an empty-string
#     clear can't accidentally re-pull a previously-cleared description.
#   * Omitting the key entirely leaves the row unchanged.


@pytest.fixture
async def uploaded_file(client, project_id):
    """A single file upload, returns the file_id."""
    headers = make_auth_header()
    resp = await client.post(
        f"/api/projects/{project_id}/files",
        files={"file": ("blueprint.pdf", b"%PDF-1.4 fake pdf", "application/pdf")},
        headers=headers,
    )
    assert resp.status_code == 201
    return resp.json()["id"]


@pytest.mark.asyncio
async def test_uploaded_file_description_defaults_to_none(
    client, project_id, uploaded_file
) -> None:
    resp = await client.get(f"/api/projects/{project_id}/files")
    assert resp.status_code == 200
    rows = resp.json()
    assert len(rows) == 1
    assert rows[0]["id"] == uploaded_file
    assert rows[0]["description"] is None


@pytest.mark.asyncio
async def test_set_file_description_persists(
    client, project_id, uploaded_file
) -> None:
    headers = make_auth_header()
    resp = await client.put(
        f"/api/projects/{project_id}/files/{uploaded_file}",
        json={"description": "Pin diagram for v1.0"},
        headers=headers,
    )
    assert resp.status_code == 200
    assert resp.json()["description"] == "Pin diagram for v1.0"
    # And it persists on a subsequent GET.
    get_resp = await client.get(f"/api/projects/{project_id}/files")
    assert get_resp.json()[0]["description"] == "Pin diagram for v1.0"


@pytest.mark.asyncio
async def test_empty_string_clears_file_description(
    client, project_id, uploaded_file
) -> None:
    headers = make_auth_header()
    # First set a description.
    await client.put(
        f"/api/projects/{project_id}/files/{uploaded_file}",
        json={"description": "Initial description"},
        headers=headers,
    )
    # Now clear it with an explicit empty string.
    resp = await client.put(
        f"/api/projects/{project_id}/files/{uploaded_file}",
        json={"description": ""},
        headers=headers,
    )
    assert resp.status_code == 200
    # Spec: empty string clears to "" (not None) — locking the semantic in
    # so a future refactor can't silently flip behaviour.
    assert resp.json()["description"] == ""


@pytest.mark.asyncio
async def test_omitting_description_leaves_existing_value_untouched(
    client, project_id, uploaded_file
) -> None:
    headers = make_auth_header()
    # Set an initial value.
    await client.put(
        f"/api/projects/{project_id}/files/{uploaded_file}",
        json={"description": "Wiring guide"},
        headers=headers,
    )
    # PUT an empty body — no field included means no change.
    resp = await client.put(
        f"/api/projects/{project_id}/files/{uploaded_file}",
        json={},
        headers=headers,
    )
    assert resp.status_code == 200
    assert resp.json()["description"] == "Wiring guide"


@pytest.mark.asyncio
async def test_non_owner_cannot_update_file_description(
    client, project_id, uploaded_file
) -> None:
    other = make_auth_header("intruder")
    resp = await client.put(
        f"/api/projects/{project_id}/files/{uploaded_file}",
        json={"description": "Hijacked!"},
        headers=other,
    )
    assert resp.status_code == 403


@pytest.mark.asyncio
async def test_public_list_surfaces_file_description(
    client, project_id, uploaded_file
) -> None:
    """Anonymous GET of the file list shows the description for files that
    have one set — the public view page reads exactly this response."""
    headers = make_auth_header()
    await client.put(
        f"/api/projects/{project_id}/files/{uploaded_file}",
        json={"description": "Drill template (PDF)"},
        headers=headers,
    )
    # No auth header — same shape the public view page hits.
    resp = await client.get(f"/api/projects/{project_id}/files")
    assert resp.status_code == 200
    rows = resp.json()
    assert any(
        r["id"] == uploaded_file and r["description"] == "Drill template (PDF)"
        for r in rows
    )
