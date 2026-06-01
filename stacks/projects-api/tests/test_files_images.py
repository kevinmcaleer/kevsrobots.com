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


# --- Image routing: images uploaded via the Files & Downloads section -----
#
# A .png/.jpg/etc. dropped on the Files & Downloads uploader should land in
# the Images gallery, not the downloads list — the backend detects it by
# extension and creates a ProjectImage instead of a ProjectFile.

_PNG_1X1 = (
    b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00\x00\x01"
    b"\x00\x00\x00\x01\x08\x02\x00\x00\x00\x90wS\xde\x00"
    b"\x00\x00\x0cIDATx\x9cc\xf8\x0f\x00\x00\x01\x01\x00"
    b"\x05\x18\xd8N\x00\x00\x00\x00IEND\xaeB`\x82"
)


@pytest.mark.asyncio
async def test_image_uploaded_via_files_endpoint_lands_in_images(
    client, project_id
) -> None:
    headers = make_auth_header()
    resp = await client.post(
        f"/api/projects/{project_id}/files",
        files={"file": ("diagram.png", _PNG_1X1, "image/png")},
        headers=headers,
    )
    assert resp.status_code == 201
    body = resp.json()
    # Image-shaped response: has sort_order, not the file_type/file_size of a
    # download row — this is the signal the frontend uses to refresh the
    # gallery.
    assert body["filename"] == "diagram.png"
    assert "file_type" not in body
    assert "sort_order" in body

    # It shows up in the Images list...
    images = await client.get(f"/api/projects/{project_id}/images")
    assert [i["filename"] for i in images.json()] == ["diagram.png"]

    # ...and NOT in the Files & Downloads list.
    files = await client.get(f"/api/projects/{project_id}/files")
    assert files.json() == []


@pytest.mark.asyncio
async def test_non_image_uploaded_via_files_endpoint_stays_a_file(
    client, project_id
) -> None:
    headers = make_auth_header()
    resp = await client.post(
        f"/api/projects/{project_id}/files",
        files={"file": ("firmware.bin", b"\x00\x01\x02", "application/octet-stream")},
        headers=headers,
    )
    # .bin isn't an allowed extension, so use a real file type instead.
    if resp.status_code == 400:
        resp = await client.post(
            f"/api/projects/{project_id}/files",
            files={"file": ("notes.txt", b"hello", "text/plain")},
            headers=headers,
        )
    assert resp.status_code == 201
    assert resp.json()["file_type"] in {"txt", "bin"}
    files = await client.get(f"/api/projects/{project_id}/files")
    assert len(files.json()) == 1
    images = await client.get(f"/api/projects/{project_id}/images")
    assert images.json() == []


@pytest.mark.asyncio
async def test_migrate_image_files_to_images_moves_existing_rows(
    sessionmaker_, monkeypatch
) -> None:
    """The backfill relocates image rows already sitting in project_files
    (uploaded before the routing existed) into project_images, drops their
    download-log rows, and is idempotent."""
    from projects_api import db as db_module
    from projects_api.models import (
        Download,
        Project,
        ProjectFile,
        ProjectImage,
    )

    monkeypatch.setattr(db_module, "get_sessionmaker", lambda: sessionmaker_)

    async with sessionmaker_() as s:
        project = Project(title="Legacy Project", author_username="testuser")
        s.add(project)
        await s.flush()
        # A stray image + a genuine download, both in project_files.
        img = ProjectFile(
            project_id=project.id,
            filename="old-photo.jpg",
            file_path="nas:projects/images/1/old-photo.jpg",
            file_size=123,
            file_type="jpg",
        )
        doc = ProjectFile(
            project_id=project.id,
            filename="schematic.pdf",
            file_path="nas:projects/files/1/schematic.pdf",
            file_size=456,
            file_type="pdf",
        )
        s.add_all([img, doc])
        await s.flush()
        s.add(Download(project_id=project.id, file_id=img.id, user_id="someone"))
        await s.commit()
        project_id = project.id

    await db_module.migrate_image_files_to_images()

    async with sessionmaker_() as s:
        from sqlalchemy import select

        files = (await s.execute(select(ProjectFile))).scalars().all()
        images = (await s.execute(select(ProjectImage))).scalars().all()
        downloads = (await s.execute(select(Download))).scalars().all()

        # The image moved; the PDF stayed.
        assert [f.filename for f in files] == ["schematic.pdf"]
        assert len(images) == 1
        assert images[0].filename == "old-photo.jpg"
        # file_path is preserved verbatim — the physical file never moved.
        assert images[0].file_path == "nas:projects/images/1/old-photo.jpg"
        assert images[0].project_id == project_id

        # The project had no cover — the moved image becomes it (editor's rule:
        # first image by sort_order), stored as an absolute /view URL.
        project = await s.get(Project, project_id)
        assert project.cover_image == (
            f"https://projects.kevsrobots.com/api/projects/{project_id}"
            f"/images/{images[0].id}/view"
        )
        # The orphaned download-log row was cleaned up.
        assert downloads == []

    # Idempotent — a second pass moves nothing.
    await db_module.migrate_image_files_to_images()
    async with sessionmaker_() as s:
        from sqlalchemy import select

        images = (await s.execute(select(ProjectImage))).scalars().all()
        assert len(images) == 1


@pytest.mark.asyncio
async def test_migrate_appends_and_preserves_existing_cover(
    sessionmaker_, monkeypatch
) -> None:
    """A project that already has gallery images + a chosen cover keeps that
    cover, and the moved image lands at the END of the gallery (so it can't
    steal the cover slot on the next editor load)."""
    from projects_api import db as db_module
    from projects_api.models import Project, ProjectFile, ProjectImage

    monkeypatch.setattr(db_module, "get_sessionmaker", lambda: sessionmaker_)

    async with sessionmaker_() as s:
        project = Project(
            title="Has Cover",
            author_username="kev",
            cover_image="https://projects.kevsrobots.com/api/projects/9/images/1/view",
        )
        s.add(project)
        await s.flush()
        # An existing gallery image at sort_order 0...
        s.add(ProjectImage(
            project_id=project.id,
            filename="hero.jpg",
            file_path="nas:projects/images/9/hero.jpg",
            sort_order=0,
        ))
        # ...and a stray image still in project_files.
        s.add(ProjectFile(
            project_id=project.id,
            filename="stray.png",
            file_path="nas:projects/images/9/stray.png",
            file_size=10,
            file_type="png",
        ))
        await s.commit()
        pid = project.id
        original_cover = project.cover_image

    await db_module.migrate_image_files_to_images()

    async with sessionmaker_() as s:
        from sqlalchemy import select

        project = await s.get(Project, pid)
        # Existing cover untouched.
        assert project.cover_image == original_cover

        images = (
            await s.execute(
                select(ProjectImage)
                .where(ProjectImage.project_id == pid)
                .order_by(ProjectImage.sort_order)
            )
        ).scalars().all()
        # Moved image appended AFTER the existing one.
        assert [i.filename for i in images] == ["hero.jpg", "stray.png"]
        assert images[1].sort_order == 1


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
