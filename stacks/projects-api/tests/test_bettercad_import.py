"""BetterCad build steps sent to a project and imported in the editor."""

from __future__ import annotations

import io
import zipfile

import pytest
from sqlalchemy import select

from projects_api import bettercad_import as bc
from projects_api.models import ProjectFile, ProjectImage
from .conftest import make_auth_header

PNG = b"\x89PNG\r\n\x1a\n" + b"\x00" * 32

README = """---
title: "Bracket"
steps: 2
generator: BetterCad
---

# How to build Bracket

2 steps in BetterCad. Sizes are in millimetres. Open [Bracket.bcad](Bracket.bcad) in BetterCad to change it.

![Bracket, finished](images/finished.png)

## Parameters

| Name | Value | Comment |
| --- | --- | --- |
| `width` | 40 mm |  |

## Step 1: Sketch1

![Step 1: Sketch1](images/step-01.png)

Draw a rectangle.

## Step 2: Extrude1

![Step 2: Extrude1](images/step-02.png)

Extrude **Sketch1**: 5 mm.
"""


def make_zip(readme: str = README, extra: dict[str, bytes] | None = None) -> bytes:
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as z:
        z.writestr("README.md", readme)
        z.writestr("Bracket.bcad", '{"bcad": 1}')
        z.writestr("models/Bracket.stl", "solid b\nendsolid b\n")
        for name in ("images/step-01.png", "images/step-02.png", "images/finished.png"):
            z.writestr(name, PNG)
        for name, data in (extra or {}).items():
            z.writestr(name, data)
    return buf.getvalue()


def read(data: bytes) -> bc.Bundle:
    return bc.read_bundle(data, max_image=10_000_000, max_file=10_000_000)


# --- the zip and the text ---


def test_reads_the_bundle() -> None:
    b = read(make_zip())
    assert b.title == "Bracket"
    assert b.steps == 2
    assert list(b.images)[0] == "images/finished.png"
    assert set(b.files) == {"Bracket.bcad", "models/Bracket.stl"}


@pytest.mark.parametrize("name,why", [
    ("../evil.stl", "isn't allowed"),
    ("/etc/passwd.txt", "isn't allowed"),
    ("images/../../x.png", "isn't allowed"),
    ("images/run.exe", "Only PNG and JPEG"),
    ("tool.exe", "can't go in a project"),
    ("other/thing.stl", "doesn't use"),
])
def test_refuses_bad_entries(name, why) -> None:
    with pytest.raises(bc.BundleError, match=why):
        read(make_zip(extra={name: b"x"}))


def test_refuses_a_fake_picture() -> None:
    with pytest.raises(bc.BundleError, match="isn't a PNG"):
        read(make_zip(extra={"images/step-03.png": b"GIF89a"}))


def test_refuses_no_readme_or_not_bettercad() -> None:
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as z:
        z.writestr("Bracket.bcad", "{}")
    with pytest.raises(bc.BundleError, match="no README"):
        read(buf.getvalue())
    with pytest.raises(bc.BundleError, match="wasn't made by BetterCad"):
        read(make_zip(readme="# Hello\n"))
    with pytest.raises(bc.BundleError, match="isn't a .zip"):
        read(b"not a zip")


def test_section_moves_headings_down_and_links_to_stored_things() -> None:
    urls = {"images/step-01.png": "https://x/1", "Bracket.bcad": "https://x/f"}
    s = bc.section(README, urls.get)
    assert s.startswith(bc.START + "\n## Build steps\n")
    assert s.endswith(bc.END)
    assert "# How to build" not in s
    assert "### Step 1: Sketch1" in s
    assert "### Parameters" in s
    assert "](https://x/1)" in s
    assert "[Bracket.bcad](https://x/f)" in s
    # Unknown links stay as they were.
    assert "](images/step-02.png)" in s
    assert "generator" not in s


def test_splice_appends_then_replaces_only_the_section() -> None:
    first = bc.splice("# My robot\n\nMy words.\n", f"{bc.START}\nold\n{bc.END}")
    assert first == f"# My robot\n\nMy words.\n\n{bc.START}\nold\n{bc.END}\n"
    second = bc.splice(first + "\nMore of mine.\n", f"{bc.START}\nnew\n{bc.END}")
    assert "old" not in second
    assert second.startswith("# My robot\n\nMy words.\n")
    assert second.endswith(f"{bc.START}\nnew\n{bc.END}\n\nMore of mine.\n")
    assert bc.splice(None, "S") == "S\n"


# --- the API ---


@pytest.fixture
async def project_id(client) -> int:
    resp = await client.post(
        "/api/projects", json={"title": "Robot arm build", "status": "draft"},
        headers=make_auth_header(),
    )
    return resp.json()["id"]


async def send(client, project_id, data=None, user="testuser"):
    return await client.post(
        f"/api/projects/{project_id}/imports",
        files={"file": ("Bracket build steps.zip", data or make_zip(), "application/zip")},
        data={"model_url": "https://bettercad.kevsrobots.com/#/m/abc"},
        headers=make_auth_header(user),
    )


@pytest.mark.asyncio
async def test_send_then_import(client, session, project_id) -> None:
    resp = await send(client, project_id)
    assert resp.status_code == 201, resp.text
    sent = resp.json()
    assert sent["status"] == "pending"
    assert (sent["title"], sent["steps"], sent["pictures"], sent["files"]) == ("Bracket", 2, 3, 2)
    assert sent["is_update"] is False

    pending = (await client.get(f"/api/projects/{project_id}/imports/pending", headers=make_auth_header())).json()
    assert pending["id"] == sent["id"]

    resp = await client.post(
        f"/api/projects/{project_id}/imports/{sent['id']}/apply",
        json={"content_md": "# Robot arm\n\nIntro.\n"}, headers=make_auth_header(),
    )
    assert resp.status_code == 200, resp.text
    applied = resp.json()
    assert applied["pictures"] == 3 and applied["files"] == 2
    assert applied["import"]["status"] == "applied"
    text = applied["content_md"]
    assert text.startswith("# Robot arm\n\nIntro.\n")
    assert "### Step 2: Extrude1" in text
    assert f"/api/projects/{project_id}/images/" in text
    assert f"/api/projects/{project_id}/files/" in text

    images = (await session.scalars(select(ProjectImage).where(ProjectImage.project_id == project_id))).all()
    assert len(images) == 3
    assert images[0].filename.endswith("finished.png")
    files = (await session.scalars(select(ProjectFile).where(ProjectFile.project_id == project_id))).all()
    assert {f.filename for f in files} == {"Bracket.bcad", "Bracket.stl"}
    assert {f.file_type for f in files if f.filename.endswith(".stl")} == {"model"}

    nothing = await client.get(f"/api/projects/{project_id}/imports/pending", headers=make_auth_header())
    assert nothing.json() is None


@pytest.mark.asyncio
async def test_update_replaces_the_last_import(client, session, project_id) -> None:
    first = (await send(client, project_id)).json()
    applied = (await client.post(
        f"/api/projects/{project_id}/imports/{first['id']}/apply",
        json={"content_md": "Mine.\n"}, headers=make_auth_header(),
    )).json()

    second = (await send(client, project_id)).json()
    assert second["is_update"] is True
    resp = await client.post(
        f"/api/projects/{project_id}/imports/{second['id']}/apply",
        json={"content_md": applied["content_md"] + "\nAfter.\n"}, headers=make_auth_header(),
    )
    text = resp.json()["content_md"]
    assert resp.json()["import"]["is_update"] is True
    assert text.count(bc.START) == 1
    assert text.startswith("Mine.\n") and text.endswith("\nAfter.\n")
    images = (await session.scalars(select(ProjectImage).where(ProjectImage.project_id == project_id))).all()
    assert len(images) == 3
    files = (await session.scalars(select(ProjectFile).where(ProjectFile.project_id == project_id))).all()
    assert len(files) == 2


@pytest.mark.asyncio
async def test_a_newer_send_replaces_a_pending_one(client, project_id) -> None:
    first = (await send(client, project_id)).json()
    second = (await send(client, project_id)).json()
    pending = (await client.get(f"/api/projects/{project_id}/imports/pending", headers=make_auth_header())).json()
    assert pending["id"] == second["id"]
    resp = await client.post(
        f"/api/projects/{project_id}/imports/{first['id']}/apply", json={}, headers=make_auth_header(),
    )
    assert resp.status_code == 409


@pytest.mark.asyncio
async def test_dismiss(client, project_id) -> None:
    sent = (await send(client, project_id)).json()
    resp = await client.post(f"/api/projects/{project_id}/imports/{sent['id']}/dismiss", headers=make_auth_header())
    assert resp.json()["status"] == "dismissed"
    nothing = await client.get(f"/api/projects/{project_id}/imports/pending", headers=make_auth_header())
    assert nothing.json() is None


@pytest.mark.asyncio
async def test_only_the_author_and_only_good_zips(client, project_id) -> None:
    assert (await send(client, project_id, user="someoneelse")).status_code == 403
    assert (await client.get(f"/api/projects/{project_id}/imports/pending")).status_code == 401
    bad = await send(client, project_id, data=make_zip(extra={"../x.stl": b"x"}))
    assert bad.status_code == 400
    assert "isn't allowed" in bad.json()["detail"]
    assert (await send(client, 99999)).status_code == 404
