"""Snakie parts.yml fields on catalog parts, the parts.yml export, the
definition upload and the seed import (issue #266)."""

from __future__ import annotations

from datetime import datetime, timedelta
from pathlib import Path
from typing import Optional

import pytest
import yaml
from sqlalchemy import select

from projects_api.import_snakie_parts import import_library
from projects_api.models import Part, PartPhoto, PartRevision
from .conftest import make_auth_header

LIBRARY = Path(__file__).parent / "fixtures" / "snakie-library"


@pytest.fixture(autouse=True)
def _old_account(monkeypatch: pytest.MonkeyPatch) -> None:
    from projects_api import auth as auth_module

    async def _fake_age(username: str, token: str) -> Optional[datetime]:
        return datetime.utcnow() - timedelta(days=365 * 5)

    monkeypatch.setattr(auth_module, "fetch_account_created_at", _fake_age)
    auth_module._clear_account_age_cache()


async def _create(client, name="Raspberry Pi Pico") -> dict:
    resp = await client.post("/api/parts", json={"name": name}, headers=make_auth_header())
    assert resp.status_code == 201, resp.text
    return resp.json()


@pytest.mark.asyncio
async def test_spec_fields_edit_and_restore(client) -> None:
    part = await _create(client)
    slug = part["slug"]
    first_rev = part["current_revision_id"]
    resp = await client.put(f"/api/parts/{slug}", json={
        "change_summary": "Specs",
        "manufacturer": "Raspberry Pi",
        "package": "THT",
        "voltage": "1.8–5.5V",
        "pin_spacing": 2.54,
        "dimensions_mm": {"width": 21, "height": 51},
        "datasheet_url": "https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf",
    }, headers=make_auth_header())
    assert resp.status_code == 200, resp.text
    body = resp.json()
    assert body["manufacturer"] == "Raspberry Pi"
    assert body["dimensions_mm"] == {"width": 21.0, "height": 51.0}
    assert body["pin_spacing"] == 2.54

    # The revision snapshots them.
    rev_id = body["current_revision_id"]
    rev = (await client.get(f"/api/parts/{slug}/revisions/{rev_id}")).json()
    assert rev["voltage"] == "1.8–5.5V"

    # Clearing one is a change; sending the same values is not.
    resp = await client.put(f"/api/parts/{slug}", json={
        "change_summary": "Same", "manufacturer": "Raspberry Pi",
    }, headers=make_auth_header())
    assert resp.status_code == 400
    resp = await client.put(f"/api/parts/{slug}", json={
        "change_summary": "Clear", "manufacturer": "",
    }, headers=make_auth_header())
    assert resp.json()["manufacturer"] is None
    assert resp.json()["package"] == "THT"

    # Restoring the first revision restores the empty spec.
    resp = await client.post(f"/api/parts/{slug}/revisions/{first_rev}/restore",
                             headers=make_auth_header())
    assert resp.status_code == 200
    assert resp.json()["package"] is None
    assert resp.json()["dimensions_mm"] is None


@pytest.mark.asyncio
async def test_put_definition_updates_spec_and_exports(client) -> None:
    part = await _create(client)
    slug = part["slug"]
    definition = yaml.safe_load((LIBRARY / "pico" / "parts.yml").read_text())
    resp = await client.put(f"/api/parts/{slug}/definition", json=definition,
                            headers=make_auth_header())
    assert resp.status_code == 200, resp.text
    body = resp.json()
    assert body["manufacturer"] == "Raspberry Pi"
    assert body["package"] == "THT"
    assert body["dimensions_mm"] == {"width": 21.0, "height": 51.0}
    assert len(body["definition"]["headers"]) == 2
    assert "image" not in body["definition"]  # file references are dropped
    assert body["name"] == "Raspberry Pi Pico"

    # Same definition again: nothing changed.
    resp = await client.put(f"/api/parts/{slug}/definition", json=definition,
                            headers=make_auth_header())
    assert resp.status_code == 400

    resp = await client.get(f"/api/parts/{slug}/parts.yml")
    assert resp.status_code == 200
    assert resp.headers["content-type"].startswith("application/yaml")
    out = yaml.safe_load(resp.text)
    assert out["id"] == slug
    assert out["name"] == "Raspberry Pi Pico"
    assert out["manufacturer"] == "Raspberry Pi"
    assert out["pinSpacing"] == 2.54
    assert out["dimensions"] == {"width": 21.0, "height": 51.0}
    assert out["headers"][0]["pins"][0]["name"] == "GP0"
    assert out["electrical"]["model"] == "regulator"
    assert "image" not in out and "help" not in out


@pytest.mark.asyncio
async def test_put_definition_rejects_bad_headers(client) -> None:
    part = await _create(client)
    resp = await client.put(f"/api/parts/{part['slug']}/definition",
                            json={"headers": "nope"}, headers=make_auth_header())
    assert resp.status_code == 422
    assert (await client.put(f"/api/parts/{part['slug']}/definition",
                             json={"headers": []})).status_code == 401


@pytest.mark.asyncio
async def test_parts_yml_minimal_part(client) -> None:
    part = await _create(client, "Mystery Board")
    out = yaml.safe_load((await client.get(f"/api/parts/{part['slug']}/parts.yml")).text)
    assert out == {"id": "mystery-board", "name": "Mystery Board", "headers": []}


@pytest.mark.asyncio
async def test_import_is_idempotent(session) -> None:
    first = await import_library(LIBRARY, session)
    assert sorted(first.created) == ["pico", "sg90"]
    assert first.skipped == []

    second = await import_library(LIBRARY, session)
    assert second.created == [] and second.updated == []
    assert sorted(second.unchanged) == ["pico", "sg90"]

    pico = await session.scalar(select(Part).where(Part.slug == "pico"))
    assert pico.name == "Raspberry Pi Pico"
    assert pico.manufacturer == "Raspberry Pi"
    assert pico.mpn == "SC0915"
    assert pico.status == "draft"
    assert pico.definition["headers"][0]["edge"] == "left"
    revs = (await session.scalars(select(PartRevision).where(PartRevision.part_id == pico.id))).all()
    assert [r.author for r in revs] == ["snakie-import"]
    assert revs[0].change_summary == "Imported from Standard Parts 1.17.0"

    photos = (await session.scalars(select(PartPhoto).where(PartPhoto.part_id == pico.id))).all()
    assert len(photos) == 1
    assert photos[0].source == "Standard Parts 1.17.0 (Snakie)"
    assert photos[0].licence is None

    sg90 = await session.scalar(select(Part).where(Part.slug == "sg90"))
    assert sg90.mesh_file_path


@pytest.mark.asyncio
async def test_import_updates_existing_part_without_clobbering(client, session) -> None:
    resp = await client.post("/api/parts", json={
        "name": "Raspberry Pi Pico", "description_md": "Our own write-up.", "tags": ["favourite"],
    }, headers=make_auth_header())
    slug = resp.json()["slug"]  # raspberry-pi-pico: matched by name, not slug

    report = await import_library(LIBRARY, session)
    assert "pico" in report.updated

    detail = (await client.get(f"/api/parts/{slug}")).json()
    assert detail["description_md"] == "Our own write-up."
    assert detail["manufacturer"] == "Raspberry Pi"
    assert "favourite" in detail["tags"] and "rp2040" in detail["tags"]
    assert detail["photos"][0]["needs_review"] is True
    assert (await client.get("/api/parts/pico")).status_code == 404


@pytest.mark.asyncio
async def test_mesh_is_served(client, session) -> None:
    await import_library(LIBRARY, session)
    resp = await client.get("/api/parts/sg90/mesh")
    assert resp.status_code == 200
    assert resp.content.startswith(b"solid")
    out = yaml.safe_load((await client.get("/api/parts/sg90/parts.yml")).text)
    assert out["mesh"].endswith("/api/parts/sg90/mesh")
    assert "image" not in out  # this fixture has no image file
    pico = yaml.safe_load((await client.get("/api/parts/pico/parts.yml")).text)
    assert pico["image"].startswith("https://projects.kevsrobots.com/api/parts/pico/photos/")
    assert (await client.get("/api/parts/sg90")).json()["has_mesh"] is True


@pytest.mark.asyncio
async def test_photo_credit(client, session, monkeypatch) -> None:
    await import_library(LIBRARY, session)
    photo = (await client.get("/api/parts/pico")).json()["photos"][0]
    assert photo["needs_review"] is True

    # Only the uploader (snakie-import) or an admin may edit it.
    url = f"/api/parts/pico/photos/{photo['id']}"
    assert (await client.put(url, json={"licence": "cc-by"},
                             headers=make_auth_header())).status_code == 403
    monkeypatch.setenv("ADMIN_USERNAMES", "admin")
    resp = await client.put(url, json={"licence": "not-a-licence"},
                            headers=make_auth_header("admin"))
    assert resp.status_code == 422
    resp = await client.put(url, json={"licence": "permission-granted:pimoroni"},
                            headers=make_auth_header("admin"))
    assert resp.status_code == 200
    assert resp.json()["licence"] == "permission-granted:pimoroni"
    assert resp.json()["needs_review"] is False


@pytest.mark.asyncio
async def test_admin_photo_review_queue(client, session, monkeypatch) -> None:
    monkeypatch.setenv("ADMIN_USERNAMES", "admin")
    await import_library(LIBRARY, session)
    assert (await client.get("/api/admin/parts/photos/needs-review",
                             headers=make_auth_header())).status_code == 403
    queue = (await client.get("/api/admin/parts/photos/needs-review",
                              headers=make_auth_header("admin"))).json()
    assert [q["part_slug"] for q in queue] == ["pico"]
    await client.put(f"/api/parts/pico/photos/{queue[0]['id']}", json={"licence": "own"},
                     headers=make_auth_header("admin"))
    assert (await client.get("/api/admin/parts/photos/needs-review",
                             headers=make_auth_header("admin"))).json() == []


@pytest.mark.asyncio
async def test_resolve_exact_names(client, session) -> None:
    await import_library(LIBRARY, session)
    resp = await client.get("/api/parts/_resolve", params=[
        ("names", "raspberry pi pico"), ("names", "SC0915"), ("names", "Pico"), ("names", "  "),
    ])
    assert resp.status_code == 200, resp.text
    got = {r["name"]: r["slug"] for r in resp.json()}
    assert got == {"raspberry pi pico": "pico", "SC0915": "pico", "Pico": None}
    assert resp.json()[0]["cover_url"].startswith("/api/parts/pico/photos/")


def _tarball(files: dict[str, bytes]) -> bytes:
    import io
    import tarfile
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w:gz") as tar:
        for name, data in files.items():
            info = tarfile.TarInfo(name)
            info.size = len(data)
            tar.addfile(info, io.BytesIO(data))
    return buf.getvalue()


def test_extract_library_keeps_only_the_standard_library(tmp_path) -> None:
    from projects_api.import_snakie_parts import extract_library
    data = _tarball({
        "Snakie-master/README.md": b"x",
        "Snakie-master/examples/parts/snakie-standard/library.yml": b"id: snakie-standard\n",
        "Snakie-master/examples/parts/snakie-standard/pico/parts.yml": b"id: pico\nname: Pico\n",
        "Snakie-master/examples/parts/snakie-basics/vl53l0x/parts.yml": b"id: vl53l0x\n",
        "Snakie-master/examples/parts/snakie-standard/../../../../escape.txt": b"no",
    })
    root = extract_library(data, tmp_path / "lib")
    assert (root / "pico" / "parts.yml").read_text() == "id: pico\nname: Pico\n"
    assert (root / "library.yml").is_file()
    assert not (root / "vl53l0x").exists()
    assert not (tmp_path / "escape.txt").exists()
    assert sorted(p.name for p in root.iterdir()) == ["library.yml", "pico"]


def test_extract_library_without_the_library_fails(tmp_path) -> None:
    from projects_api.import_snakie_parts import extract_library
    with pytest.raises(ValueError):
        extract_library(_tarball({"Snakie-master/README.md": b"x"}), tmp_path / "lib")


@pytest.mark.asyncio
async def test_admin_sync_snakie(client, monkeypatch) -> None:
    from projects_api.routers import admin_parts

    def _fake_download(ref: str, dest: Path) -> Path:
        assert ref == "master"
        return LIBRARY

    monkeypatch.setattr(admin_parts, "download_github_library", _fake_download)
    url = "/api/admin/parts/sync-snakie"
    assert (await client.post(url, headers=make_auth_header())).status_code == 403

    monkeypatch.setenv("ADMIN_USERNAMES", "admin")
    admin = make_auth_header("admin")
    first = await client.post(url, headers=admin)
    assert first.status_code == 200, first.text
    assert first.json()["created"]
    again = (await client.post(url, headers=admin)).json()
    assert again["created"] == [] and again["unchanged"]

    assert (await client.post(url + "?ref=../x", headers=admin)).status_code == 400


@pytest.mark.asyncio
async def test_admin_sync_snakie_download_failure(client, monkeypatch) -> None:
    from projects_api.routers import admin_parts

    def _fail(ref: str, dest: Path) -> Path:
        raise OSError("no route to host")

    monkeypatch.setattr(admin_parts, "download_github_library", _fail)
    monkeypatch.setenv("ADMIN_USERNAMES", "admin")
    resp = await client.post("/api/admin/parts/sync-snakie", headers=make_auth_header("admin"))
    assert resp.status_code == 502
    assert "no route to host" in resp.json()["detail"]


def test_smb_chunks_stay_under_the_credit_limit() -> None:
    from projects_api.storage import _smb_read_chunks, _smb_write_chunks

    class FakeOpen:
        def __init__(self) -> None:
            self.data = bytearray()
            self.sizes: list[int] = []

        def write(self, data: bytes, offset: int) -> None:
            self.sizes.append(len(data))
            self.data[offset:offset + len(data)] = data

        def read(self, offset: int, length: int) -> bytes:
            self.sizes.append(length)
            return bytes(self.data[offset:offset + length])

    content = bytes(range(256)) * (6 * 4096 + 7)  # ~6 MB, like the LiPo photos
    f = FakeOpen()
    _smb_write_chunks(f, content)
    assert bytes(f.data) == content
    assert max(f.sizes) <= 1024 * 1024
    f.sizes.clear()
    assert _smb_read_chunks(f, len(content)) == content
    assert max(f.sizes) <= 1024 * 1024


@pytest.mark.asyncio
async def test_parts_yml_prefers_the_snakie_picture_over_catalog_photos(client, session) -> None:
    # A part made on the site with its own product photo, later matched by
    # the Snakie import: Snakie's pins are placed over Snakie's picture, so
    # that is the image the wiring editor must draw it with.
    part = await _create(client)
    resp = await client.post(
        f"/api/parts/{part['slug']}/photos",
        files={"file": ("box.png", b"\x89PNG\r\n\x1a\nproduct-shot", "image/png")},
        data={"title": "Product shot", "licence": "own"}, headers=make_auth_header(),
    )
    assert resp.status_code == 201, resp.text
    own = resp.json()

    await import_library(LIBRARY, session)
    photos = (await client.get(f"/api/parts/{part['slug']}")).json()["photos"]
    assert photos[0]["id"] == own["id"]  # still the catalog's cover photo
    snakie = next(p for p in photos if (p["source"] or "").endswith("(Snakie)"))

    out = yaml.safe_load((await client.get(f"/api/parts/{part['slug']}/parts.yml")).text)
    assert out["image"].endswith(f"/photos/{snakie['id']}/view")


@pytest.mark.asyncio
async def test_new_library_version_keeps_one_snakie_picture(session, tmp_path) -> None:
    import shutil

    await import_library(LIBRARY, session)
    newer = tmp_path / "snakie-standard"
    shutil.copytree(LIBRARY, newer)
    (newer / "library.yml").write_text("id: snakie-standard\nname: Standard Parts\nversion: 1.18.0\n")

    await import_library(newer, session)
    pico = await session.scalar(select(Part).where(Part.slug == "pico"))
    photos = (await session.scalars(select(PartPhoto).where(PartPhoto.part_id == pico.id))).all()
    assert len(photos) == 1

    # A redrawn picture replaces the file and asks for its licence again.
    photos[0].licence = "own"
    await session.commit()
    (newer / "pico" / "image.png").write_bytes(b"\x89PNG\r\n\x1a\nredrawn")
    report = await import_library(newer, session)
    assert "pico" in report.updated
    photos = (await session.scalars(select(PartPhoto).where(PartPhoto.part_id == pico.id))).all()
    assert len(photos) == 1
    assert photos[0].licence is None
    assert photos[0].source == "Standard Parts 1.18.0 (Snakie)"
