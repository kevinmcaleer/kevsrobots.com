"""The part's electronics, edited on /parts/edit.html with the embedded
Snakie part editor (issue #269): the definition with its photo, a 3-D model
upload, photo licences on upload and "wrong pinout" reports."""

from __future__ import annotations

from datetime import datetime, timedelta
from pathlib import Path
from typing import Optional

import pytest
import yaml

from .conftest import make_auth_header

LIBRARY = Path(__file__).parent / "fixtures" / "snakie-library"
PNG = b"\x89PNG\r\n\x1a\n" + b"fake-image-bytes"
STL = b"solid cube\nendsolid cube\n"


@pytest.fixture(autouse=True)
def _old_account(monkeypatch: pytest.MonkeyPatch) -> None:
    from projects_api import auth as auth_module

    async def _fake_age(username: str, token: str) -> Optional[datetime]:
        return datetime.utcnow() - timedelta(days=365 * 5)

    monkeypatch.setattr(auth_module, "fetch_account_created_at", _fake_age)
    auth_module._clear_account_age_cache()


async def _create(client, name="Raspberry Pi Pico") -> str:
    resp = await client.post("/api/parts", json={"name": name}, headers=make_auth_header())
    assert resp.status_code == 201, resp.text
    return resp.json()["slug"]


async def _photo(client, slug, licence="own"):
    data = {"title": "Top"}
    if licence:
        data["licence"] = licence
    return await client.post(
        f"/api/parts/{slug}/photos", files={"file": ("top.png", PNG, "image/png")},
        data=data, headers=make_auth_header(),
    )


@pytest.mark.asyncio
async def test_upload_needs_a_licence(client) -> None:
    slug = await _create(client)
    assert (await _photo(client, slug, licence=None)).status_code == 422
    assert (await _photo(client, slug, licence="whatever")).status_code == 422
    resp = await _photo(client, slug, licence="cc-by")
    assert resp.status_code == 201, resp.text
    assert resp.json()["licence"] == "cc-by"


@pytest.mark.asyncio
async def test_definition_keeps_its_photo_and_drops_image_data(client) -> None:
    slug = await _create(client)
    photo = (await _photo(client, slug)).json()
    definition = yaml.safe_load((LIBRARY / "pico" / "parts.yml").read_text())
    definition["imageData"] = "data:image/png;base64,AAAA"
    definition["rear"] = {"imageData": "data:image/png;base64,BBBB", "pins": []}
    definition["photo_id"] = photo["id"]
    resp = await client.put(
        f"/api/parts/{slug}/definition", params={"change_summary": "Placed the pins"},
        json=definition, headers=make_auth_header(),
    )
    assert resp.status_code == 200, resp.text
    body = resp.json()
    assert "imageData" not in body["definition"]
    assert body["definition"]["rear"] == {"pins": []}
    assert body["definition"]["photo_id"] == photo["id"]
    assert body["recent_revisions"][0]["change_summary"] == "Placed the pins"

    # parts.yml uses that photo and doesn't carry our photo_id.
    out = yaml.safe_load((await client.get(f"/api/parts/{slug}/parts.yml")).text)
    assert out["image"].endswith(f"/api/parts/{slug}/photos/{photo['id']}/view")
    assert "photo_id" not in out


@pytest.mark.asyncio
async def test_definition_photo_must_be_this_parts(client) -> None:
    slug = await _create(client)
    other = await _create(client, "Other board")
    photo = (await _photo(client, other)).json()
    resp = await client.put(
        f"/api/parts/{slug}/definition", json={"headers": [], "photo_id": photo["id"]},
        headers=make_auth_header(),
    )
    assert resp.status_code == 422


@pytest.mark.asyncio
async def test_definition_restore(client) -> None:
    slug = await _create(client)
    one = {"headers": [{"edge": "left", "pins": [{"name": "GP0", "type": "io"}]}]}
    two = {"headers": [{"edge": "left", "pins": [{"name": "GP1", "type": "io"}]}]}
    for d in (one, two):
        assert (await client.put(f"/api/parts/{slug}/definition", json=d,
                                 headers=make_auth_header())).status_code == 200
    revs = (await client.get(f"/api/parts/{slug}/revisions")).json()
    first = [r for r in revs if r["change_summary"] == "Updated the part definition"][-1]
    resp = await client.post(f"/api/parts/{slug}/revisions/{first['id']}/restore",
                             headers=make_auth_header())
    assert resp.status_code == 200, resp.text
    assert resp.json()["definition"]["headers"][0]["pins"][0]["name"] == "GP0"


@pytest.mark.asyncio
async def test_mesh_upload_replace_and_remove(client) -> None:
    slug = await _create(client)
    bad = await client.post(f"/api/parts/{slug}/mesh", files={"file": ("m.gltf", b"{}", "model/gltf+json")},
                            headers=make_auth_header())
    assert bad.status_code == 400
    assert (await client.post(f"/api/parts/{slug}/mesh", files={"file": ("m.stl", STL, "model/stl")})).status_code == 401

    resp = await client.post(f"/api/parts/{slug}/mesh", files={"file": ("board.stl", STL, "model/stl")},
                             headers=make_auth_header())
    assert resp.status_code == 200, resp.text
    assert resp.json()["has_mesh"] is True
    assert resp.json()["mesh_format"] == "stl"
    assert resp.json()["recent_revisions"][0]["change_summary"] == "Added a 3-D model"
    got = await client.get(f"/api/parts/{slug}/mesh")
    assert got.status_code == 200 and got.content == STL
    assert got.headers["content-type"] == "model/stl"

    resp = await client.delete(f"/api/parts/{slug}/mesh", headers=make_auth_header())
    assert resp.status_code == 200
    assert resp.json()["has_mesh"] is False
    assert (await client.get(f"/api/parts/{slug}/mesh")).status_code == 404
    assert (await client.delete(f"/api/parts/{slug}/mesh", headers=make_auth_header())).status_code == 404

    # Restoring the revision that added it brings the model back.
    revs = (await client.get(f"/api/parts/{slug}/revisions")).json()
    added = next(r for r in revs if r["change_summary"] == "Added a 3-D model")
    resp = await client.post(f"/api/parts/{slug}/revisions/{added['id']}/restore", headers=make_auth_header())
    assert resp.status_code == 200, resp.text
    assert resp.json()["has_mesh"] is True
    assert (await client.get(f"/api/parts/{slug}/mesh")).content == STL


@pytest.mark.asyncio
async def test_wrong_pinout_report(client) -> None:
    slug = await _create(client)
    resp = await client.post(f"/api/parts/{slug}/report", json={"reason": "pinout", "note": "GP0 is GP1"},
                             headers=make_auth_header("someone"))
    assert resp.status_code == 201, resp.text
    assert resp.json()["reason"] == "pinout"
