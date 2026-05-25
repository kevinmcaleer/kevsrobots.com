"""Part photo gallery — uploads, links, ordering, edit/delete, detail embed.

Photos are gated like part edits (14-day account age, faked open via
``_old_account``). The test client bypasses the T&Cs gate but NOT the age
gate, so the fixture is required.
"""

from __future__ import annotations

from datetime import datetime, timedelta
from typing import Optional

import pytest

from .conftest import make_auth_header

PNG = b"\x89PNG\r\n\x1a\n" + b"fake-image-bytes"


@pytest.fixture(autouse=True)
def _old_account(monkeypatch: pytest.MonkeyPatch) -> None:
    from projects_api import auth as auth_module

    async def _fake_age(username: str, token: str) -> Optional[datetime]:
        return datetime.utcnow() - timedelta(days=365 * 5)

    monkeypatch.setattr(auth_module, "fetch_account_created_at", _fake_age)
    auth_module._clear_account_age_cache()


async def _make_part(client, name="Servo", user="testuser") -> str:
    resp = await client.post("/api/parts", json={"name": name}, headers=make_auth_header(user))
    assert resp.status_code == 201, resp.text
    return resp.json()["slug"]


async def _upload(client, slug, filename="top.png", title="Top view", user="testuser"):
    return await client.post(
        f"/api/parts/{slug}/photos",
        files={"file": (filename, PNG, "image/png")},
        data={"title": title},
        headers=make_auth_header(user),
    )


@pytest.mark.asyncio
async def test_upload_photo_and_view(client) -> None:
    slug = await _make_part(client)
    resp = await _upload(client, slug)
    assert resp.status_code == 201, resp.text
    photo = resp.json()
    assert photo["is_external"] is False
    assert photo["title"] == "Top view"
    assert photo["url"] == f"/api/parts/{slug}/photos/{photo['id']}/view"
    # The bytes round-trip through storage.
    view = await client.get(photo["url"])
    assert view.status_code == 200
    assert view.content == PNG


@pytest.mark.asyncio
async def test_link_external_photo(client) -> None:
    slug = await _make_part(client)
    resp = await client.post(
        f"/api/parts/{slug}/photos/link",
        json={"external_url": "https://example.com/servo-side.jpg", "title": "Side"},
        headers=make_auth_header(),
    )
    assert resp.status_code == 201, resp.text
    photo = resp.json()
    assert photo["is_external"] is True
    assert photo["url"] == "https://example.com/servo-side.jpg"
    assert photo["title"] == "Side"


@pytest.mark.asyncio
async def test_link_rejects_non_http(client) -> None:
    slug = await _make_part(client)
    resp = await client.post(
        f"/api/parts/{slug}/photos/link",
        json={"external_url": "ftp://nope/img.png"},
        headers=make_auth_header(),
    )
    assert resp.status_code == 400


@pytest.mark.asyncio
async def test_list_and_detail_ordering(client) -> None:
    slug = await _make_part(client)
    await _upload(client, slug, filename="a.png", title="first")
    await client.post(
        f"/api/parts/{slug}/photos/link",
        json={"external_url": "https://example.com/b.jpg", "title": "second"},
        headers=make_auth_header(),
    )
    listed = (await client.get(f"/api/parts/{slug}/photos")).json()
    assert [p["title"] for p in listed] == ["first", "second"]
    assert [p["sort_order"] for p in listed] == [0, 1]

    # The gallery rides along on the part detail.
    detail = (await client.get(f"/api/parts/{slug}")).json()
    assert [p["title"] for p in detail["photos"]] == ["first", "second"]


@pytest.mark.asyncio
async def test_edit_title(client) -> None:
    slug = await _make_part(client)
    pid = (await _upload(client, slug)).json()["id"]
    resp = await client.put(
        f"/api/parts/{slug}/photos/{pid}",
        json={"title": "Pin-1 corner"},
        headers=make_auth_header(),
    )
    assert resp.status_code == 200, resp.text
    assert resp.json()["title"] == "Pin-1 corner"


@pytest.mark.asyncio
async def test_uploader_can_delete(client) -> None:
    slug = await _make_part(client)
    pid = (await _upload(client, slug)).json()["id"]
    resp = await client.delete(
        f"/api/parts/{slug}/photos/{pid}", headers=make_auth_header()
    )
    assert resp.status_code == 204
    assert (await client.get(f"/api/parts/{slug}/photos")).json() == []


@pytest.mark.asyncio
async def test_non_uploader_cannot_delete(client) -> None:
    slug = await _make_part(client)
    pid = (await _upload(client, slug, user="alice")).json()["id"]
    resp = await client.delete(
        f"/api/parts/{slug}/photos/{pid}", headers=make_auth_header("bob")
    )
    assert resp.status_code == 403


@pytest.mark.asyncio
async def test_photos_for_unknown_part_404(client) -> None:
    resp = await client.get("/api/parts/does-not-exist/photos")
    assert resp.status_code == 404
