"""Parts submitted from the project editor's "New part" dialog.

Any signed-in account may submit one, however new. It waits as
``pending`` (hidden from search and the catalog for everyone but its
sender) until an admin approves or rejects it.
"""

from __future__ import annotations

from datetime import datetime, timedelta
from typing import Optional

import pytest

from .conftest import make_auth_header

PNG = b"\x89PNG\r\n\x1a\n" + b"fake-image-bytes"


@pytest.fixture(autouse=True)
def _ages(monkeypatch: pytest.MonkeyPatch) -> None:
    """``newbie`` signed up yesterday; everyone else years ago."""
    from projects_api import auth as auth_module

    async def _fake_age(username: str, token: str) -> Optional[datetime]:
        days = 1 if username == "newbie" else 365 * 5
        return datetime.utcnow() - timedelta(days=days)

    monkeypatch.setattr(auth_module, "fetch_account_created_at", _fake_age)
    auth_module._clear_account_age_cache()
    monkeypatch.setenv("ADMIN_USERNAMES", "admin")


FULL = {
    "name": "HENGE 8A UBEC",
    "description_md": "5V/6V switching regulator",
    "category": "power",
    "manufacturer": "HENGE",
    "voltage": "7-25.5V in",
    "datasheet_url": "https://example.com/ubec.pdf",
    "links": [
        {"url": "https://shop.example.com/ubec", "name": "Example Shop", "unit_cost": 6.5, "currency_code": "GBP"},
        {"url": "https://shop.example.com/ubec", "name": "Duplicate"},
    ],
}


async def _submit(client, body=None, user="newbie"):
    return await client.post("/api/parts/submit", json=body or FULL, headers=make_auth_header(user))


@pytest.mark.asyncio
async def test_new_account_can_submit_and_it_is_pending(client) -> None:
    resp = await _submit(client)
    assert resp.status_code == 201, resp.text
    part = resp.json()
    assert part["status"] == "pending"
    assert part["created_by"] == "newbie"
    assert part["manufacturer"] == "HENGE"
    assert part["datasheet_url"] == "https://example.com/ubec.pdf"
    assert len(part["suppliers"]) == 1  # the duplicate URL is dropped
    shop = part["suppliers"][0]
    assert shop["url"] == "https://shop.example.com/ubec"
    assert shop["unit_cost"] == 6.5 and shop["currency_code"] == "GBP"
    # The old route still holds new accounts back.
    old = await client.post("/api/parts", json={"name": "Thing"}, headers=make_auth_header("newbie"))
    assert old.status_code == 403


@pytest.mark.asyncio
async def test_admin_submission_goes_straight_in(client) -> None:
    resp = await _submit(client, user="admin")
    assert resp.status_code == 201, resp.text
    assert resp.json()["status"] == "draft"


@pytest.mark.asyncio
async def test_pending_part_only_listed_for_its_sender(client) -> None:
    slug = (await _submit(client)).json()["slug"]
    mine = await client.get("/api/parts?q=ubec", headers=make_auth_header("newbie"))
    assert [p["slug"] for p in mine.json()] == [slug]
    others = await client.get("/api/parts?q=ubec", headers=make_auth_header("bob"))
    assert others.json() == []
    anon = await client.get("/api/parts?q=ubec")
    assert anon.json() == []
    cat_anon = await client.get("/api/parts/_catalog")
    assert slug not in [p["slug"] for p in cat_anon.json()]
    cat_mine = await client.get("/api/parts/_catalog", headers=make_auth_header("newbie"))
    assert slug in [p["slug"] for p in cat_mine.json()]
    res = await client.get("/api/parts/_resolve?names=HENGE 8A UBEC")
    assert res.json()[0]["slug"] is None
    res_mine = await client.get("/api/parts/_resolve?names=HENGE 8A UBEC", headers=make_auth_header("newbie"))
    assert res_mine.json()[0]["slug"] == slug
    # Its own page still opens, for the projects linking it.
    page = await client.get(f"/api/parts/{slug}")
    assert page.status_code == 200 and page.json()["status"] == "pending"


@pytest.mark.asyncio
async def test_sender_can_add_photos_and_edit_while_pending(client) -> None:
    slug = (await _submit(client)).json()["slug"]
    up = await client.post(
        f"/api/parts/{slug}/photos",
        files={"file": ("top.png", PNG, "image/png")},
        data={"licence": "own"},
        headers=make_auth_header("newbie"),
    )
    assert up.status_code == 201, up.text
    link = await client.post(
        f"/api/parts/{slug}/photos/link",
        json={"external_url": "https://example.com/side.jpg"},
        headers=make_auth_header("newbie"),
    )
    assert link.status_code == 201, link.text
    edit = await client.put(
        f"/api/parts/{slug}",
        json={"description_md": "Better words", "change_summary": "Fix"},
        headers=make_auth_header("newbie"),
    )
    assert edit.status_code == 200, edit.text
    pins = await client.put(
        f"/api/parts/{slug}/definition",
        json={"headers": [{"id": "h1", "pins": [{"number": 1, "name": "VIN"}]}]},
        headers=make_auth_header("newbie"),
    )
    assert pins.status_code == 200, pins.text
    model = await client.post(
        f"/api/parts/{slug}/mesh",
        files={"file": ("ubec.stl", b"solid ubec\nendsolid ubec\n", "model/stl")},
        headers=make_auth_header("newbie"),
    )
    assert model.status_code == 200, model.text
    assert model.json()["has_mesh"] is True
    gone = await client.delete(f"/api/parts/{slug}/mesh", headers=make_auth_header("newbie"))
    assert gone.status_code == 200, gone.text


@pytest.mark.asyncio
async def test_new_account_cannot_edit_others_parts(client, monkeypatch) -> None:
    from projects_api import auth as auth_module

    async def _fake_age(username: str, token: str) -> Optional[datetime]:
        days = 1 if username.startswith("new") else 365 * 5
        return datetime.utcnow() - timedelta(days=days)

    monkeypatch.setattr(auth_module, "fetch_account_created_at", _fake_age)
    slug = (await _submit(client, user="newbie")).json()["slug"]
    other = await client.put(
        f"/api/parts/{slug}",
        json={"description_md": "Vandal", "change_summary": "x"},
        headers=make_auth_header("newcomer"),
    )
    assert other.status_code == 403
    photo = await client.post(
        f"/api/parts/{slug}/photos/link",
        json={"external_url": "https://example.com/x.jpg"},
        headers=make_auth_header("newcomer"),
    )
    assert photo.status_code == 403
    model = await client.post(
        f"/api/parts/{slug}/mesh",
        files={"file": ("x.stl", b"solid x\nendsolid x\n", "model/stl")},
        headers=make_auth_header("newcomer"),
    )
    assert model.status_code == 403
    # Once approved, the sender is a new account editing a catalog part: held back too.
    ok = await client.post(f"/api/admin/parts/{slug}/approve", headers=make_auth_header("admin"))
    assert ok.status_code == 200
    again = await client.put(
        f"/api/parts/{slug}",
        json={"description_md": "Later", "change_summary": "x"},
        headers=make_auth_header("newbie"),
    )
    assert again.status_code == 403


@pytest.mark.asyncio
async def test_bad_link_is_refused(client) -> None:
    resp = await _submit(client, {"name": "Thing", "links": [{"url": "javascript:alert(1)"}]})
    assert resp.status_code == 422


@pytest.mark.asyncio
async def test_pending_cap(client, monkeypatch) -> None:
    from projects_api.routers import parts as parts_module

    monkeypatch.setattr(parts_module, "MAX_PENDING_PER_USER", 2)
    for i in range(2):
        assert (await _submit(client, {"name": f"Part {i}"})).status_code == 201
    assert (await _submit(client, {"name": "One more"})).status_code == 429


@pytest.mark.asyncio
async def test_admin_review_queue(client) -> None:
    a = (await _submit(client, {"name": "Part A", "links": [{"url": "https://s.example/a"}]})).json()["slug"]
    b = (await _submit(client, {"name": "Part B"})).json()["slug"]
    denied = await client.get("/api/admin/parts/submitted", headers=make_auth_header("bob"))
    assert denied.status_code == 403
    queue = await client.get("/api/admin/parts/submitted", headers=make_auth_header("admin"))
    assert queue.status_code == 200
    items = queue.json()
    assert [i["slug"] for i in items] == [a, b]
    assert items[0]["links"] == ["https://s.example/a"]
    assert items[0]["created_by"] == "newbie"

    ok = await client.post(f"/api/admin/parts/{a}/approve", headers=make_auth_header("admin"))
    assert ok.status_code == 200 and ok.json()["status"] == "draft"
    no = await client.post(f"/api/admin/parts/{b}/reject", headers=make_auth_header("admin"))
    assert no.status_code == 200 and no.json()["status"] == "rejected"

    # Approved: everyone finds it. Rejected: still hidden, page still opens.
    found = await client.get("/api/parts?q=part", headers=make_auth_header("bob"))
    assert [p["slug"] for p in found.json()] == [a]
    assert (await client.get(f"/api/parts/{b}")).status_code == 200
    rejected = await client.get("/api/admin/parts/submitted?status=rejected", headers=make_auth_header("admin"))
    assert [i["slug"] for i in rejected.json()] == [b]
    # Approving a catalog part again is a conflict.
    again = await client.post(f"/api/admin/parts/{a}/approve", headers=make_auth_header("admin"))
    assert again.status_code == 409
