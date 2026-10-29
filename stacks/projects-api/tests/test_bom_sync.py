"""The stored Bill of Materials follows the Markdown table (issue #264)."""

from __future__ import annotations

from datetime import datetime, timedelta
from typing import Optional

import pytest

from .conftest import make_auth_header


@pytest.fixture(autouse=True)
def _old_account(monkeypatch: pytest.MonkeyPatch) -> None:
    from projects_api import auth as auth_module

    async def _fake_age(username: str, token: str) -> Optional[datetime]:
        return datetime.utcnow() - timedelta(days=365 * 5)

    monkeypatch.setattr(auth_module, "fetch_account_created_at", _fake_age)
    auth_module._clear_account_age_cache()


async def _part(client, name: str, suppliers=None) -> dict:
    part = (await client.post("/api/parts", json={"name": name}, headers=make_auth_header())).json()
    if suppliers:
        part = (await client.put(f"/api/parts/{part['slug']}", json={
            "suppliers": suppliers, "change_summary": "Shops",
        }, headers=make_auth_header())).json()
    return part


async def _project(client) -> int:
    resp = await client.post("/api/projects", json={"title": "Sync bot", "status": "wip"},
                             headers=make_auth_header())
    return resp.json()["id"]


async def _sync(client, project_id, rows, user="testuser"):
    return await client.put(f"/api/projects/{project_id}/bom/sync", json={"rows": rows},
                            headers=make_auth_header(user))


@pytest.mark.asyncio
async def test_sync_creates_updates_and_deletes(client) -> None:
    pico = await _part(client, "Pico")
    pid = await _project(client)
    resp = await _sync(client, pid, [
        {"name": "Pico", "quantity": 1, "part_slug": pico["slug"], "unit_cost": 4.0, "currency_code": "GBP"},
        {"name": "Jumper wires", "quantity": 10},
    ])
    assert resp.status_code == 200, resp.text
    rows = resp.json()
    assert [(r["name"], r["quantity"], r["part_slug"]) for r in rows] == [
        ("Pico", 1, pico["slug"]), ("Jumper wires", 10, None),
    ]
    assert rows[0]["part_revision_id"] == pico["current_revision_id"]
    assert (await client.get(f"/api/parts/{pico['slug']}")).json()["usage_count"] == 1

    # Reordered, a quantity changed, a row gone.
    rows2 = (await _sync(client, pid, [
        {"name": "Jumper wires", "quantity": 20},
    ])).json()
    assert [(r["id"], r["quantity"]) for r in rows2] == [(rows[1]["id"], 20)]
    assert (await client.get(f"/api/parts/{pico['slug']}")).json()["usage_count"] == 0


@pytest.mark.asyncio
async def test_sync_keeps_pin_supplier_and_unpriced_cost(client) -> None:
    pico = await _part(client, "Pico", [
        {"url": "https://thepihut.com/pico", "unit_cost": 3.6, "currency_code": "GBP"},
    ])
    pid = await _project(client)
    first = (await _sync(client, pid, [{"name": "Pico", "part_slug": pico["slug"], "unit_cost": 4.0,
                                        "currency_code": "GBP"}])).json()[0]
    # The author picks a supplier in the old BOM UI, then the part is edited.
    sup_id = pico["suppliers"][0]["id"]
    await client.put(f"/api/projects/{pid}/bom/{first['id']}", json={
        "name": "Pico", "quantity": 1, "part_id": pico["id"], "supplier_id": sup_id,
        "part_revision_id": first["part_revision_id"], "unit_cost": 4.0, "currency_code": "GBP",
    }, headers=make_auth_header())
    await client.put(f"/api/parts/{pico['slug']}", json={"description_md": "New", "change_summary": "x"},
                     headers=make_auth_header())

    # Renamed in the text and no Price column: same row, pin, supplier and price.
    row = (await _sync(client, pid, [{"name": "Raspberry Pi Pico", "quantity": 2,
                                      "part_slug": pico["slug"]}])).json()[0]
    assert row["id"] == first["id"]
    assert row["name"] == "Raspberry Pi Pico" and row["quantity"] == 2
    assert row["supplier_id"] == sup_id
    assert row["part_revision_id"] == first["part_revision_id"]
    assert row["part_revision_outdated"] is True
    assert row["unit_cost"] == 4.0


@pytest.mark.asyncio
async def test_linking_a_named_row_keeps_it(client) -> None:
    pico = await _part(client, "Pico")
    other = await _part(client, "Pico W")
    pid = await _project(client)
    plain = (await _sync(client, pid, [{"name": "Pico"}])).json()[0]
    linked = (await _sync(client, pid, [{"name": "Pico", "part_slug": pico["slug"]}])).json()[0]
    assert linked["id"] == plain["id"]
    assert linked["part_slug"] == pico["slug"]

    # Linked to another part: re-pinned to that one.
    moved = (await _sync(client, pid, [{"name": "Pico", "part_slug": other["slug"]}])).json()[0]
    assert moved["id"] == plain["id"]
    assert moved["part_revision_id"] == other["current_revision_id"]

    # An unknown slug is kept as an unlinked row.
    row = (await _sync(client, pid, [{"name": "Pico", "part_slug": "no-such-part"}])).json()[0]
    assert row["part_id"] is None and row["part_revision_id"] is None


@pytest.mark.asyncio
async def test_sync_is_owner_only(client) -> None:
    pid = await _project(client)
    assert (await _sync(client, pid, [], user="stranger")).status_code == 403
    assert (await client.put(f"/api/projects/{pid}/bom/sync", json={"rows": []})).status_code == 401
