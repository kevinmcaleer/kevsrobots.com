"""Supplier price history, availability, link kinds and the BOM price
fallback (issues #202 and #267)."""

from __future__ import annotations

from datetime import datetime, timedelta
from typing import Optional

import httpx
import pytest
from sqlalchemy import select

from projects_api.models import PartPriceSnapshot, PartSupplier
from projects_api.supplier_health import run_health_check_for_suppliers
from .conftest import make_auth_header

SHOP_A = "https://thepihut.com/pico"
SHOP_B = "https://shop.pimoroni.com/pico"


@pytest.fixture(autouse=True)
def _old_account(monkeypatch: pytest.MonkeyPatch) -> None:
    from projects_api import auth as auth_module

    async def _fake_age(username: str, token: str) -> Optional[datetime]:
        return datetime.utcnow() - timedelta(days=365 * 5)

    monkeypatch.setattr(auth_module, "fetch_account_created_at", _fake_age)
    auth_module._clear_account_age_cache()


async def _part(client, name="History Pico") -> dict:
    resp = await client.post("/api/parts", json={"name": name}, headers=make_auth_header())
    assert resp.status_code == 201, resp.text
    return resp.json()


async def _suppliers(client, slug, suppliers, summary="Prices") -> dict:
    resp = await client.put(f"/api/parts/{slug}", json={"suppliers": suppliers, "change_summary": summary},
                            headers=make_auth_header())
    assert resp.status_code == 200, resp.text
    return resp.json()


@pytest.mark.asyncio
async def test_price_changes_make_snapshots_and_keep_supplier_ids(client) -> None:
    part = await _part(client)
    slug = part["slug"]
    body = await _suppliers(client, slug, [{"name": "ThePiHut", "url": SHOP_A, "unit_cost": 4.0, "currency_code": "GBP"}])
    sup_id = body["suppliers"][0]["id"]

    # Same price, other field changed: no new snapshot, same supplier row.
    body = await _suppliers(client, slug, [{"name": "The Pi Hut", "url": SHOP_A, "unit_cost": 4.0, "currency_code": "GBP"}])
    assert body["suppliers"][0]["id"] == sup_id
    body = await _suppliers(client, slug, [
        {"name": "The Pi Hut", "url": SHOP_A, "unit_cost": 4.2, "currency_code": "GBP"},
        {"name": "Pimoroni", "url": SHOP_B, "unit_cost": 3.9, "currency_code": "GBP"},
    ])
    assert body["suppliers"][0]["id"] == sup_id
    assert body["suppliers"][0]["last_price"] == 4.2

    history = (await client.get(f"/api/parts/{slug}/price-history")).json()
    assert [h["name"] for h in history] == ["The Pi Hut", "Pimoroni"]
    assert [p["unit_cost"] for p in history[0]["points"]] == [4.0, 4.2]
    assert [p["unit_cost"] for p in history[1]["points"]] == [3.9]


@pytest.mark.asyncio
async def test_restore_keeps_history(client) -> None:
    part = await _part(client)
    slug = part["slug"]
    first = await _suppliers(client, slug, [{"url": SHOP_A, "unit_cost": 4.0, "currency_code": "GBP"}])
    await _suppliers(client, slug, [{"url": SHOP_A, "unit_cost": 5.0, "currency_code": "GBP"}])
    resp = await client.post(f"/api/parts/{slug}/revisions/{first['current_revision_id']}/restore",
                             headers=make_auth_header())
    assert resp.status_code == 200
    points = (await client.get(f"/api/parts/{slug}/price-history")).json()[0]["points"]
    assert [p["unit_cost"] for p in points] == [4.0, 5.0, 4.0]


@pytest.mark.asyncio
async def test_daily_check_snapshots_untracked_prices(client, session) -> None:
    part = await _part(client)
    await _suppliers(client, part["slug"], [{"url": SHOP_A, "unit_cost": 4.0, "currency_code": "GBP"}])
    sup = await session.scalar(select(PartSupplier).where(PartSupplier.url == SHOP_A))
    # A price changed behind the API's back (say, by an older deploy).
    sup.unit_cost = 3.5
    await session.commit()
    transport = httpx.MockTransport(lambda request: httpx.Response(200))
    async with httpx.AsyncClient(transport=transport) as http:
        await run_health_check_for_suppliers(session, [sup], client=http)
        await run_health_check_for_suppliers(session, [sup], client=http)
    snaps = (await session.scalars(select(PartPriceSnapshot).where(PartPriceSnapshot.supplier_id == sup.id))).all()
    assert [s.unit_cost for s in snaps] == [4.0, 3.5]


@pytest.mark.asyncio
async def test_mark_and_clear_unavailable(client) -> None:
    part = await _part(client)
    slug = part["slug"]
    body = await _suppliers(client, slug, [{"url": SHOP_A, "unit_cost": 4.0, "currency_code": "GBP"}])
    sup_id = body["suppliers"][0]["id"]
    url = f"/api/parts/{slug}/suppliers/{sup_id}/unavailable"

    assert (await client.post(url, json={})).status_code == 401
    resp = await client.post(url, json={"note": "Out of stock"}, headers=make_auth_header("reporter"))
    assert resp.status_code == 200
    sup = resp.json()["suppliers"][0]
    assert sup["unavailable_by"] == "reporter" and sup["unavailable_note"] == "Out of stock"

    assert (await client.delete(url, headers=make_auth_header("stranger"))).status_code == 403
    # The part's creator (testuser) may clear it.
    resp = await client.delete(url, headers=make_auth_header())
    assert resp.status_code == 200
    assert resp.json()["suppliers"][0]["unavailable_at"] is None


@pytest.mark.asyncio
async def test_link_kinds(client) -> None:
    part = await _part(client)
    body = await _suppliers(client, part["slug"], [
        {"url": SHOP_A, "unit_cost": 4.0, "currency_code": "GBP"},
        {"name": "Datasheet", "url": "https://datasheets.raspberrypi.com/pico.pdf", "kind": "datasheet"},
        {"name": "MicroPython", "url": "https://micropython.org/download/RPI_PICO/", "kind": "driver"},
    ])
    assert [s["kind"] for s in body["suppliers"]] == ["shop", "datasheet", "driver"]
    # Only shops have a price history.
    assert len((await client.get(f"/api/parts/{part['slug']}/price-history")).json()) == 1
    resp = await client.put(f"/api/parts/{part['slug']}", json={
        "suppliers": [{"url": SHOP_A, "kind": "warehouse"}], "change_summary": "x",
    }, headers=make_auth_header())
    assert resp.status_code == 422


async def _bom_row(client, project_id, part, **extra) -> dict:
    resp = await client.post(f"/api/projects/{project_id}/bom", json={
        "name": part["name"], "quantity": 2, "part_id": part["id"], **extra,
    }, headers=make_auth_header())
    assert resp.status_code == 201, resp.text
    return resp.json()


@pytest.mark.asyncio
async def test_bom_falls_back_to_cheapest_available(client) -> None:
    part = await _part(client)
    body = await _suppliers(client, part["slug"], [
        {"url": SHOP_A, "unit_cost": 4.2, "currency_code": "GBP"},
        {"url": SHOP_B, "unit_cost": 3.9, "currency_code": "GBP"},
    ])
    a_id, b_id = (s["id"] for s in body["suppliers"])
    project_id = (await client.post("/api/projects", json={"title": "BOM fallback", "status": "wip"},
                                    headers=make_auth_header())).json()["id"]

    # No price on the row and no chosen supplier: cheapest available.
    row = await _bom_row(client, project_id, part)
    assert (row["effective_unit_cost"], row["price_source"]) == (3.9, "cheapest")

    # A chosen supplier wins while it's available...
    chosen = await _bom_row(client, project_id, part, supplier_id=a_id)
    assert (chosen["effective_unit_cost"], chosen["price_source"]) == (4.2, "supplier")

    # ...and gives way to the cheapest available shop once it isn't.
    await client.post(f"/api/parts/{part['slug']}/suppliers/{a_id}/unavailable", json={},
                      headers=make_auth_header())
    rows = {r["id"]: r for r in (await client.get(f"/api/projects/{project_id}/bom")).json()}
    assert (rows[chosen["id"]]["effective_unit_cost"], rows[chosen["id"]]["price_source"]) == (3.9, "cheapest")
    assert rows[chosen["id"]]["suppliers_unavailable"] is False

    # Every shop unavailable: warn, and the row keeps no price.
    await client.post(f"/api/parts/{part['slug']}/suppliers/{b_id}/unavailable", json={},
                      headers=make_auth_header())
    rows = {r["id"]: r for r in (await client.get(f"/api/projects/{project_id}/bom")).json()}
    assert rows[row["id"]]["suppliers_unavailable"] is True
    assert rows[row["id"]]["effective_unit_cost"] is None
