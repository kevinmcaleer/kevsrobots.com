"""Tests for the supplier-pricing feature.

Moves price + currency onto ``PartSupplier`` so a BOM row can link to a
specific supplier via ``ProjectBOMItem.supplier_id`` and read the price
live. Editing the supplier price auto-updates every linked BOM row's
``effective_unit_cost`` on the next read — no per-row mutation needed.

Covers:
  * Supplier create + update round-trips with the new ``unit_cost`` +
    ``currency_code`` fields.
  * Revision-snapshot audit trail picks up pure price edits.
  * BOM ``supplier_id`` resolution + fallback rules.
  * Auto-update behaviour after a supplier price change.
  * Stale ``supplier_id`` (supplier deleted via ``ON DELETE SET NULL``)
    silently falling back to row values.
  * Auth + 14-day account-age gate on supplier edits.
"""

from __future__ import annotations

from datetime import datetime, timedelta
from typing import Optional

import pytest

from .conftest import make_auth_header


@pytest.fixture(autouse=True)
def _old_account(monkeypatch: pytest.MonkeyPatch) -> None:
    """Pretend every account is 5 years old so the age gate passes by
    default. Individual tests can override by reaching into the auth
    module and substituting a "young account" stub.
    """
    from projects_api import auth as auth_module

    async def _fake_age(username: str, token: str) -> Optional[datetime]:
        return datetime.utcnow() - timedelta(days=365 * 5)

    monkeypatch.setattr(auth_module, "fetch_account_created_at", _fake_age)
    auth_module._clear_account_age_cache()


# --- helpers ------------------------------------------------------------


async def _create_part(client) -> dict:
    headers = make_auth_header()
    resp = await client.post(
        "/api/parts",
        json={
            "name": "Pricing Test Servo",
            "supplier_url": "https://thepihut.com/things",
            "supplier_name": "ThePiHut",
        },
        headers=headers,
    )
    assert resp.status_code == 201, resp.text
    return resp.json()


async def _set_suppliers(client, slug: str, suppliers: list[dict]) -> dict:
    headers = make_auth_header()
    resp = await client.put(
        f"/api/parts/{slug}",
        json={"suppliers": suppliers, "change_summary": "Set suppliers"},
        headers=headers,
    )
    assert resp.status_code == 200, resp.text
    return resp.json()


async def _create_project(client, title: str = "Pricing Project") -> int:
    headers = make_auth_header()
    resp = await client.post(
        "/api/projects", json={"title": title}, headers=headers
    )
    assert resp.status_code == 201, resp.text
    return resp.json()["id"]


# --- supplier round-trips ----------------------------------------------


@pytest.mark.asyncio
async def test_supplier_price_round_trips(client) -> None:
    """PUT a supplier with unit_cost + currency_code, GET it back."""
    part = await _create_part(client)
    slug = part["slug"]
    data = await _set_suppliers(
        client,
        slug,
        [
            {
                "name": "Pimoroni",
                "url": "https://shop.pimoroni.com/products/servo",
                "country_code": "GB",
                "unit_cost": 12.5,
                "currency_code": "GBP",
            },
            {
                "name": "Adafruit",
                "url": "https://www.adafruit.com/product/169",
                "country_code": "US",
                "unit_cost": 14.99,
                "currency_code": "USD",
            },
        ],
    )
    by_name = {s["name"]: s for s in data["suppliers"]}
    assert by_name["Pimoroni"]["unit_cost"] == 12.5
    assert by_name["Pimoroni"]["currency_code"] == "GBP"
    assert by_name["Adafruit"]["unit_cost"] == 14.99
    assert by_name["Adafruit"]["currency_code"] == "USD"

    # GET round-trips them too.
    resp = await client.get(f"/api/parts/{slug}")
    assert resp.status_code == 200
    by_name = {s["name"]: s for s in resp.json()["suppliers"]}
    assert by_name["Pimoroni"]["unit_cost"] == 12.5
    assert by_name["Pimoroni"]["currency_code"] == "GBP"


@pytest.mark.asyncio
async def test_supplier_without_price_serializes_nulls(client) -> None:
    """A supplier saved without a price should round-trip as null —
    not omit the keys, not 500."""
    part = await _create_part(client)
    resp = await client.get(f"/api/parts/{part['slug']}")
    suppliers = resp.json()["suppliers"]
    assert len(suppliers) == 1
    assert "unit_cost" in suppliers[0]
    assert "currency_code" in suppliers[0]
    assert suppliers[0]["unit_cost"] is None
    assert suppliers[0]["currency_code"] is None


@pytest.mark.asyncio
@pytest.mark.parametrize("bad_currency", ["gbp", "GB", "XYZ123", "G1P"])
async def test_supplier_bad_currency_rejected_422(
    client, bad_currency: str
) -> None:
    headers = make_auth_header()
    part = await _create_part(client)
    resp = await client.put(
        f"/api/parts/{part['slug']}",
        json={
            "suppliers": [
                {
                    "name": "Pimoroni",
                    "url": "https://shop.pimoroni.com/x",
                    "unit_cost": 10.0,
                    "currency_code": bad_currency,
                }
            ],
            "change_summary": "Bad currency",
        },
        headers=headers,
    )
    assert resp.status_code == 422, resp.text


@pytest.mark.asyncio
async def test_supplier_negative_price_rejected_422(client) -> None:
    headers = make_auth_header()
    part = await _create_part(client)
    resp = await client.put(
        f"/api/parts/{part['slug']}",
        json={
            "suppliers": [
                {
                    "name": "Pimoroni",
                    "url": "https://shop.pimoroni.com/x",
                    "unit_cost": -1.0,
                    "currency_code": "GBP",
                }
            ],
            "change_summary": "Bad price",
        },
        headers=headers,
    )
    assert resp.status_code == 422, resp.text


# --- revision audit trail ----------------------------------------------


@pytest.mark.asyncio
async def test_price_edit_records_revision(client) -> None:
    """Editing only the price (no other field touched) must still write
    a new revision whose ``suppliers_json`` snapshot includes the new
    ``unit_cost`` + ``currency_code``.
    """
    part = await _create_part(client)
    slug = part["slug"]
    # First, set a price so we have something to change.
    first = await _set_suppliers(
        client,
        slug,
        [
            {
                "name": "Pimoroni",
                "url": "https://shop.pimoroni.com/x",
                "unit_cost": 10.0,
                "currency_code": "GBP",
            }
        ],
    )
    rev_count_before = len(first["recent_revisions"])

    # Now change ONLY the price.
    second = await _set_suppliers(
        client,
        slug,
        [
            {
                "name": "Pimoroni",
                "url": "https://shop.pimoroni.com/x",
                "unit_cost": 15.0,
                "currency_code": "GBP",
            }
        ],
    )
    rev_count_after = len(second["recent_revisions"])
    assert rev_count_after == rev_count_before + 1, (
        "Pure price edit should create a revision"
    )

    # The newest revision's snapshot should include the new price.
    rev_id = second["recent_revisions"][0]["id"]
    rev = await client.get(f"/api/parts/{slug}/revisions/{rev_id}")
    assert rev.status_code == 200
    rev_data = rev.json()
    assert len(rev_data["suppliers"]) == 1
    assert rev_data["suppliers"][0]["unit_cost"] == 15.0
    assert rev_data["suppliers"][0]["currency_code"] == "GBP"


# --- BOM resolution rules -----------------------------------------------


@pytest.mark.asyncio
async def test_bom_supplier_id_resolves_to_supplier_price(client) -> None:
    """A BOM row linked via ``supplier_id`` returns the supplier's
    price in ``effective_unit_cost`` / ``effective_currency_code`` and
    ``price_source = "supplier"``."""
    part = await _create_part(client)
    data = await _set_suppliers(
        client,
        part["slug"],
        [
            {
                "name": "Pimoroni",
                "url": "https://shop.pimoroni.com/x",
                "unit_cost": 12.5,
                "currency_code": "GBP",
            }
        ],
    )
    supplier_id = data["suppliers"][0]["id"]
    project_id = await _create_project(client)
    headers = make_auth_header()
    resp = await client.post(
        f"/api/projects/{project_id}/bom",
        json={
            "name": "Linked Servo",
            "quantity": 2,
            "unit_cost": 999.99,  # Row value should be ignored.
            "currency_code": "USD",  # Row currency should be ignored.
            "part_id": data["id"],
            "supplier_id": supplier_id,
        },
        headers=headers,
    )
    assert resp.status_code == 201, resp.text
    row = resp.json()
    assert row["supplier_id"] == supplier_id
    assert row["effective_unit_cost"] == 12.5
    assert row["effective_currency_code"] == "GBP"
    assert row["price_source"] == "supplier"
    # The row's own values are still echoed back for reference.
    assert row["unit_cost"] == 999.99
    assert row["currency_code"] == "USD"


@pytest.mark.asyncio
async def test_bom_supplier_id_with_no_price_falls_back_to_row(client) -> None:
    """A supplier without a price recorded should not override the
    row's own values — ``price_source`` should be ``"row"``."""
    part = await _create_part(client)
    data = await _set_suppliers(
        client,
        part["slug"],
        [
            {
                "name": "Pimoroni",
                "url": "https://shop.pimoroni.com/x",
                # No unit_cost / currency_code.
            }
        ],
    )
    supplier_id = data["suppliers"][0]["id"]
    project_id = await _create_project(client)
    headers = make_auth_header()
    resp = await client.post(
        f"/api/projects/{project_id}/bom",
        json={
            "name": "Linked Servo",
            "quantity": 1,
            "unit_cost": 7.5,
            "currency_code": "GBP",
            "supplier_id": supplier_id,
        },
        headers=headers,
    )
    assert resp.status_code == 201, resp.text
    row = resp.json()
    assert row["supplier_id"] == supplier_id
    assert row["effective_unit_cost"] == 7.5
    assert row["effective_currency_code"] == "GBP"
    assert row["price_source"] == "row"


@pytest.mark.asyncio
async def test_bom_without_supplier_id_uses_row_values(client) -> None:
    """Regression: BOM rows that pre-date this feature (no
    ``supplier_id`` at all) still render their own price with
    ``price_source = "row"``."""
    project_id = await _create_project(client)
    headers = make_auth_header()
    resp = await client.post(
        f"/api/projects/{project_id}/bom",
        json={
            "name": "Manual row",
            "quantity": 3,
            "unit_cost": 4.25,
            "currency_code": "EUR",
        },
        headers=headers,
    )
    assert resp.status_code == 201, resp.text
    row = resp.json()
    assert row["supplier_id"] is None
    assert row["effective_unit_cost"] == 4.25
    assert row["effective_currency_code"] == "EUR"
    assert row["price_source"] == "row"


# --- auto-update propagation --------------------------------------------


@pytest.mark.asyncio
async def test_editing_supplier_price_propagates_to_linked_bom(client) -> None:
    """Edit the supplier's price; every BOM row linked to that supplier
    sees the new price on the next GET — no per-row mutation needed."""
    part = await _create_part(client)
    data = await _set_suppliers(
        client,
        part["slug"],
        [
            {
                "name": "Pimoroni",
                "url": "https://shop.pimoroni.com/x",
                "unit_cost": 12.5,
                "currency_code": "GBP",
            }
        ],
    )
    supplier_id = data["suppliers"][0]["id"]
    project_id = await _create_project(client)
    headers = make_auth_header()
    create = await client.post(
        f"/api/projects/{project_id}/bom",
        json={
            "name": "Linked Servo",
            "quantity": 1,
            "supplier_id": supplier_id,
        },
        headers=headers,
    )
    assert create.status_code == 201
    item_id = create.json()["id"]

    # Bump the supplier's price. No touch on the BOM row.
    await _set_suppliers(
        client,
        part["slug"],
        [
            {
                "name": "Pimoroni",
                "url": "https://shop.pimoroni.com/x",
                "unit_cost": 99.0,
                "currency_code": "GBP",
            }
        ],
    )

    # The BOM list should reflect the new price.
    bom = await client.get(f"/api/projects/{project_id}/bom")
    assert bom.status_code == 200
    rows = bom.json()
    assert len(rows) == 1
    assert rows[0]["id"] == item_id
    assert rows[0]["effective_unit_cost"] == 99.0
    assert rows[0]["price_source"] == "supplier"


# --- stale supplier_id (ON DELETE SET NULL fallback) --------------------


@pytest.mark.asyncio
async def test_stale_supplier_id_silently_falls_back(client) -> None:
    """A BOM row holding a supplier_id that doesn't exist (e.g. because
    the supplier was deleted under us and ON DELETE SET NULL hasn't
    been enforced — or, equivalently, a value the client typed that
    points at no row) must silently fall back to the row's own price.
    No 500, no exception, no leaking the stale id into the
    effective_* fields.

    We exercise this via the validate-or-drop path on POST: a
    supplier_id of 999_999 (no row exists) is dropped and the
    resolver sees supplier_id=None — the same shape the FK SET NULL
    cascade would produce on Postgres.
    """
    project_id = await _create_project(client)
    headers = make_auth_header()
    resp = await client.post(
        f"/api/projects/{project_id}/bom",
        json={
            "name": "Stale-linked",
            "quantity": 1,
            "unit_cost": 3.0,
            "currency_code": "GBP",
            "supplier_id": 999_999,  # No such supplier.
        },
        headers=headers,
    )
    assert resp.status_code == 201, resp.text
    row = resp.json()
    assert row["supplier_id"] is None
    assert row["effective_unit_cost"] == 3.0
    assert row["effective_currency_code"] == "GBP"
    assert row["price_source"] == "row"

    bom = await client.get(f"/api/projects/{project_id}/bom")
    assert bom.status_code == 200
    rows = bom.json()
    assert len(rows) == 1
    assert rows[0]["effective_unit_cost"] == 3.0
    assert rows[0]["price_source"] == "row"


@pytest.mark.asyncio
async def test_resolver_handles_truly_stale_supplier_id(
    sessionmaker_, client
) -> None:
    """Defence-in-depth: even if a stale supplier_id leaks past the
    POST validator (e.g. because the supplier was deleted AFTER the
    BOM row was saved and the FK SET NULL hasn't fired), the
    ``_to_response`` resolver must not crash and must report
    ``price_source = "row"``.

    We force the stale state by writing supplier_id directly on the
    ProjectBOMItem via the session — bypassing the router's validator
    — and then deleting the supplier underneath it.
    """
    from projects_api.models import (
        Part, PartSupplier, Project, ProjectBOMItem,
    )

    part = await _create_part(client)
    data = await _set_suppliers(
        client,
        part["slug"],
        [
            {
                "name": "Pimoroni",
                "url": "https://shop.pimoroni.com/x",
                "unit_cost": 12.5,
                "currency_code": "GBP",
            }
        ],
    )
    supplier_id = data["suppliers"][0]["id"]
    project_id = await _create_project(client)
    headers = make_auth_header()
    create = await client.post(
        f"/api/projects/{project_id}/bom",
        json={
            "name": "Linked",
            "quantity": 1,
            "unit_cost": 4.0,
            "currency_code": "GBP",
            "supplier_id": supplier_id,
        },
        headers=headers,
    )
    assert create.status_code == 201
    item_id = create.json()["id"]

    # Now delete the supplier directly (bypass the router so we keep
    # the BOM row's FK pointing at it — simulating the race window
    # before ON DELETE SET NULL fires on Postgres, or any other path
    # that leaves a dangling id).
    from sqlalchemy import delete as sql_delete

    async with sessionmaker_() as s:
        await s.execute(
            sql_delete(PartSupplier).where(PartSupplier.id == supplier_id)
        )
        await s.commit()

    bom = await client.get(f"/api/projects/{project_id}/bom")
    assert bom.status_code == 200
    rows = bom.json()
    assert len(rows) == 1
    assert rows[0]["id"] == item_id
    # supplier_id may still be the stale value on SQLite (no FK
    # enforcement) — what matters is that the resolver falls back.
    assert rows[0]["effective_unit_cost"] == 4.0
    assert rows[0]["effective_currency_code"] == "GBP"
    assert rows[0]["price_source"] == "row"


@pytest.mark.asyncio
async def test_unknown_supplier_id_in_post_silently_dropped(client) -> None:
    """Posting a BOM row with a supplier_id that doesn't exist should
    NOT 400 — drop the FK and save the rest of the row (matches the
    ``part_id`` validate-or-drop pattern)."""
    project_id = await _create_project(client)
    headers = make_auth_header()
    resp = await client.post(
        f"/api/projects/{project_id}/bom",
        json={
            "name": "Ghost-linked row",
            "quantity": 1,
            "unit_cost": 1.0,
            "currency_code": "GBP",
            "supplier_id": 999_999,  # Does not exist.
        },
        headers=headers,
    )
    assert resp.status_code == 201, resp.text
    row = resp.json()
    assert row["supplier_id"] is None
    assert row["effective_unit_cost"] == 1.0
    assert row["price_source"] == "row"


# --- auth + age gate ----------------------------------------------------


@pytest.mark.asyncio
async def test_supplier_edit_enforces_age_gate(
    client, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A young account (account < 14 days old) cannot edit suppliers,
    just like every other parts wiki edit."""
    part = await _create_part(client)
    slug = part["slug"]

    # Now flip the account-age stub to "3 days old" for this test only.
    from projects_api import auth as auth_module

    async def _young(username: str, token: str) -> Optional[datetime]:
        return datetime.utcnow() - timedelta(days=3)

    monkeypatch.setattr(auth_module, "fetch_account_created_at", _young)
    auth_module._clear_account_age_cache()

    headers = make_auth_header()
    resp = await client.put(
        f"/api/parts/{slug}",
        json={
            "suppliers": [
                {
                    "name": "Pimoroni",
                    "url": "https://shop.pimoroni.com/x",
                    "unit_cost": 5.0,
                    "currency_code": "GBP",
                }
            ],
            "change_summary": "Try to edit",
        },
        headers=headers,
    )
    assert resp.status_code == 403, resp.text


@pytest.mark.asyncio
async def test_supplier_edit_requires_auth(client) -> None:
    part = await _create_part(client)
    # No auth header.
    resp = await client.put(
        f"/api/parts/{part['slug']}",
        json={
            "suppliers": [
                {
                    "name": "Pimoroni",
                    "url": "https://shop.pimoroni.com/x",
                    "unit_cost": 5.0,
                    "currency_code": "GBP",
                }
            ],
            "change_summary": "No auth",
        },
    )
    assert resp.status_code == 401, resp.text


# --- migration helpers no-op on SQLite ----------------------------------


@pytest.mark.asyncio
async def test_supplier_pricing_helper_is_noop_on_sqlite() -> None:
    """The migration helper short-circuits on non-Postgres dialects so
    it can be called on every startup without harm."""
    from projects_api.db import add_supplier_pricing_if_missing

    await add_supplier_pricing_if_missing()


@pytest.mark.asyncio
async def test_bom_supplier_id_helper_is_noop_on_sqlite() -> None:
    """Same for the BOM FK helper — must be a no-op on SQLite."""
    from projects_api.db import add_bom_supplier_id_if_missing

    await add_bom_supplier_id_if_missing()
