"""Tests for supplier country_code on parts (issue #149)."""

from __future__ import annotations

from datetime import datetime, timedelta
from typing import Optional

import pytest

from .conftest import make_auth_header


@pytest.fixture(autouse=True)
def _old_account(monkeypatch: pytest.MonkeyPatch) -> None:
    """Pretend every account is 5 years old so the age gate passes."""
    from projects_api import auth as auth_module

    async def _fake_age(username: str, token: str) -> Optional[datetime]:
        return datetime.utcnow() - timedelta(days=365 * 5)

    monkeypatch.setattr(auth_module, "fetch_account_created_at", _fake_age)
    auth_module._clear_account_age_cache()


async def _create_part_with_supplier(client) -> dict:
    headers = make_auth_header()
    resp = await client.post(
        "/api/parts",
        json={
            "name": "Country Test Servo",
            "supplier_url": "https://thepihut.com/things",
            "supplier_name": "ThePiHut",
        },
        headers=headers,
    )
    assert resp.status_code == 201, resp.text
    return resp.json()


@pytest.mark.asyncio
async def test_supplier_country_code_round_trips(client) -> None:
    """PUT a supplier with country_code, then GET it back."""
    headers = make_auth_header()
    part = await _create_part_with_supplier(client)
    slug = part["slug"]

    # Edit: add country_code to the supplier.
    resp = await client.put(
        f"/api/parts/{slug}",
        json={
            "suppliers": [
                {
                    "name": "Pimoroni",
                    "url": "https://shop.pimoroni.com/products/servo",
                    "country_code": "GB",
                },
                {
                    "name": "Adafruit",
                    "url": "https://www.adafruit.com/product/169",
                    "country_code": "US",
                },
            ],
            "change_summary": "Added country tags to suppliers",
        },
        headers=headers,
    )
    assert resp.status_code == 200, resp.text
    data = resp.json()
    assert len(data["suppliers"]) == 2
    by_name = {s["name"]: s for s in data["suppliers"]}
    assert by_name["Pimoroni"]["country_code"] == "GB"
    assert by_name["Adafruit"]["country_code"] == "US"

    # GET should also include the codes.
    resp = await client.get(f"/api/parts/{slug}")
    assert resp.status_code == 200
    data = resp.json()
    by_name = {s["name"]: s for s in data["suppliers"]}
    assert by_name["Pimoroni"]["country_code"] == "GB"
    assert by_name["Adafruit"]["country_code"] == "US"


@pytest.mark.asyncio
async def test_legacy_supplier_without_country_code_serializes(client) -> None:
    """A part created via the legacy create-path (no country) returns
    ``country_code: null`` rather than 500ing or omitting the key."""
    part = await _create_part_with_supplier(client)
    resp = await client.get(f"/api/parts/{part['slug']}")
    assert resp.status_code == 200
    suppliers = resp.json()["suppliers"]
    assert len(suppliers) == 1
    # Field is always present even when value is null.
    assert "country_code" in suppliers[0]
    assert suppliers[0]["country_code"] is None


@pytest.mark.asyncio
@pytest.mark.parametrize("bad_code", ["gb", "GBR", "GB-X", "g1", "1A"])
async def test_invalid_country_code_rejected_422(client, bad_code: str) -> None:
    headers = make_auth_header()
    part = await _create_part_with_supplier(client)
    resp = await client.put(
        f"/api/parts/{part['slug']}",
        json={
            "suppliers": [
                {
                    "name": "Pimoroni",
                    "url": "https://shop.pimoroni.com/x",
                    "country_code": bad_code,
                }
            ],
            "change_summary": "Bad country code",
        },
        headers=headers,
    )
    assert resp.status_code == 422, resp.text


@pytest.mark.asyncio
async def test_country_code_null_clears_value(client) -> None:
    """Submitting ``country_code: null`` (or omitting it) keeps the field
    null in the response — the "Other / unknown" option in the dropdown."""
    headers = make_auth_header()
    part = await _create_part_with_supplier(client)
    slug = part["slug"]

    # First set a code so we know clearing actually does something.
    resp = await client.put(
        f"/api/parts/{slug}",
        json={
            "suppliers": [
                {
                    "name": "Pimoroni",
                    "url": "https://shop.pimoroni.com/x",
                    "country_code": "GB",
                }
            ],
            "change_summary": "Add GB",
        },
        headers=headers,
    )
    assert resp.status_code == 200
    assert resp.json()["suppliers"][0]["country_code"] == "GB"

    # Now clear it by omitting country_code.
    resp = await client.put(
        f"/api/parts/{slug}",
        json={
            "suppliers": [
                {
                    "name": "Pimoroni",
                    "url": "https://shop.pimoroni.com/x",
                }
            ],
            "change_summary": "Clear country",
        },
        headers=headers,
    )
    assert resp.status_code == 200
    assert resp.json()["suppliers"][0]["country_code"] is None


@pytest.mark.asyncio
async def test_revision_snapshot_records_country_code(client) -> None:
    """The revision snapshot (suppliers_json) should retain country_code
    so the restore-revision endpoint can rebuild the supplier row."""
    headers = make_auth_header()
    part = await _create_part_with_supplier(client)
    slug = part["slug"]

    resp = await client.put(
        f"/api/parts/{slug}",
        json={
            "suppliers": [
                {
                    "name": "Pimoroni",
                    "url": "https://shop.pimoroni.com/x",
                    "country_code": "GB",
                }
            ],
            "change_summary": "Add GB",
        },
        headers=headers,
    )
    assert resp.status_code == 200
    revs = resp.json()["recent_revisions"]
    rev_id = revs[0]["id"]

    rev = await client.get(f"/api/parts/{slug}/revisions/{rev_id}")
    assert rev.status_code == 200
    rev_data = rev.json()
    assert len(rev_data["suppliers"]) == 1
    assert rev_data["suppliers"][0]["country_code"] == "GB"


@pytest.mark.asyncio
async def test_add_supplier_country_helper_is_noop_on_sqlite() -> None:
    """The migration helper short-circuits on non-Postgres dialects so
    it can be called on every startup without harm."""
    from projects_api.db import add_supplier_country_if_missing
    # Calling on the in-memory SQLite test DB must not raise.
    await add_supplier_country_if_missing()
