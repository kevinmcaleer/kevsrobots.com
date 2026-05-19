"""Tests for BOM currency_code (issue #149)."""

from __future__ import annotations

import pytest

from .conftest import make_auth_header


@pytest.fixture
async def project_id(client) -> int:
    headers = make_auth_header()
    resp = await client.post(
        "/api/projects",
        json={"title": "BOM Currency Test"},
        headers=headers,
    )
    return resp.json()["id"]


@pytest.mark.asyncio
async def test_bom_currency_round_trips(client, project_id) -> None:
    headers = make_auth_header()
    resp = await client.post(
        f"/api/projects/{project_id}/bom",
        json={
            "name": "STS3215 Servo",
            "quantity": 2,
            "unit_cost": 12.50,
            "currency_code": "GBP",
        },
        headers=headers,
    )
    assert resp.status_code == 201, resp.text
    data = resp.json()
    assert data["currency_code"] == "GBP"
    assert data["unit_cost"] == 12.50
    item_id = data["id"]

    # GET via list
    resp = await client.get(f"/api/projects/{project_id}/bom")
    rows = resp.json()
    assert len(rows) == 1
    assert rows[0]["currency_code"] == "GBP"

    # PUT changes the currency
    resp = await client.put(
        f"/api/projects/{project_id}/bom/{item_id}",
        json={
            "name": "STS3215 Servo",
            "quantity": 2,
            "unit_cost": 14.99,
            "currency_code": "EUR",
        },
        headers=headers,
    )
    assert resp.status_code == 200, resp.text
    assert resp.json()["currency_code"] == "EUR"


@pytest.mark.asyncio
async def test_bom_legacy_row_without_currency_serializes(
    client, project_id
) -> None:
    """An item posted without currency_code returns currency_code=null."""
    headers = make_auth_header()
    resp = await client.post(
        f"/api/projects/{project_id}/bom",
        json={"name": "Mystery Widget", "quantity": 1, "unit_cost": 5.00},
        headers=headers,
    )
    assert resp.status_code == 201, resp.text
    data = resp.json()
    assert "currency_code" in data
    assert data["currency_code"] is None

    # GET list also exposes the field as null.
    resp = await client.get(f"/api/projects/{project_id}/bom")
    rows = resp.json()
    assert "currency_code" in rows[0]
    assert rows[0]["currency_code"] is None


@pytest.mark.asyncio
@pytest.mark.parametrize("bad_code", ["gbp", "GB", "XYZ123", "G1P", "12"])
async def test_invalid_currency_code_rejected_422(
    client, project_id, bad_code: str
) -> None:
    headers = make_auth_header()
    resp = await client.post(
        f"/api/projects/{project_id}/bom",
        json={
            "name": "Bad Currency",
            "quantity": 1,
            "unit_cost": 10.0,
            "currency_code": bad_code,
        },
        headers=headers,
    )
    assert resp.status_code == 422, resp.text


@pytest.mark.asyncio
async def test_bom_currency_without_unit_cost_allowed(
    client, project_id
) -> None:
    """A currency_code without a unit_cost is still allowed (the editor
    might set the currency before filling in the price)."""
    headers = make_auth_header()
    resp = await client.post(
        f"/api/projects/{project_id}/bom",
        json={
            "name": "Currency only",
            "quantity": 1,
            "currency_code": "JPY",
        },
        headers=headers,
    )
    assert resp.status_code == 201
    data = resp.json()
    assert data["currency_code"] == "JPY"
    assert data["unit_cost"] is None


@pytest.mark.asyncio
async def test_unit_cost_without_currency_allowed(client, project_id) -> None:
    """Back-compat: a unit_cost without currency_code is permitted —
    the frontend renders it as a raw number with a tooltip."""
    headers = make_auth_header()
    resp = await client.post(
        f"/api/projects/{project_id}/bom",
        json={"name": "Legacy", "quantity": 1, "unit_cost": 7.50},
        headers=headers,
    )
    assert resp.status_code == 201
    data = resp.json()
    assert data["unit_cost"] == 7.50
    assert data["currency_code"] is None


@pytest.mark.asyncio
async def test_add_bom_currency_helper_is_noop_on_sqlite() -> None:
    """The migration helper short-circuits on non-Postgres dialects so
    it can be called on every startup without harm."""
    from projects_api.db import add_bom_currency_if_missing
    await add_bom_currency_if_missing()
