"""Project symbol CRUD tests (Symbol Designer).

Mirrors the fixture pattern in ``tests/test_schematics.py``: a per-test
project is created up front, then we exercise the symbol endpoints
against it. The terms-gate test deliberately strips the
``conftest.client`` bypass to confirm the gate fires on writes.
"""

from __future__ import annotations

import json

import pytest

from .conftest import make_auth_header


@pytest.fixture
async def project_id(client) -> int:
    headers = make_auth_header()
    resp = await client.post(
        "/api/projects",
        json={"title": "Symbol Test Project"},
        headers=headers,
    )
    assert resp.status_code == 201, resp.text
    return resp.json()["id"]


async def _bom_item_id(client, project_id: int, name: str = "Pico") -> int:
    """Create a BOM row on ``project_id`` and return its id."""
    headers = make_auth_header()
    resp = await client.post(
        f"/api/projects/{project_id}/bom",
        json={"name": name, "quantity": 1},
        headers=headers,
    )
    assert resp.status_code in (200, 201), resp.text
    return resp.json()["id"]


# --- Basic CRUD ---------------------------------------------------------


@pytest.mark.asyncio
async def test_post_creates_symbol_then_list_returns_it(client, project_id) -> None:
    headers = make_auth_header()
    resp = await client.post(
        f"/api/projects/{project_id}/symbols",
        json={"name": "MyLED", "ref_des_prefix": "D", "description": "5mm red"},
        headers=headers,
    )
    assert resp.status_code == 201, resp.text
    created = resp.json()
    assert created["name"] == "MyLED"
    assert created["ref_des_prefix"] == "D"
    assert created["description"] == "5mm red"
    assert created["project_id"] == project_id
    assert created["bom_item_id"] is None
    assert created["symbol_data"] is None

    listing = await client.get(f"/api/projects/{project_id}/symbols")
    assert listing.status_code == 200
    items = listing.json()
    assert isinstance(items, list)
    assert len(items) == 1
    assert items[0]["id"] == created["id"]


@pytest.mark.asyncio
async def test_post_without_name_returns_422(client, project_id) -> None:
    headers = make_auth_header()
    resp = await client.post(
        f"/api/projects/{project_id}/symbols",
        json={"ref_des_prefix": "U"},  # missing required `name`
        headers=headers,
    )
    assert resp.status_code == 422


@pytest.mark.asyncio
async def test_put_partial_update(client, project_id) -> None:
    """PUT applies the fields in the payload, leaves omitted fields alone."""
    headers = make_auth_header()
    bom_id = await _bom_item_id(client, project_id)
    created = await client.post(
        f"/api/projects/{project_id}/symbols",
        json={"name": "Initial", "description": "Initial desc"},
        headers=headers,
    )
    assert created.status_code == 201
    sid = created.json()["id"]

    # Update only the name.
    resp = await client.put(
        f"/api/projects/{project_id}/symbols/{sid}",
        json={"name": "Renamed"},
        headers=headers,
    )
    assert resp.status_code == 200, resp.text
    body = resp.json()
    assert body["name"] == "Renamed"
    assert body["description"] == "Initial desc"

    # Now update description + symbol_data + bom_item_id.
    blob = json.dumps({
        "bodyShapes": [{"id": "s1", "kind": "rect", "x": -32, "y": -16, "w": 64, "h": 32}],
        "pins": [{"id": "p1", "number": "1", "name": "A", "type": "I/O",
                  "side": "left", "x": -32, "y": 0}],
    })
    resp = await client.put(
        f"/api/projects/{project_id}/symbols/{sid}",
        json={
            "description": "Now linked",
            "symbol_data": blob,
            "bom_item_id": bom_id,
        },
        headers=headers,
    )
    assert resp.status_code == 200, resp.text
    body = resp.json()
    assert body["name"] == "Renamed"
    assert body["description"] == "Now linked"
    assert body["symbol_data"] == blob
    assert body["bom_item_id"] == bom_id

    # GET single reflects the latest state.
    one = await client.get(f"/api/projects/{project_id}/symbols/{sid}")
    assert one.status_code == 200
    assert one.json()["symbol_data"] == blob


@pytest.mark.asyncio
async def test_delete_removes_symbol(client, project_id) -> None:
    headers = make_auth_header()
    created = await client.post(
        f"/api/projects/{project_id}/symbols",
        json={"name": "Doomed"},
        headers=headers,
    )
    sid = created.json()["id"]

    resp = await client.delete(
        f"/api/projects/{project_id}/symbols/{sid}", headers=headers
    )
    assert resp.status_code == 204

    one = await client.get(f"/api/projects/{project_id}/symbols/{sid}")
    assert one.status_code == 404


@pytest.mark.asyncio
async def test_duplicate_creates_clone_with_copy_suffix(client, project_id) -> None:
    headers = make_auth_header()
    bom_id = await _bom_item_id(client, project_id)
    blob = json.dumps({"bodyShapes": [], "pins": [{"id": "p1"}]})
    created = await client.post(
        f"/api/projects/{project_id}/symbols",
        json={
            "name": "Original",
            "ref_des_prefix": "R",
            "description": "first",
            "bom_item_id": bom_id,
            "symbol_data": blob,
        },
        headers=headers,
    )
    sid = created.json()["id"]

    dup = await client.post(
        f"/api/projects/{project_id}/symbols/{sid}/duplicate",
        headers=headers,
    )
    assert dup.status_code == 201, dup.text
    body = dup.json()
    assert body["id"] != sid
    assert body["name"] == "Original (copy)"
    assert body["ref_des_prefix"] == "R"
    assert body["description"] == "first"
    assert body["bom_item_id"] == bom_id
    assert body["symbol_data"] == blob

    # List should now have two rows.
    listing = await client.get(f"/api/projects/{project_id}/symbols")
    assert len(listing.json()) == 2


# --- Public read --------------------------------------------------------


@pytest.mark.asyncio
async def test_anonymous_can_list_symbols(client, project_id) -> None:
    """Public GETs work without any auth header."""
    headers = make_auth_header()
    await client.post(
        f"/api/projects/{project_id}/symbols",
        json={"name": "PublicReadCheck"},
        headers=headers,
    )
    # No auth header.
    resp = await client.get(f"/api/projects/{project_id}/symbols")
    assert resp.status_code == 200
    assert len(resp.json()) == 1
    assert resp.json()[0]["name"] == "PublicReadCheck"


# --- Ownership ----------------------------------------------------------


@pytest.mark.asyncio
async def test_non_owner_post_returns_403(client, project_id) -> None:
    other = make_auth_header(username="someoneelse")
    resp = await client.post(
        f"/api/projects/{project_id}/symbols",
        json={"name": "hijack"},
        headers=other,
    )
    assert resp.status_code == 403


@pytest.mark.asyncio
async def test_non_owner_put_returns_403(client, project_id) -> None:
    owner = make_auth_header()
    created = await client.post(
        f"/api/projects/{project_id}/symbols",
        json={"name": "ok"},
        headers=owner,
    )
    sid = created.json()["id"]

    other = make_auth_header(username="someoneelse")
    resp = await client.put(
        f"/api/projects/{project_id}/symbols/{sid}",
        json={"name": "hijacked"},
        headers=other,
    )
    assert resp.status_code == 403


@pytest.mark.asyncio
async def test_non_owner_delete_returns_403(client, project_id) -> None:
    owner = make_auth_header()
    created = await client.post(
        f"/api/projects/{project_id}/symbols",
        json={"name": "ok"},
        headers=owner,
    )
    sid = created.json()["id"]

    other = make_auth_header(username="someoneelse")
    resp = await client.delete(
        f"/api/projects/{project_id}/symbols/{sid}",
        headers=other,
    )
    assert resp.status_code == 403


# --- BOM-link validation ------------------------------------------------


@pytest.mark.asyncio
async def test_create_with_nonexistent_bom_id_returns_422(client, project_id) -> None:
    """Linking to a non-existent BOM item id → 422 (FastAPI body-validation
    convention; documented in the route docstring)."""
    headers = make_auth_header()
    resp = await client.post(
        f"/api/projects/{project_id}/symbols",
        json={"name": "BadLink", "bom_item_id": 999_999},
        headers=headers,
    )
    assert resp.status_code == 422


@pytest.mark.asyncio
async def test_update_with_nonexistent_bom_id_returns_422(client, project_id) -> None:
    headers = make_auth_header()
    created = await client.post(
        f"/api/projects/{project_id}/symbols",
        json={"name": "Ok"},
        headers=headers,
    )
    sid = created.json()["id"]

    resp = await client.put(
        f"/api/projects/{project_id}/symbols/{sid}",
        json={"bom_item_id": 999_999},
        headers=headers,
    )
    assert resp.status_code == 422


# --- Large payload ------------------------------------------------------


@pytest.mark.asyncio
async def test_large_symbol_data_round_trips(client, project_id) -> None:
    """A ~1 MB symbol_data blob should round-trip via POST + GET.

    Typical custom-symbol JSON is well under a few KB; the 1 MB ceiling
    matches the schematic-data test and ensures even pathologically
    large body-shape sets are accepted by the column."""
    headers = make_auth_header()
    blob = "x" * (1 * 1024 * 1024)
    resp = await client.post(
        f"/api/projects/{project_id}/symbols",
        json={"name": "Huge", "symbol_data": blob},
        headers=headers,
    )
    assert resp.status_code == 201, resp.text
    sid = resp.json()["id"]
    assert resp.json()["symbol_data"] == blob

    one = await client.get(f"/api/projects/{project_id}/symbols/{sid}")
    assert one.status_code == 200
    assert len(one.json()["symbol_data"]) == len(blob)


# --- Terms gate ---------------------------------------------------------


@pytest.mark.asyncio
async def test_write_without_terms_acceptance_returns_403(client) -> None:
    """Confirms the T&Cs gate fires on symbol writes."""
    from projects_api import auth as auth_module

    app = client._transport.app
    for dep in (
        auth_module.require_terms_accepted,
        auth_module.require_terms_accepted_aged,
    ):
        app.dependency_overrides.pop(dep, None)

    headers = make_auth_header("alice")

    # Accept terms so the project can be created (POST /api/projects is
    # also gated by require_terms_accepted).
    acc = await client.post(
        "/api/users/me/accept-terms",
        json={"version": "1.0"},
        headers=headers,
    )
    assert acc.status_code == 200, acc.text
    proj = await client.post(
        "/api/projects",
        json={"title": "Gated Symbol Project"},
        headers=headers,
    )
    assert proj.status_code == 201, proj.text
    pid = proj.json()["id"]

    # Bump the terms version so alice's 1.0 acceptance goes stale.
    from projects_api import config as config_module
    from projects_api.routers import users as users_router

    def _fake_get_settings():
        s = config_module.Settings()
        return s.model_copy(update={"current_terms_version": "2.0"})

    import pytest as _pytest

    monkeypatch = _pytest.MonkeyPatch()
    monkeypatch.setattr(config_module, "get_settings", _fake_get_settings)
    monkeypatch.setattr(auth_module, "get_settings", _fake_get_settings)
    monkeypatch.setattr(users_router, "get_settings", _fake_get_settings)

    try:
        resp = await client.post(
            f"/api/projects/{pid}/symbols",
            json={"name": "should be blocked"},
            headers=headers,
        )
        assert resp.status_code == 403
        detail = resp.json().get("detail")
        assert isinstance(detail, dict)
        assert detail.get("detail") == "terms_not_accepted"
    finally:
        monkeypatch.undo()
