"""Project schematic CRUD tests (issue #178, Phase E2).

Mirrors the fixture pattern in ``tests/test_instructions.py``: a per-test
project is created up front, then we exercise the schematic endpoints
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
        json={"title": "Schematic Test Project"},
        headers=headers,
    )
    assert resp.status_code == 201, resp.text
    return resp.json()["id"]


# --- Basic CRUD ---------------------------------------------------------


@pytest.mark.asyncio
async def test_schematic_post_is_idempotent(client, project_id) -> None:
    """A second POST returns the SAME row with 200, not 201."""
    headers = make_auth_header()
    first = await client.post(
        f"/api/projects/{project_id}/schematic",
        json={"name": "Main schematic", "description": "Pico + LED"},
        headers=headers,
    )
    assert first.status_code == 200, first.text
    first_body = first.json()
    assert first_body["name"] == "Main schematic"
    assert first_body["description"] == "Pico + LED"
    assert first_body["project_id"] == project_id
    assert first_body["schematic_data"] is None

    second = await client.post(
        f"/api/projects/{project_id}/schematic",
        json={"name": "Different on second POST"},
        headers=headers,
    )
    assert second.status_code == 200
    second_body = second.json()
    # Idempotent: same row id, original name preserved (no overwrite).
    assert second_body["id"] == first_body["id"]
    assert second_body["name"] == "Main schematic"


@pytest.mark.asyncio
async def test_get_returns_404_when_no_schematic(client, project_id) -> None:
    resp = await client.get(f"/api/projects/{project_id}/schematic")
    assert resp.status_code == 404


@pytest.mark.asyncio
async def test_put_updates_fields_partially(client, project_id) -> None:
    """PUT applies the fields in the payload, leaves omitted fields alone."""
    headers = make_auth_header()
    await client.post(
        f"/api/projects/{project_id}/schematic",
        json={"name": "Initial", "description": "Initial desc"},
        headers=headers,
    )

    # Update only the name — description should stay.
    resp = await client.put(
        f"/api/projects/{project_id}/schematic",
        json={"name": "Updated"},
        headers=headers,
    )
    assert resp.status_code == 200, resp.text
    body = resp.json()
    assert body["name"] == "Updated"
    assert body["description"] == "Initial desc"

    # Now update schematic_data — name + description still preserved.
    blob = json.dumps({"instances": [{"id": "i1", "symbolId": "rpi-pico"}]})
    resp = await client.put(
        f"/api/projects/{project_id}/schematic",
        json={"schematic_data": blob},
        headers=headers,
    )
    assert resp.status_code == 200, resp.text
    body = resp.json()
    assert body["name"] == "Updated"
    assert body["description"] == "Initial desc"
    assert body["schematic_data"] == blob

    # GET reflects the latest state.
    get_resp = await client.get(f"/api/projects/{project_id}/schematic")
    assert get_resp.status_code == 200
    assert get_resp.json()["schematic_data"] == blob


@pytest.mark.asyncio
async def test_delete_removes_schematic(client, project_id) -> None:
    headers = make_auth_header()
    await client.post(
        f"/api/projects/{project_id}/schematic", json={}, headers=headers
    )
    resp = await client.delete(
        f"/api/projects/{project_id}/schematic", headers=headers
    )
    assert resp.status_code == 204

    get_resp = await client.get(f"/api/projects/{project_id}/schematic")
    assert get_resp.status_code == 404


# --- Public read --------------------------------------------------------


@pytest.mark.asyncio
async def test_anonymous_can_read_schematic(client, project_id) -> None:
    """Public GETs work without any auth header."""
    headers = make_auth_header()
    await client.post(
        f"/api/projects/{project_id}/schematic",
        json={"name": "public read", "schematic_data": "{}"},
        headers=headers,
    )

    # No auth header.
    resp = await client.get(f"/api/projects/{project_id}/schematic")
    assert resp.status_code == 200
    assert resp.json()["name"] == "public read"
    assert resp.json()["schematic_data"] == "{}"


# --- Ownership ----------------------------------------------------------


@pytest.mark.asyncio
async def test_non_owner_post_returns_403(client, project_id) -> None:
    other = make_auth_header(username="someoneelse")
    resp = await client.post(
        f"/api/projects/{project_id}/schematic",
        json={"name": "hijack"},
        headers=other,
    )
    assert resp.status_code == 403


@pytest.mark.asyncio
async def test_non_owner_put_returns_403(client, project_id) -> None:
    owner = make_auth_header()
    await client.post(
        f"/api/projects/{project_id}/schematic",
        json={"name": "ok"},
        headers=owner,
    )
    other = make_auth_header(username="someoneelse")
    resp = await client.put(
        f"/api/projects/{project_id}/schematic",
        json={"name": "hijacked"},
        headers=other,
    )
    assert resp.status_code == 403


@pytest.mark.asyncio
async def test_non_owner_delete_returns_403(client, project_id) -> None:
    owner = make_auth_header()
    await client.post(
        f"/api/projects/{project_id}/schematic", json={}, headers=owner
    )
    other = make_auth_header(username="someoneelse")
    resp = await client.delete(
        f"/api/projects/{project_id}/schematic", headers=other
    )
    assert resp.status_code == 403


# --- Large payload ------------------------------------------------------


@pytest.mark.asyncio
async def test_large_schematic_data_round_trips(client, project_id) -> None:
    """A ~1 MB schematic_data blob should round-trip via POST and GET.

    Schematic graphs for typical Pico-class projects sit at a few tens
    of KB; the 1 MB ceiling is the practical upper bound for v1 (e.g. a
    massive shield with hundreds of pins + nets) and we want it well
    inside what the API will accept."""
    headers = make_auth_header()
    # Build a deterministic payload roughly 1 MB in size. The blob itself
    # is opaque to the server — what matters is that the column accepts
    # it and the JSON encoder round-trips it intact.
    blob = "x" * (1 * 1024 * 1024)
    resp = await client.post(
        f"/api/projects/{project_id}/schematic",
        json={"schematic_data": blob},
        headers=headers,
    )
    assert resp.status_code == 200, resp.text
    assert resp.json()["schematic_data"] == blob

    # GET still returns it verbatim.
    get_resp = await client.get(f"/api/projects/{project_id}/schematic")
    assert get_resp.status_code == 200
    assert len(get_resp.json()["schematic_data"]) == len(blob)


# --- Terms gate ---------------------------------------------------------
#
# The default ``client`` fixture in ``conftest.py`` patches
# ``require_terms_accepted`` to no-op so the bulk of the suite doesn't
# need to first POST /api/users/me/accept-terms. This test opts back
# into the real gate (same pattern as ``tests/test_instructions.py``)
# and confirms a schematic-write hits 403 before acceptance.


@pytest.mark.asyncio
async def test_write_without_terms_acceptance_returns_403(client) -> None:
    from projects_api import auth as auth_module

    app = client._transport.app
    for dep in (
        auth_module.require_terms_accepted,
        auth_module.require_terms_accepted_aged,
    ):
        app.dependency_overrides.pop(dep, None)

    headers = make_auth_header("alice")

    # Accept terms so the project itself can be created — POST /api/projects
    # is also gated by ``require_terms_accepted``.
    acc = await client.post(
        "/api/users/me/accept-terms",
        json={"version": "1.0"},
        headers=headers,
    )
    assert acc.status_code == 200, acc.text
    proj = await client.post(
        "/api/projects",
        json={"title": "Gated Schematic Project"},
        headers=headers,
    )
    assert proj.status_code == 201, proj.text
    pid = proj.json()["id"]

    # Now bump the terms version so alice's 1.0 acceptance goes stale.
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
            f"/api/projects/{pid}/schematic",
            json={"name": "should be blocked"},
            headers=headers,
        )
        assert resp.status_code == 403
        detail = resp.json().get("detail")
        assert isinstance(detail, dict)
        assert detail.get("detail") == "terms_not_accepted"
    finally:
        monkeypatch.undo()
