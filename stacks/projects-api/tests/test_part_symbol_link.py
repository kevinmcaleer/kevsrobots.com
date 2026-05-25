"""Part↔symbol link (part-hub UX).

A part may point at a curated library-symbol lineage so it can be dropped
straight into a schematic. Covers create/update linking, the nested
``symbol`` summary on the detail response, revision-snapshot carry-over,
clearing the link, and rejection of a dangling symbol id.

The 14-day account-age gate is faked open via ``_old_account`` (same
pattern as ``test_parts.py``). Library symbols are created as admin
("kev").
"""

from __future__ import annotations

from datetime import datetime, timedelta
from typing import Optional

import pytest

from .conftest import make_auth_header

ADMIN = make_auth_header("kev")


@pytest.fixture(autouse=True)
def _old_account(monkeypatch: pytest.MonkeyPatch) -> None:
    """Pretend every account is 5 years old so the age gate passes."""
    from projects_api import auth as auth_module

    async def _fake_age(username: str, token: str) -> Optional[datetime]:
        return datetime.utcnow() - timedelta(days=365 * 5)

    monkeypatch.setattr(auth_module, "fetch_account_created_at", _fake_age)
    auth_module._clear_account_age_cache()


async def _make_symbol(client, name="Resistor", category="Passive", ref="R") -> dict:
    resp = await client.post(
        "/api/library/symbols",
        json={
            "name": name,
            "category": category,
            "ref_des_prefix": ref,
            "symbol_data": '{"pins":[]}',
        },
        headers=ADMIN,
    )
    assert resp.status_code == 201, resp.text
    return resp.json()


async def _make_part(client, name="Widget", **extra) -> dict:
    body = {"name": name}
    body.update(extra)
    resp = await client.post("/api/parts", json=body, headers=make_auth_header())
    assert resp.status_code == 201, resp.text
    return resp.json()


@pytest.mark.asyncio
async def test_create_part_with_symbol_link(client) -> None:
    sym = await _make_symbol(client)
    part = await _make_part(client, name="220 Ohm Resistor", symbol_id=sym["id"])

    assert part["symbol_id"] == sym["id"]
    assert part["symbol"] is not None
    assert part["symbol"]["id"] == sym["id"]
    assert part["symbol"]["name"] == "Resistor"
    assert part["symbol"]["ref_des_prefix"] == "R"
    assert part["symbol"]["category"] == "Passive"
    # symbol_data + current_revision_id surfaced for the canvas/version label.
    assert part["symbol"]["symbol_data"] == '{"pins":[]}'
    assert part["symbol"]["current_revision_id"] is not None

    # Initial revision snapshots the link.
    rev = await client.get(
        f"/api/parts/{part['slug']}/revisions/{part['current_revision_id']}"
    )
    assert rev.status_code == 200
    assert rev.json()["symbol_id"] == sym["id"]


@pytest.mark.asyncio
async def test_link_via_update(client) -> None:
    sym = await _make_symbol(client, name="LED", category="Active", ref="D")
    part = await _make_part(client, name="Red LED")
    assert part["symbol_id"] is None
    assert part["symbol"] is None

    resp = await client.put(
        f"/api/parts/{part['slug']}",
        json={"symbol_id": sym["id"], "change_summary": "link symbol"},
        headers=make_auth_header(),
    )
    assert resp.status_code == 200, resp.text
    updated = resp.json()
    assert updated["symbol_id"] == sym["id"]
    assert updated["symbol"]["name"] == "LED"

    rev = await client.get(
        f"/api/parts/{part['slug']}/revisions/{updated['current_revision_id']}"
    )
    assert rev.json()["symbol_id"] == sym["id"]


@pytest.mark.asyncio
async def test_clear_link_via_null(client) -> None:
    sym = await _make_symbol(client, name="Cap", category="Passive", ref="C")
    part = await _make_part(client, name="100nF Cap", symbol_id=sym["id"])
    assert part["symbol_id"] == sym["id"]

    resp = await client.put(
        f"/api/parts/{part['slug']}",
        json={"symbol_id": None, "change_summary": "unlink symbol"},
        headers=make_auth_header(),
    )
    assert resp.status_code == 200, resp.text
    cleared = resp.json()
    assert cleared["symbol_id"] is None
    assert cleared["symbol"] is None


@pytest.mark.asyncio
async def test_omitting_symbol_id_leaves_link_unchanged(client) -> None:
    sym = await _make_symbol(client, name="Diode", category="Active", ref="D")
    part = await _make_part(client, name="1N4148", symbol_id=sym["id"])

    # Edit an unrelated field WITHOUT sending symbol_id — link must persist.
    resp = await client.put(
        f"/api/parts/{part['slug']}",
        json={"description_md": "fast switching diode", "change_summary": "desc"},
        headers=make_auth_header(),
    )
    assert resp.status_code == 200, resp.text
    assert resp.json()["symbol_id"] == sym["id"]


@pytest.mark.asyncio
async def test_create_rejects_unknown_symbol(client) -> None:
    resp = await client.post(
        "/api/parts",
        json={"name": "Ghost Part", "symbol_id": 99999},
        headers=make_auth_header(),
    )
    assert resp.status_code == 400, resp.text


@pytest.mark.asyncio
async def test_update_rejects_unknown_symbol(client) -> None:
    part = await _make_part(client, name="Real Part")
    resp = await client.put(
        f"/api/parts/{part['slug']}",
        json={"symbol_id": 99999, "change_summary": "bad link"},
        headers=make_auth_header(),
    )
    assert resp.status_code == 400, resp.text


@pytest.mark.asyncio
async def test_restore_carries_symbol_link(client) -> None:
    sym = await _make_symbol(client, name="Transistor", category="Active", ref="Q")
    part = await _make_part(client, name="2N2222", symbol_id=sym["id"])
    linked_rev = part["current_revision_id"]

    # Unlink, then restore the earlier (linked) revision.
    await client.put(
        f"/api/parts/{part['slug']}",
        json={"symbol_id": None, "change_summary": "unlink"},
        headers=make_auth_header(),
    )
    resp = await client.post(
        f"/api/parts/{part['slug']}/revisions/{linked_rev}/restore",
        headers=make_auth_header(),
    )
    assert resp.status_code == 200, resp.text
    assert resp.json()["symbol_id"] == sym["id"]
