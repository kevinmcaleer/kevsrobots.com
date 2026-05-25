"""Versioning Phase 2 — library-symbol versioned writes + forking.

Covers the PUT append-vs-fork branch, the revision-history endpoints,
and the fork-filtering on the list endpoint. Admin = "kev" (default
ADMIN_USERNAMES); ordinary users are anything else.
"""

import pytest

from tests.conftest import make_auth_header

ADMIN = make_auth_header("kev")


def _payload(name="Widget", category="Custom", ref="U", data="{}"):
    return {"name": name, "category": category, "ref_des_prefix": ref,
            "symbol_data": data}


async def _create(client, **kw):
    resp = await client.post("/api/library/symbols", json=_payload(**kw), headers=ADMIN)
    assert resp.status_code == 201, resp.text
    return resp.json()


@pytest.mark.asyncio
async def test_create_writes_revision_one(client):
    sym = await _create(client, name="Resistor", category="Passive", ref="R")
    assert sym["current_revision_id"] is not None
    revs = await client.get(f"/api/library/symbols/{sym['id']}/revisions")
    assert revs.status_code == 200
    rows = revs.json()
    assert len(rows) == 1
    assert rows[0]["is_current"] is True
    assert rows[0]["change_summary"] == "Initial version"


@pytest.mark.asyncio
async def test_owner_edit_appends_revision(client):
    sym = await _create(client, name="Cap", category="Passive", ref="C")
    sid = sym["id"]
    # kev is both owner (created_by) and admin → appends.
    resp = await client.put(
        f"/api/library/symbols/{sid}",
        json={"description": "polarised", "change_summary": "note polarity"},
        headers=ADMIN,
    )
    assert resp.status_code == 200, resp.text
    updated = resp.json()
    assert updated["id"] == sid  # same lineage
    assert updated["description"] == "polarised"
    assert updated["forked_from_symbol_id"] is None
    revs = (await client.get(f"/api/library/symbols/{sid}/revisions")).json()
    assert len(revs) == 2
    assert revs[0]["is_current"] is True
    assert revs[0]["change_summary"] == "note polarity"


@pytest.mark.asyncio
async def test_non_owner_edit_forks(client):
    sym = await _create(client, name="LED", category="Active", ref="D")
    sid = sym["id"]
    other = make_auth_header("alice")
    resp = await client.put(
        f"/api/library/symbols/{sid}",
        json={"description": "alice's tweak"},
        headers=other,
    )
    assert resp.status_code == 200, resp.text
    fork = resp.json()
    assert fork["id"] != sid                       # new lineage
    assert fork["forked_from_symbol_id"] == sid
    assert fork["forked_from_revision_id"] is not None
    assert fork["created_by_username"] == "alice"
    assert fork["description"] == "alice's tweak"
    # Original lineage untouched.
    orig = (await client.get(f"/api/library/symbols/{sid}")).json()
    assert orig["description"] != "alice's tweak"
    orig_revs = (await client.get(f"/api/library/symbols/{sid}/revisions")).json()
    assert len(orig_revs) == 1  # no revision appended to the original


@pytest.mark.asyncio
async def test_list_excludes_forks_by_default(client):
    base = await _create(client, name="BaseSym", category="Module")
    other = make_auth_header("bob")
    fork = (await client.put(
        f"/api/library/symbols/{base['id']}",
        json={"description": "bob's"}, headers=other,
    )).json()

    listed = (await client.get("/api/library/symbols")).json()
    ids = {s["id"] for s in listed}
    assert base["id"] in ids
    assert fork["id"] not in ids   # fork hidden from curated list

    # ?mine as bob includes bob's fork.
    mine = (await client.get("/api/library/symbols?mine=1", headers=other)).json()
    mine_ids = {s["id"] for s in mine}
    assert fork["id"] in mine_ids
    assert base["id"] in mine_ids


@pytest.mark.asyncio
async def test_get_specific_revision(client):
    sym = await _create(client, name="Diode", category="Active", ref="D",
                        data='{"v":1}')
    sid = sym["id"]
    await client.put(f"/api/library/symbols/{sid}",
                     json={"symbol_data": '{"v":2}'}, headers=ADMIN)
    revs = (await client.get(f"/api/library/symbols/{sid}/revisions")).json()
    assert len(revs) == 2
    # Oldest revision still holds the v1 snapshot (immutability).
    oldest = revs[-1]
    full = (await client.get(
        f"/api/library/symbols/{sid}/revisions/{oldest['id']}")).json()
    assert full["symbol_data"] == '{"v":1}'


# --- Power model: is_power_port flag -----------------------------------


@pytest.mark.asyncio
async def test_create_defaults_to_component(client):
    """A normal symbol is BOM-eligible (is_power_port False by default)."""
    sym = await _create(client, name="Resistor", category="Passive", ref="R")
    assert sym["is_power_port"] is False


@pytest.mark.asyncio
async def test_create_power_port(client):
    resp = await client.post(
        "/api/library/symbols",
        json={"name": "GND", "category": "Power", "ref_des_prefix": "GND",
              "symbol_data": "{}", "is_power_port": True},
        headers=ADMIN,
    )
    assert resp.status_code == 201, resp.text
    assert resp.json()["is_power_port"] is True


@pytest.mark.asyncio
async def test_seed_flags_power_ports(client):
    resp = await client.post("/api/library/symbols/seed-builtins", headers=ADMIN)
    assert resp.status_code in (200, 201), resp.text
    by_name = {s["name"]: s for s in resp.json()}
    # Rails / net flags are power ports — never a BOM line.
    for rail in ("GND", "V+ rail", "+5V", "+3V3", "VBUS"):
        assert by_name[rail]["is_power_port"] is True, rail
    # A buyable component is not.
    assert by_name["Resistor"]["is_power_port"] is False
    assert by_name["Raspberry Pi Pico"]["is_power_port"] is False
