"""3-D pin maps (definition.sockets3d): where a part's pins are on its model,
saved from a project's 3D scene. Sync from Snakie keeps them, the review
queue shows how far a submitted part's is done, and admins check the ones
saved from projects."""

from __future__ import annotations

from datetime import datetime, timedelta
from pathlib import Path
from typing import Optional

import pytest
from sqlalchemy import select

from projects_api.import_snakie_parts import import_library
from projects_api.models import Part
from projects_api.snakie_parts import pin_map_stats, pin_ports
from .conftest import make_auth_header

LIBRARY = Path(__file__).parent / "fixtures" / "snakie-library"

# The SG90's servo-1 group (Signal, VCC, GND) as a plug socket.
PLUG = {"size": [23, 32.5, 29], "sockets": [
    {"id": "s1", "point": [0, -16, 5], "normal": [0, -1, 0], "connector": "molex-5264-3", "port": "servo-1"},
]}


@pytest.fixture(autouse=True)
def _ages(monkeypatch: pytest.MonkeyPatch) -> None:
    from projects_api import auth as auth_module

    async def _fake_age(username: str, token: str) -> Optional[datetime]:
        return datetime.utcnow() - timedelta(days=1 if username == "newbie" else 365 * 5)

    monkeypatch.setattr(auth_module, "fetch_account_created_at", _fake_age)
    auth_module._clear_account_age_cache()
    monkeypatch.setenv("ADMIN_USERNAMES", "admin")


def test_pin_map_stats() -> None:
    d = {
        "headers": [
            {"pins": [{"name": "D1", "group": "bus"}, {"name": "V1", "group": "bus"}, {"name": "A"}, {"name": "B"}]},
        ],
        "connectors": [{"pins": [{"name": "SDA"}, {"name": "SCL"}]}],
    }
    assert pin_ports(d) == {"bus": ["D1", "V1"], "header-1": ["A", "B"], "connector-1": ["SDA", "SCL"]}
    assert pin_map_stats(d) == (0, 6)
    d["sockets3d"] = {"size": [1, 1, 1], "sockets": [
        {"id": "s1", "connector": "jst-xh-2", "port": "bus"},
        {"id": "s2", "connector": "pin", "pin": "A"},
        {"id": "s3", "connector": "pin", "pin": "nope"},
    ]}
    assert pin_map_stats(d) == (3, 6)
    assert pin_map_stats(None) == (0, 0)


async def _save_map(client, slug: str, user: str, sockets3d=PLUG):
    part = (await client.get(f"/api/parts/{slug}")).json()
    definition = {**(part.get("definition") or {}), "sockets3d": sockets3d}
    return await client.put(
        f"/api/parts/{slug}/definition?change_summary=" + "3D sockets for its model",
        json=definition, headers=make_auth_header(user),
    )


@pytest.mark.asyncio
async def test_sync_keeps_a_pin_map_saved_from_a_project(client, session) -> None:
    await import_library(LIBRARY, session)
    assert (await _save_map(client, "sg90", "bob")).status_code == 200
    # Snakie's SG90 has no pin map: the sync leaves the saved one.
    report = await import_library(LIBRARY, session)
    assert "sg90" in report.unchanged
    session.expire_all()
    sg90 = await session.scalar(select(Part).where(Part.slug == "sg90"))
    assert sg90.definition["sockets3d"]["sockets"][0]["port"] == "servo-1"


@pytest.mark.asyncio
async def test_sync_takes_snakies_own_pin_map(client, session, tmp_path) -> None:
    import shutil

    lib = tmp_path / "lib"
    shutil.copytree(LIBRARY, lib)
    await import_library(lib, session)
    assert (await _save_map(client, "sg90", "bob")).status_code == 200
    yml = lib / "sg90" / "parts.yml"
    yml.write_text(yml.read_text() + "sockets3d:\n  size: [23, 32.5, 29]\n  sockets:\n  - {id: p1, point: [1, 2, 3], normal: [0, 0, 1], connector: pin, pin: VCC}\n")
    await import_library(lib, session)
    session.expire_all()
    sg90 = await session.scalar(select(Part).where(Part.slug == "sg90"))
    assert [s["id"] for s in sg90.definition["sockets3d"]["sockets"]] == ["p1"]


@pytest.mark.asyncio
async def test_review_queue_shows_the_pin_map(client) -> None:
    sub = await client.post("/api/parts/submit", json={"name": "My Motor"}, headers=make_auth_header("newbie"))
    slug = sub.json()["slug"]
    definition = {"headers": [{"edge": "left", "pins": [{"name": "M+", "x": 0.2, "y": 0.9}, {"name": "M-", "x": 0.8, "y": 0.9}]}]}
    put = await client.put(f"/api/parts/{slug}/definition", json=definition, headers=make_auth_header("newbie"))
    assert put.status_code == 200, put.text
    one = {"size": [12, 35, 10], "sockets": [{"id": "s1", "point": [1, 2, 3], "normal": [0, -1, 0], "connector": "pin", "pin": "M+"}]}
    # Its sender may map its pins while it waits for review.
    assert (await _save_map(client, slug, "newbie", one)).status_code == 200
    queue = (await client.get("/api/admin/parts/submitted", headers=make_auth_header("admin"))).json()
    item = next(i for i in queue if i["slug"] == slug)
    assert (item["pins_on_model"], item["pins_total"], item["has_mesh"]) == (1, 2, False)
    # It's in the pin maps to check too.
    maps = (await client.get("/api/admin/parts/pin-maps", headers=make_auth_header("admin"))).json()
    assert [m["slug"] for m in maps] == [slug]
    assert maps[0]["saved_by"] == "newbie" and maps[0]["status"] == "pending"


@pytest.mark.asyncio
async def test_admin_checks_and_removes_pin_maps(client, session) -> None:
    await import_library(LIBRARY, session)
    assert (await client.get("/api/admin/parts/pin-maps", headers=make_auth_header("admin"))).json() == []
    assert (await client.get("/api/admin/parts/pin-maps", headers=make_auth_header("bob"))).status_code == 403
    assert (await _save_map(client, "sg90", "bob")).status_code == 200
    assert (await _save_map(client, "pico", "carol", {"size": [21, 51, 4], "sockets": [
        {"id": "s1", "point": [0, 0, 2], "normal": [0, 0, 1], "connector": "pin", "pin": "GP0"}]})).status_code == 200
    maps = (await client.get("/api/admin/parts/pin-maps", headers=make_auth_header("admin"))).json()
    assert [m["slug"] for m in maps] == ["pico", "sg90"]  # newest first
    sg90 = next(m for m in maps if m["slug"] == "sg90")
    assert (sg90["pins_on_model"], sg90["pins_total"], sg90["has_mesh"], sg90["saved_by"]) == (3, 3, True, "bob")

    # Looks right: off the list, without a new revision.
    revs = len((await client.get("/api/parts/sg90/revisions")).json())
    ok = await client.post("/api/admin/parts/sg90/pin-map/checked", headers=make_auth_header("admin"))
    assert ok.status_code == 200, ok.text
    assert ok.json()["definition"]["sockets3d"]["checked"]["by"] == "admin"
    assert len((await client.get("/api/parts/sg90/revisions")).json()) == revs
    # Saved again from a scene (the scene writes it afresh): back on the list.
    assert (await _save_map(client, "sg90", "bob", {**PLUG, "sockets": [{**PLUG["sockets"][0], "spin": 90}]})).status_code == 200
    maps = (await client.get("/api/admin/parts/pin-maps", headers=make_auth_header("admin"))).json()
    assert "sg90" in [m["slug"] for m in maps]

    # Wrong: removed, as a revision.
    gone = await client.delete("/api/admin/parts/pico/pin-map", headers=make_auth_header("admin"))
    assert gone.status_code == 200, gone.text
    assert "sockets3d" not in gone.json()["definition"]
    assert (await client.get("/api/parts/pico/revisions")).json()[0]["change_summary"] == "Removed the 3D pin map"
    maps = (await client.get("/api/admin/parts/pin-maps", headers=make_auth_header("admin"))).json()
    assert "pico" not in [m["slug"] for m in maps]
    assert (await client.delete("/api/admin/parts/pico/pin-map", headers=make_auth_header("admin"))).status_code == 404
