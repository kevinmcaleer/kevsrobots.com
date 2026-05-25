"""Versioning Phase 3 — BOM rows pin a part revision.

A BOM row linked to a part is pinned to that part's revision at link time,
so later edits to the part don't silently change the project's BOM. The
``part_revision_outdated`` flag is the "newer version available" signal.

Part creation is age-gated (faked open via ``_old_account``); BOM endpoints
are owner-gated (the project author is the test user).
"""

from __future__ import annotations

from datetime import datetime, timedelta
from typing import Optional

import pytest

from .conftest import make_auth_header

H = make_auth_header()


@pytest.fixture(autouse=True)
def _old_account(monkeypatch: pytest.MonkeyPatch) -> None:
    from projects_api import auth as auth_module

    async def _fake_age(username: str, token: str) -> Optional[datetime]:
        return datetime.utcnow() - timedelta(days=365 * 5)

    monkeypatch.setattr(auth_module, "fetch_account_created_at", _fake_age)
    auth_module._clear_account_age_cache()


@pytest.fixture
async def project_id(client) -> int:
    resp = await client.post("/api/projects", json={"title": "BOM v Project"}, headers=H)
    return resp.json()["id"]


async def _make_part(client, name="Servo") -> dict:
    resp = await client.post("/api/parts", json={"name": name}, headers=H)
    assert resp.status_code == 201, resp.text
    return resp.json()


async def _edit_part(client, slug, summary="tweak") -> dict:
    """Write a new revision; returns the updated part (new current rev)."""
    resp = await client.put(
        f"/api/parts/{slug}",
        json={"description_md": "edit " + summary, "change_summary": summary},
        headers=H,
    )
    assert resp.status_code == 200, resp.text
    return resp.json()


async def _add_bom(client, project_id, **extra) -> dict:
    body = {"name": "row", "quantity": 1}
    body.update(extra)
    resp = await client.post(f"/api/projects/{project_id}/bom", json=body, headers=H)
    assert resp.status_code == 201, resp.text
    return resp.json()


@pytest.mark.asyncio
async def test_links_pin_current_revision(client, project_id) -> None:
    part = await _make_part(client, "Pinned Servo")
    item = await _add_bom(client, project_id, part_id=part["id"])
    assert item["part_revision_id"] == part["current_revision_id"]
    assert item["part_revision_outdated"] is False


@pytest.mark.asyncio
async def test_part_edit_marks_bom_outdated(client, project_id) -> None:
    part = await _make_part(client, "Drifting Part")
    rev1 = part["current_revision_id"]
    item = await _add_bom(client, project_id, part_id=part["id"])
    assert item["part_revision_id"] == rev1

    updated = await _edit_part(client, part["slug"])
    assert updated["current_revision_id"] != rev1  # new revision

    rows = (await client.get(f"/api/projects/{project_id}/bom")).json()
    row = next(r for r in rows if r["id"] == item["id"])
    assert row["part_revision_id"] == rev1            # still pinned to the old rev
    assert row["part_revision_outdated"] is True       # newer available


@pytest.mark.asyncio
async def test_explicit_revision_honoured(client, project_id) -> None:
    part = await _make_part(client, "Explicit Part")
    rev1 = part["current_revision_id"]
    await _edit_part(client, part["slug"])  # now rev2 is current

    item = await _add_bom(client, project_id, part_id=part["id"], part_revision_id=rev1)
    assert item["part_revision_id"] == rev1
    assert item["part_revision_outdated"] is True


@pytest.mark.asyncio
async def test_invalid_revision_falls_back_to_current(client, project_id) -> None:
    part = await _make_part(client, "Fallback Part")
    item = await _add_bom(client, project_id, part_id=part["id"], part_revision_id=999999)
    assert item["part_revision_id"] == part["current_revision_id"]
    assert item["part_revision_outdated"] is False


@pytest.mark.asyncio
async def test_no_part_no_pin(client, project_id) -> None:
    item = await _add_bom(client, project_id, name="freeform widget")
    assert item["part_revision_id"] is None
    assert item["part_revision_outdated"] is False


@pytest.mark.asyncio
async def test_unrelated_edit_preserves_pin(client, project_id) -> None:
    part = await _make_part(client, "Stable Pin Part")
    rev1 = part["current_revision_id"]
    item = await _add_bom(client, project_id, part_id=part["id"])
    await _edit_part(client, part["slug"])  # part now has rev2

    # Edit an unrelated BOM field WITHOUT sending part_revision_id.
    resp = await client.put(
        f"/api/projects/{project_id}/bom/{item['id']}",
        json={"name": "row", "quantity": 5, "part_id": part["id"]},
        headers=H,
    )
    assert resp.status_code == 200, resp.text
    # Pin is preserved (NOT silently bumped to latest), still flagged outdated.
    assert resp.json()["part_revision_id"] == rev1
    assert resp.json()["part_revision_outdated"] is True


@pytest.mark.asyncio
async def test_upgrade_revision_repins_to_current(client, project_id) -> None:
    part = await _make_part(client, "Upgradable Part")
    rev1 = part["current_revision_id"]
    item = await _add_bom(client, project_id, part_id=part["id"])
    updated = await _edit_part(client, part["slug"])      # rev2 is current now
    rev2 = updated["current_revision_id"]

    # Outdated before upgrade.
    rows = (await client.get(f"/api/projects/{project_id}/bom")).json()
    assert next(r for r in rows if r["id"] == item["id"])["part_revision_outdated"] is True

    resp = await client.post(
        f"/api/projects/{project_id}/bom/{item['id']}/upgrade-revision", headers=H
    )
    assert resp.status_code == 200, resp.text
    body = resp.json()
    assert body["part_revision_id"] == rev2 != rev1
    assert body["part_revision_outdated"] is False


@pytest.mark.asyncio
async def test_upgrade_revision_requires_a_part(client, project_id) -> None:
    item = await _add_bom(client, project_id, name="freeform")
    resp = await client.post(
        f"/api/projects/{project_id}/bom/{item['id']}/upgrade-revision", headers=H
    )
    assert resp.status_code == 400


@pytest.mark.asyncio
async def test_backfill_pins_legacy_rows(client, project_id, sessionmaker_) -> None:
    """A pre-Phase-3 row (part linked, no pin) gets pinned to the part's
    current revision by the startup backfill."""
    from projects_api import db as db_module
    from projects_api.models import ProjectBOMItem

    part = await _make_part(client, "Legacy Part")
    item = await _add_bom(client, project_id, part_id=part["id"])
    # Simulate a legacy row by clearing the pin the API just set.
    async with sessionmaker_() as s:
        row = await s.get(ProjectBOMItem, item["id"])
        row.part_revision_id = None
        await s.commit()

    await db_module.backfill_bom_part_revisions()

    async with sessionmaker_() as s:
        row = await s.get(ProjectBOMItem, item["id"])
        assert row.part_revision_id == part["current_revision_id"]


@pytest.mark.asyncio
async def test_bom_row_exposes_part_cover(client, project_id) -> None:
    part = await _make_part(client, "BOM Cover Part")
    await client.post(
        f"/api/parts/{part['slug']}/photos/link",
        json={"external_url": "https://example.com/bom-cover.jpg"},
        headers=H,
    )
    item = await _add_bom(client, project_id, part_id=part["id"])
    rows = (await client.get(f"/api/projects/{project_id}/bom")).json()
    row = next(r for r in rows if r["id"] == item["id"])
    assert row["cover_url"] == "https://example.com/bom-cover.jpg"


@pytest.mark.asyncio
async def test_changing_part_repins_to_new_current(client, project_id) -> None:
    a = await _make_part(client, "Part A")
    b = await _make_part(client, "Part B")
    item = await _add_bom(client, project_id, part_id=a["id"])
    assert item["part_revision_id"] == a["current_revision_id"]

    resp = await client.put(
        f"/api/projects/{project_id}/bom/{item['id']}",
        json={"name": "row", "quantity": 1, "part_id": b["id"]},
        headers=H,
    )
    assert resp.status_code == 200, resp.text
    assert resp.json()["part_revision_id"] == b["current_revision_id"]
