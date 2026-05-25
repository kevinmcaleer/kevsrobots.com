"""Tests for the parts catalog (issue #121).

The 14-day account-age gate is exercised by monkeypatching
``projects_api.auth.fetch_account_created_at``. The default fixture
returns a "5 years ago" timestamp so most tests don't have to think
about it.
"""

from __future__ import annotations

from datetime import datetime, timedelta
from typing import Optional

import pytest

from .conftest import make_auth_header


@pytest.fixture(autouse=True)
def _old_account(monkeypatch: pytest.MonkeyPatch) -> None:
    """By default, pretend every account is 5 years old."""
    from projects_api import auth as auth_module

    async def _fake_age(username: str, token: str) -> Optional[datetime]:
        return datetime.utcnow() - timedelta(days=365 * 5)

    monkeypatch.setattr(auth_module, "fetch_account_created_at", _fake_age)
    auth_module._clear_account_age_cache()


# --- create / get / search ---------------------------------------------


@pytest.mark.asyncio
async def test_create_part_writes_initial_revision(client) -> None:
    headers = make_auth_header()
    resp = await client.post(
        "/api/parts",
        json={
            "name": "SG90 Micro Servo",
            "sku": "SG90",
            "supplier_url": "https://example.com/sg90",
            "supplier_name": "ThePiHut",
            "tags": ["servo", "rc"],
        },
        headers=headers,
    )
    assert resp.status_code == 201, resp.text
    data = resp.json()
    assert data["slug"] == "sg90-micro-servo"
    assert data["name"] == "SG90 Micro Servo"
    assert data["sku"] == "SG90"
    assert data["created_by"] == "testuser"
    assert data["status"] == "draft"
    assert data["usage_count"] == 0
    assert data["current_revision_id"] is not None
    assert len(data["suppliers"]) == 1
    assert data["suppliers"][0]["url"] == "https://example.com/sg90"
    assert len(data["recent_revisions"]) == 1
    assert data["recent_revisions"][0]["change_summary"] == "Initial draft"
    assert sorted(data["tags"]) == ["rc", "servo"]


@pytest.mark.asyncio
async def test_slug_disambiguation(client) -> None:
    headers = make_auth_header()
    r1 = await client.post("/api/parts", json={"name": "Widget"}, headers=headers)
    r2 = await client.post("/api/parts", json={"name": "Widget"}, headers=headers)
    assert r1.json()["slug"] == "widget"
    assert r2.json()["slug"] == "widget-2"


@pytest.mark.asyncio
async def test_search_matches_name_sku_and_mpn(client, session) -> None:
    headers = make_auth_header()
    await client.post(
        "/api/parts",
        json={"name": "SG90 Micro Servo", "sku": "SG90", "mpn": "TIANKONGRC-SG90"},
        headers=headers,
    )
    await client.post(
        "/api/parts",
        json={"name": "Raspberry Pi Pico", "sku": "RP2040-PICO"},
        headers=headers,
    )

    # By name
    r = await client.get("/api/parts", params={"q": "servo"})
    assert r.status_code == 200
    assert any("Servo" in p["name"] for p in r.json())

    # By sku
    r = await client.get("/api/parts", params={"q": "rp2040"})
    assert any(p["sku"] == "RP2040-PICO" for p in r.json())

    # By mpn
    r = await client.get("/api/parts", params={"q": "tiankongrc"})
    assert any(p["name"] == "SG90 Micro Servo" for p in r.json())


@pytest.mark.asyncio
async def test_search_matches_alias(client, session) -> None:
    headers = make_auth_header()
    resp = await client.post(
        "/api/parts",
        json={"name": "SG90 Micro Servo"},
        headers=headers,
    )
    part_id = resp.json()["id"]
    # Insert an alias directly — the public API for aliases lands in Phase 3.
    from projects_api.models import PartAlias

    session.add(PartAlias(part_id=part_id, alias="9g servo"))
    await session.commit()

    r = await client.get("/api/parts", params={"q": "9g"})
    assert r.status_code == 200
    assert any(p["id"] == part_id for p in r.json())


# --- update / revisions / restore --------------------------------------


@pytest.mark.asyncio
async def test_update_creates_new_revision_and_bumps_current(client) -> None:
    headers = make_auth_header()
    create = await client.post(
        "/api/parts",
        json={"name": "Widget", "sku": "W1"},
        headers=headers,
    )
    slug = create.json()["slug"]
    original_rev = create.json()["current_revision_id"]

    upd = await client.put(
        f"/api/parts/{slug}",
        json={
            "description_md": "Now with documentation",
            "change_summary": "Add description",
        },
        headers=headers,
    )
    assert upd.status_code == 200, upd.text
    new_rev = upd.json()["current_revision_id"]
    assert new_rev != original_rev
    assert upd.json()["description_md"] == "Now with documentation"
    # Two revisions present.
    revs = await client.get(f"/api/parts/{slug}/revisions")
    assert len(revs.json()) == 2


@pytest.mark.asyncio
async def test_update_with_no_change_is_400(client) -> None:
    headers = make_auth_header()
    create = await client.post(
        "/api/parts",
        json={"name": "Widget", "sku": "W1"},
        headers=headers,
    )
    slug = create.json()["slug"]
    # Same data + just a change_summary.
    resp = await client.put(
        f"/api/parts/{slug}",
        json={"name": "Widget", "sku": "W1", "change_summary": "noop"},
        headers=headers,
    )
    assert resp.status_code == 400
    assert "no fields changed" in resp.json()["detail"].lower()


@pytest.mark.asyncio
async def test_restore_creates_new_revision_non_destructive(client) -> None:
    headers = make_auth_header()
    create = await client.post(
        "/api/parts",
        json={"name": "Widget", "description_md": "v1"},
        headers=headers,
    )
    slug = create.json()["slug"]
    rev1 = create.json()["current_revision_id"]

    # Edit to v2.
    await client.put(
        f"/api/parts/{slug}",
        json={"description_md": "v2", "change_summary": "to v2"},
        headers=headers,
    )

    # Restore rev1.
    restore = await client.post(
        f"/api/parts/{slug}/revisions/{rev1}/restore",
        headers=headers,
    )
    assert restore.status_code == 200, restore.text
    restored = restore.json()
    # New revision id != rev1 (history is linear, restore is non-destructive).
    assert restored["current_revision_id"] != rev1
    assert restored["description_md"] == "v1"

    revs = await client.get(f"/api/parts/{slug}/revisions")
    revs_list = revs.json()
    # rev1 + v2 edit + restore = 3 revisions
    assert len(revs_list) == 3
    # Latest revision change_summary mentions the restore.
    assert revs_list[0]["change_summary"] == f"Restored from revision #{rev1}"


# --- account-age gate --------------------------------------------------


@pytest.mark.asyncio
async def test_account_age_gate_blocks_young_accounts(client, monkeypatch) -> None:
    from projects_api import auth as auth_module

    async def _young(username: str, token: str) -> Optional[datetime]:
        return datetime.utcnow() - timedelta(days=3)

    monkeypatch.setattr(auth_module, "fetch_account_created_at", _young)
    auth_module._clear_account_age_cache()

    resp = await client.post(
        "/api/parts",
        json={"name": "Too new"},
        headers=make_auth_header(),
    )
    assert resp.status_code == 403
    assert "14 days" in resp.json()["detail"]


@pytest.mark.asyncio
async def test_account_age_gate_fails_closed_on_lookup_failure(client, monkeypatch) -> None:
    from projects_api import auth as auth_module

    async def _failed(username: str, token: str) -> Optional[datetime]:
        return None

    monkeypatch.setattr(auth_module, "fetch_account_created_at", _failed)
    auth_module._clear_account_age_cache()

    resp = await client.post(
        "/api/parts",
        json={"name": "Anything"},
        headers=make_auth_header(),
    )
    assert resp.status_code == 403
    assert "verified" in resp.json()["detail"].lower()


# --- BOM <-> usage_count ----------------------------------------------


async def _create_project(client) -> int:
    headers = make_auth_header()
    resp = await client.post("/api/projects", json={"title": "Usage Count Test"}, headers=headers)
    return resp.json()["id"]


async def _create_part(client, name: str) -> dict:
    headers = make_auth_header()
    resp = await client.post("/api/parts", json={"name": name}, headers=headers)
    return resp.json()


@pytest.mark.asyncio
async def test_bom_part_link_bumps_usage_count(client) -> None:
    project_id = await _create_project(client)
    part = await _create_part(client, "Pi Pico")
    headers = make_auth_header()

    resp = await client.post(
        f"/api/projects/{project_id}/bom",
        json={"name": "Pi Pico", "quantity": 1, "part_id": part["id"]},
        headers=headers,
    )
    assert resp.status_code == 201
    assert resp.json()["part_id"] == part["id"]

    detail = await client.get(f"/api/parts/{part['slug']}")
    assert detail.json()["usage_count"] == 1


@pytest.mark.asyncio
async def test_bom_part_switch_moves_usage_count(client) -> None:
    project_id = await _create_project(client)
    part_a = await _create_part(client, "Part A")
    part_b = await _create_part(client, "Part B")
    headers = make_auth_header()

    create = await client.post(
        f"/api/projects/{project_id}/bom",
        json={"name": "Part A", "quantity": 1, "part_id": part_a["id"]},
        headers=headers,
    )
    item_id = create.json()["id"]

    a_detail = await client.get(f"/api/parts/{part_a['slug']}")
    assert a_detail.json()["usage_count"] == 1

    # Switch this BOM row from Part A → Part B.
    await client.put(
        f"/api/projects/{project_id}/bom/{item_id}",
        json={"name": "Part B", "quantity": 1, "part_id": part_b["id"]},
        headers=headers,
    )

    a_after = await client.get(f"/api/parts/{part_a['slug']}")
    b_after = await client.get(f"/api/parts/{part_b['slug']}")
    assert a_after.json()["usage_count"] == 0
    assert b_after.json()["usage_count"] == 1


# --- category + family + related parts (issue #135) ------------------


@pytest.mark.asyncio
async def test_create_part_with_category_and_family(client) -> None:
    headers = make_auth_header()
    resp = await client.post(
        "/api/parts",
        json={
            "name": "Raspberry Pi Pico",
            "sku": "PICO",
            "category": "microcontroller",
            "family": "raspberry-pi-pico",
        },
        headers=headers,
    )
    assert resp.status_code == 201, resp.text
    data = resp.json()
    # Category is canonicalised to the curated human label; family is free text.
    assert data["category"] == "Microcontroller"
    assert data["family"] == "raspberry-pi-pico"


@pytest.mark.asyncio
async def test_update_can_change_category_and_family(client) -> None:
    headers = make_auth_header()
    create = await client.post(
        "/api/parts",
        json={"name": "Widget"},
        headers=headers,
    )
    slug = create.json()["slug"]
    upd = await client.put(
        f"/api/parts/{slug}",
        json={
            "category": "sensor",
            "family": "bmp",
            "change_summary": "Categorise widget",
        },
        headers=headers,
    )
    assert upd.status_code == 200, upd.text
    assert upd.json()["category"] == "Sensor"
    assert upd.json()["family"] == "bmp"


@pytest.mark.asyncio
async def test_family_endpoint_groups_siblings(client) -> None:
    headers = make_auth_header()
    for name in ("Pico", "Pico W", "Pico 2"):
        resp = await client.post(
            "/api/parts",
            json={"name": name, "family": "raspberry-pi-pico"},
            headers=headers,
        )
        assert resp.status_code == 201

    detail = await client.get("/api/parts/pico")
    body = detail.json()
    assert body["family"] == "raspberry-pi-pico"
    family_names = {p["name"] for p in body["family_parts"]}
    assert family_names == {"Pico W", "Pico 2"}

    sib = await client.get("/api/parts/pico/family")
    assert sib.status_code == 200
    assert {p["name"] for p in sib.json()} == {"Pico W", "Pico 2"}


@pytest.mark.asyncio
async def test_meta_categories_returns_starter_list(client) -> None:
    resp = await client.get("/api/parts/_meta/categories")
    assert resp.status_code == 200
    data = resp.json()
    # Human-readable curated labels.
    assert "Microcontroller" in data
    assert "Sensor" in data
    assert "Single Board Computer" in data


@pytest.mark.asyncio
async def test_category_canonicalises_slug_and_casing(client) -> None:
    """Variants of a curated category converge onto one canonical label."""
    headers = make_auth_header()
    # Slug-cased + odd-cased inputs all land on "Single Board Computer".
    for name, raw in (("Pi 5", "single-board-computer"),
                      ("Pi 4", "Single Board Computer"),
                      ("Pi 3", "SINGLE BOARD COMPUTER")):
        r = await client.post(
            "/api/parts", json={"name": name, "category": raw}, headers=headers
        )
        assert r.status_code == 201, r.text
        assert r.json()["category"] == "Single Board Computer"
    # The autocomplete list shows it exactly once (deduped by key).
    cats = (await client.get("/api/parts/_meta/categories")).json()
    assert cats.count("Single Board Computer") == 1


@pytest.mark.asyncio
async def test_user_coined_category_is_kept_and_suggested(client) -> None:
    """A category outside the curated list is stored as typed and offered
    for reuse via the autocomplete vocabulary."""
    headers = make_auth_header()
    r = await client.post(
        "/api/parts",
        json={"name": "Mystery Widget", "category": "Flux Capacitor"},
        headers=headers,
    )
    assert r.json()["category"] == "Flux Capacitor"
    cats = (await client.get("/api/parts/_meta/categories")).json()
    assert "Flux Capacitor" in cats
    # A later casing variant reuses the first-writer's label.
    r2 = await client.post(
        "/api/parts",
        json={"name": "Another Widget", "category": "flux capacitor"},
        headers=headers,
    )
    assert r2.json()["category"] == "Flux Capacitor"


@pytest.mark.asyncio
async def test_meta_families_returns_distinct_values(client) -> None:
    headers = make_auth_header()
    for name, family in (("Pico", "raspberry-pi-pico"), ("Pico W", "raspberry-pi-pico"), ("ESP32", "esp32")):
        await client.post(
            "/api/parts",
            json={"name": name, "family": family},
            headers=headers,
        )
    resp = await client.get("/api/parts/_meta/families")
    assert resp.status_code == 200
    fams = resp.json()
    assert "raspberry-pi-pico" in fams
    assert "esp32" in fams
    # most common first
    assert fams[0] == "raspberry-pi-pico"


@pytest.mark.asyncio
async def test_add_and_remove_related_part(client) -> None:
    headers = make_auth_header()
    a = await client.post("/api/parts", json={"name": "Servo"}, headers=headers)
    b = await client.post("/api/parts", json={"name": "Servo Driver"}, headers=headers)
    slug_a = a.json()["slug"]
    slug_b = b.json()["slug"]

    add = await client.post(
        f"/api/parts/{slug_a}/related",
        json={"related_slug": slug_b},
        headers=headers,
    )
    assert add.status_code == 201, add.text
    names = {p["name"] for p in add.json()}
    assert "Servo Driver" in names

    # Symmetric: same link visible from B as well.
    from_b = await client.get(f"/api/parts/{slug_b}/related")
    assert any(p["slug"] == slug_a for p in from_b.json())

    # Idempotent: adding again doesn't duplicate the link.
    add2 = await client.post(
        f"/api/parts/{slug_a}/related",
        json={"related_slug": slug_b},
        headers=headers,
    )
    assert add2.status_code == 201
    assert len(add2.json()) == 1

    # Detail view surfaces it under related_parts.
    detail = await client.get(f"/api/parts/{slug_a}")
    assert any(p["slug"] == slug_b for p in detail.json()["related_parts"])

    # Remove the link.
    rm = await client.delete(
        f"/api/parts/{slug_a}/related/{slug_b}",
        headers=headers,
    )
    assert rm.status_code == 204
    after = await client.get(f"/api/parts/{slug_a}/related")
    assert after.json() == []


@pytest.mark.asyncio
async def test_cannot_relate_part_to_itself(client) -> None:
    headers = make_auth_header()
    p = await client.post("/api/parts", json={"name": "Lonely"}, headers=headers)
    slug = p.json()["slug"]
    resp = await client.post(
        f"/api/parts/{slug}/related",
        json={"related_slug": slug},
        headers=headers,
    )
    assert resp.status_code == 400


@pytest.mark.asyncio
async def test_relate_unknown_slug_404(client) -> None:
    headers = make_auth_header()
    p = await client.post("/api/parts", json={"name": "Widget"}, headers=headers)
    slug = p.json()["slug"]
    resp = await client.post(
        f"/api/parts/{slug}/related",
        json={"related_slug": "does-not-exist"},
        headers=headers,
    )
    assert resp.status_code == 404


@pytest.mark.asyncio
async def test_related_endpoint_enforces_account_age_gate(client, monkeypatch) -> None:
    headers = make_auth_header()
    a = await client.post("/api/parts", json={"name": "Part A"}, headers=headers)
    b = await client.post("/api/parts", json={"name": "Part B"}, headers=headers)
    slug_a = a.json()["slug"]
    slug_b = b.json()["slug"]

    from projects_api import auth as auth_module

    async def _young(username: str, token: str) -> Optional[datetime]:
        return datetime.utcnow() - timedelta(days=3)

    monkeypatch.setattr(auth_module, "fetch_account_created_at", _young)
    auth_module._clear_account_age_cache()

    resp = await client.post(
        f"/api/parts/{slug_a}/related",
        json={"related_slug": slug_b},
        headers=headers,
    )
    assert resp.status_code == 403


@pytest.mark.asyncio
async def test_search_filter_by_category(client) -> None:
    headers = make_auth_header()
    await client.post(
        "/api/parts",
        json={"name": "BMP280", "category": "sensor"},
        headers=headers,
    )
    await client.post(
        "/api/parts",
        json={"name": "Pico", "category": "microcontroller"},
        headers=headers,
    )
    resp = await client.get("/api/parts", params={"category": "sensor"})
    assert resp.status_code == 200
    names = {p["name"] for p in resp.json()}
    assert names == {"BMP280"}


@pytest.mark.asyncio
async def test_usage_count_counts_distinct_projects(client) -> None:
    """Two BOM rows for the same part within a single project = 1, not 2."""
    project_id = await _create_project(client)
    part = await _create_part(client, "Servo")
    headers = make_auth_header()

    for _ in range(2):
        resp = await client.post(
            f"/api/projects/{project_id}/bom",
            json={"name": "Servo", "quantity": 1, "part_id": part["id"]},
            headers=headers,
        )
        assert resp.status_code == 201

    detail = await client.get(f"/api/parts/{part['slug']}")
    assert detail.json()["usage_count"] == 1
