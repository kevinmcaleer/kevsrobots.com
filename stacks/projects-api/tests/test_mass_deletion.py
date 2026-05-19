"""Tests for the mass-deletion guard on the parts wiki (issue #136).

Covers:

* A single PUT that nukes most of a populated part is blocked, the
  account is auto-disabled, and the prior content is left intact.
* Normal small edits don't trip the heuristic.
* When rollback fires across multiple parts the user has edited in the
  last hour, each is restored to the state immediately before the
  user's first edit in that window.
* Admins are exempt from the guard.
* Disabled accounts get 403 on subsequent authenticated requests.
"""

from __future__ import annotations

from datetime import datetime, timedelta
from typing import Optional

import pytest

from .conftest import make_auth_header


@pytest.fixture(autouse=True)
def _old_account(monkeypatch: pytest.MonkeyPatch) -> None:
    """Pretend every account is 5 years old (skip the 14-day gate)."""
    from projects_api import auth as auth_module

    async def _fake_age(username: str, token: str) -> Optional[datetime]:
        return datetime.utcnow() - timedelta(days=365 * 5)

    monkeypatch.setattr(auth_module, "fetch_account_created_at", _fake_age)
    auth_module._clear_account_age_cache()


POPULATED_DESC = (
    "This is a long, well-documented part description. "
    "It contains plenty of useful information about pinouts, "
    "datasheets, suppliers, sample code, and gotchas. "
    "Definitely more than two hundred characters of content "
    "so the populated-min threshold kicks in correctly."
)
assert len(POPULATED_DESC) >= 200


async def _create_populated_part(client, name: str = "Widget") -> dict:
    headers = make_auth_header()
    resp = await client.post(
        "/api/parts",
        json={
            "name": name,
            "sku": "W1",
            "mpn": "MPN-001",
            "description_md": POPULATED_DESC,
            "image_url": "https://example.com/img.png",
            "supplier_url": "https://example.com/buy",
            "supplier_name": "ExampleCo",
            "tags": ["widget", "shiny"],
        },
        headers=headers,
    )
    assert resp.status_code == 201, resp.text
    return resp.json()


# --- Trigger B: single mass-delete --------------------------------------


@pytest.mark.asyncio
async def test_single_mass_delete_is_blocked_and_user_disabled(client) -> None:
    part = await _create_populated_part(client)
    slug = part["slug"]

    # Wipe ~everything — clears desc, image, sku, mpn, tags, suppliers.
    resp = await client.put(
        f"/api/parts/{slug}",
        json={
            "description_md": "",
            "image_url": "",
            "sku": "",
            "mpn": "",
            "tags": [],
            "suppliers": [],
            "change_summary": "cleanup",
        },
        headers=make_auth_header(),
    )
    assert resp.status_code == 403, resp.text
    assert "mass deletion" in resp.json()["detail"].lower()

    # The content is unchanged — the destructive write did not land.
    after = await client.get(f"/api/parts/{slug}")
    assert after.json()["description_md"] == POPULATED_DESC
    assert after.json()["sku"] == "W1"
    assert len(after.json()["suppliers"]) == 1

    # A subsequent authenticated request from this user gets 403.
    follow = await client.post(
        "/api/parts",
        json={"name": "Another"},
        headers=make_auth_header(),
    )
    assert follow.status_code == 403
    assert "disabled" in follow.json()["detail"].lower()


@pytest.mark.asyncio
async def test_normal_small_edit_does_not_trip_guard(client) -> None:
    part = await _create_populated_part(client)
    slug = part["slug"]

    # Add a single tag — non-destructive.
    resp = await client.put(
        f"/api/parts/{slug}",
        json={
            "tags": ["widget", "shiny", "blue"],
            "change_summary": "add tag",
        },
        headers=make_auth_header(),
    )
    assert resp.status_code == 200, resp.text
    assert sorted(resp.json()["tags"]) == ["blue", "shiny", "widget"]

    # And the user is NOT disabled.
    follow = await client.post(
        "/api/parts",
        json={"name": "Another Widget"},
        headers=make_auth_header(),
    )
    assert follow.status_code == 201


@pytest.mark.asyncio
async def test_clearing_three_structured_fields_trips_guard(client) -> None:
    """Even if content-length isn't catastrophically reduced, clearing
    3+ structured fields in one edit fires Trigger B."""
    part = await _create_populated_part(client)
    slug = part["slug"]

    # Clear sku, mpn, and image_url — three structured fields, but
    # description stays intact so the content-fraction rule wouldn't
    # fire on its own.
    resp = await client.put(
        f"/api/parts/{slug}",
        json={
            "sku": "",
            "mpn": "",
            "image_url": "",
            "change_summary": "tidy",
        },
        headers=make_auth_header(),
    )
    assert resp.status_code == 403
    assert "mass deletion" in resp.json()["detail"].lower()


# --- Rollback semantics -------------------------------------------------


@pytest.mark.asyncio
async def test_rollback_restores_prior_content_across_multiple_parts(
    client,
) -> None:
    """When the heuristic fires, every part this user edited in the
    last hour is restored to its pre-edit state."""
    # Part A — created by someone else, then the bad user makes a tiny
    # benign edit before going on a rampage.
    other_headers = make_auth_header(username="otheruser")
    part_a_create = await client.post(
        "/api/parts",
        json={
            "name": "Part A",
            "description_md": POPULATED_DESC,
            "tags": ["a", "b"],
        },
        headers=other_headers,
    )
    part_a = part_a_create.json()

    # Bad user edits Part A (small, non-destructive change).
    bad_headers = make_auth_header(username="baduser")
    await client.put(
        f"/api/parts/{part_a['slug']}",
        json={
            "tags": ["a", "b", "c"],
            "change_summary": "add tag",
        },
        headers=bad_headers,
    )

    # Bad user also touches Part B with a benign edit.
    part_b_create = await client.post(
        "/api/parts",
        json={
            "name": "Part B",
            "description_md": POPULATED_DESC,
            "sku": "B1",
        },
        headers=other_headers,
    )
    part_b = part_b_create.json()
    await client.put(
        f"/api/parts/{part_b['slug']}",
        json={"sku": "B2", "change_summary": "fix sku"},
        headers=bad_headers,
    )

    # Now the bad user triggers a mass-delete on Part A.
    resp = await client.put(
        f"/api/parts/{part_a['slug']}",
        json={
            "description_md": "",
            "tags": [],
            "change_summary": "wipe",
        },
        headers=bad_headers,
    )
    assert resp.status_code == 403

    # Part A is restored to the pre-bad-user state — original tags
    # ["a", "b"], NOT the tag-added ["a", "b", "c"].
    a_after = await client.get(f"/api/parts/{part_a['slug']}")
    assert sorted(a_after.json()["tags"]) == ["a", "b"]
    assert a_after.json()["description_md"] == POPULATED_DESC

    # Part B is restored to "B1" (pre-bad-user sku), not the "B2" the
    # bad user pushed.
    b_after = await client.get(f"/api/parts/{part_b['slug']}")
    assert b_after.json()["sku"] == "B1"


@pytest.mark.asyncio
async def test_admin_is_exempt_from_mass_deletion_guard(client) -> None:
    """The user `kev` is the default admin in settings — their
    mass-edit must NOT be blocked."""
    headers = make_auth_header(username="kev")
    create = await client.post(
        "/api/parts",
        json={
            "name": "Admin Widget",
            "description_md": POPULATED_DESC,
            "sku": "AW1",
            "mpn": "AW-MPN",
            "image_url": "https://example.com/img.png",
            "tags": ["a", "b"],
        },
        headers=headers,
    )
    slug = create.json()["slug"]

    resp = await client.put(
        f"/api/parts/{slug}",
        json={
            "description_md": "",
            "image_url": "",
            "sku": "",
            "mpn": "",
            "tags": [],
            "change_summary": "admin cleanup",
        },
        headers=headers,
    )
    # Admin's mass-edit goes through.
    assert resp.status_code == 200, resp.text
    assert resp.json()["description_md"] in (None, "")


# --- Trigger A: rate of destructive edits over the window -------------


@pytest.mark.asyncio
async def test_trigger_a_fires_after_repeated_destructive_edits(
    client, session, monkeypatch
) -> None:
    """Lower the threshold to 2 destructive edits so we can exercise the
    rate trigger without writing dozens of fixtures."""
    from projects_api import mass_deletion as md_mod

    monkeypatch.setattr(md_mod, "DESTRUCTIVE_EDIT_LIMIT", 2)

    bad_headers = make_auth_header(username="ratey")
    other_headers = make_auth_header(username="other")

    # Build 3 populated parts owned by `other`, then have `ratey` make
    # destructive edits on the first two — quietly enough that they
    # individually fire Trigger B too. We want to *also* show that
    # after a second historical destructive edit, even a non-destructive
    # follow-up fires Trigger A.
    parts = []
    for i in range(3):
        resp = await client.post(
            "/api/parts",
            json={
                "name": f"Hist Part {i}",
                "description_md": POPULATED_DESC,
                "sku": f"S{i}",
                "mpn": f"M{i}",
                "image_url": "https://example.com/img.png",
                "tags": ["a", "b"],
            },
            headers=other_headers,
        )
        assert resp.status_code == 201
        parts.append(resp.json())

    # Manually seed 2 destructive revisions in `ratey`'s history. Direct
    # ORM writes — we're synthesising history that pre-dates the current
    # request.
    from projects_api.models import PartRevision

    for p in parts[:2]:
        session.add(
            PartRevision(
                part_id=p["id"],
                author="ratey",
                change_summary="seed destructive",
                name=p["name"],
                sku=None,
                mpn=None,
                description_md="",
                image_url=None,
                tags=[],
                suppliers_json=[],
            )
        )
    await session.commit()

    # Now `ratey` attempts a tiny, benign edit on parts[2]. Trigger B
    # does NOT fire (small edit), but Trigger A does because we already
    # have 2 destructive revisions on record (>= the lowered limit).
    resp = await client.put(
        f"/api/parts/{parts[2]['slug']}",
        json={
            "tags": ["a", "b", "c"],
            "change_summary": "innocent tag add",
        },
        headers=bad_headers,
    )
    assert resp.status_code == 403
    assert "mass deletion" in resp.json()["detail"].lower()


# --- Disabled-account auth gate ----------------------------------------


@pytest.mark.asyncio
async def test_disabled_account_blocked_on_all_authenticated_endpoints(
    client, session
) -> None:
    """Once a user is auto-disabled, every authenticated endpoint
    returns 403."""
    from projects_api.models import User

    session.add(User(username="disabled", is_disabled=True, disabled_reason="test"))
    await session.commit()

    headers = make_auth_header(username="disabled")
    # /api/projects requires get_optional_user; disabled accounts should
    # still 403 there even though anon access works.
    resp = await client.post(
        "/api/projects",
        json={"title": "Should be blocked"},
        headers=headers,
    )
    assert resp.status_code == 403
