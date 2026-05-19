"""Tests for the parts catalog reports surface (issue #123).

The 14-day account-age gate does NOT apply to reporting — any logged-in
user can flag a part. We still patch ``fetch_account_created_at`` so the
helper that creates the part (POST /api/parts, which IS age-gated) works.
"""

from __future__ import annotations

from datetime import datetime, timedelta
from typing import Optional

import pytest

from .conftest import make_auth_header


@pytest.fixture(autouse=True)
def _old_account(monkeypatch: pytest.MonkeyPatch) -> None:
    """Pretend every account is 5 years old by default."""
    from projects_api import auth as auth_module

    async def _fake_age(username: str, token: str) -> Optional[datetime]:
        return datetime.utcnow() - timedelta(days=365 * 5)

    monkeypatch.setattr(auth_module, "fetch_account_created_at", _fake_age)
    auth_module._clear_account_age_cache()


@pytest.fixture(autouse=True)
def _admin_user(monkeypatch: pytest.MonkeyPatch) -> None:
    """Make ``admin`` an admin so the admin endpoints don't 403."""
    monkeypatch.setenv("ADMIN_USERNAMES", "admin")


async def _make_part(client, name: str = "Widget") -> dict:
    resp = await client.post(
        "/api/parts", json={"name": name}, headers=make_auth_header("alice")
    )
    assert resp.status_code == 201, resp.text
    return resp.json()


@pytest.mark.asyncio
async def test_report_part_creates_row(client) -> None:
    part = await _make_part(client)

    resp = await client.post(
        f"/api/parts/{part['slug']}/report",
        json={"reason": "spam", "note": "Looks like a copy-paste"},
        headers=make_auth_header("bob"),
    )
    assert resp.status_code == 201, resp.text
    data = resp.json()
    assert data["part_id"] == part["id"]
    assert data["reporter_username"] == "bob"
    assert data["reason"] == "spam"
    assert data["note"] == "Looks like a copy-paste"
    assert data["resolved_at"] is None


@pytest.mark.asyncio
async def test_report_rejects_unknown_reason(client) -> None:
    part = await _make_part(client)
    resp = await client.post(
        f"/api/parts/{part['slug']}/report",
        json={"reason": "bogus"},
        headers=make_auth_header("bob"),
    )
    # Pydantic regex rejects -> 422 from validation
    assert resp.status_code == 422


@pytest.mark.asyncio
async def test_one_open_report_per_user_per_part(client) -> None:
    """A second report from the same user on the same part is 409 while
    the first is unresolved."""
    part = await _make_part(client)

    r1 = await client.post(
        f"/api/parts/{part['slug']}/report",
        json={"reason": "wrong"},
        headers=make_auth_header("bob"),
    )
    assert r1.status_code == 201

    r2 = await client.post(
        f"/api/parts/{part['slug']}/report",
        json={"reason": "spam"},
        headers=make_auth_header("bob"),
    )
    assert r2.status_code == 409
    assert "open report" in r2.json()["detail"].lower()


@pytest.mark.asyncio
async def test_different_users_can_both_report(client) -> None:
    part = await _make_part(client)
    for user in ("bob", "carol"):
        r = await client.post(
            f"/api/parts/{part['slug']}/report",
            json={"reason": "duplicate"},
            headers=make_auth_header(user),
        )
        assert r.status_code == 201, r.text


@pytest.mark.asyncio
async def test_report_requires_auth(client) -> None:
    part = await _make_part(client)
    resp = await client.post(
        f"/api/parts/{part['slug']}/report",
        json={"reason": "spam"},
    )
    assert resp.status_code == 401


@pytest.mark.asyncio
async def test_admin_queue_lists_open_reports(client) -> None:
    part = await _make_part(client, "Pi Pico")
    await client.post(
        f"/api/parts/{part['slug']}/report",
        json={"reason": "spam"},
        headers=make_auth_header("bob"),
    )

    # Non-admin gets 403
    forbidden = await client.get(
        "/api/admin/parts/reports", headers=make_auth_header("bob")
    )
    assert forbidden.status_code == 403

    # Admin sees the open report with the embedded part info.
    queue = await client.get(
        "/api/admin/parts/reports", headers=make_auth_header("admin")
    )
    assert queue.status_code == 200, queue.text
    body = queue.json()
    assert body["total"] == 1
    assert len(body["items"]) == 1
    item = body["items"][0]
    assert item["part"]["slug"] == part["slug"]
    assert item["part"]["name"] == "Pi Pico"
    assert item["reason"] == "spam"


@pytest.mark.asyncio
async def test_admin_resolve_marks_resolved(client) -> None:
    part = await _make_part(client)
    create = await client.post(
        f"/api/parts/{part['slug']}/report",
        json={"reason": "wrong"},
        headers=make_auth_header("bob"),
    )
    report_id = create.json()["id"]

    resolve = await client.patch(
        f"/api/admin/parts/reports/{report_id}",
        json={"resolution": "accepted"},
        headers=make_auth_header("admin"),
    )
    assert resolve.status_code == 200, resolve.text
    body = resolve.json()
    assert body["resolved_at"] is not None
    assert body["resolved_by"] == "admin"
    assert body["resolution"] == "accepted"

    # Queue with status=open should now be empty; status=resolved shows it.
    open_q = await client.get(
        "/api/admin/parts/reports?status=open", headers=make_auth_header("admin")
    )
    assert open_q.json()["total"] == 0
    resolved_q = await client.get(
        "/api/admin/parts/reports?status=resolved",
        headers=make_auth_header("admin"),
    )
    assert resolved_q.json()["total"] == 1


@pytest.mark.asyncio
async def test_resolved_report_lets_user_file_a_new_one(client) -> None:
    """After admin resolves a report, the same user may file a fresh one."""
    part = await _make_part(client)
    first = await client.post(
        f"/api/parts/{part['slug']}/report",
        json={"reason": "wrong"},
        headers=make_auth_header("bob"),
    )
    report_id = first.json()["id"]
    await client.patch(
        f"/api/admin/parts/reports/{report_id}",
        json={"resolution": "dismissed"},
        headers=make_auth_header("admin"),
    )
    again = await client.post(
        f"/api/parts/{part['slug']}/report",
        json={"reason": "spam", "note": "another go"},
        headers=make_auth_header("bob"),
    )
    assert again.status_code == 201
