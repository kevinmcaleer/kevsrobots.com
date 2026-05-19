"""Tests for ``GET /api/auth/me`` (issue #139).

The Chatter ``/api/me`` lookup is monkey-patched so we don't make real
HTTP calls. We exercise the happy path, the not-logged-in 401 path, the
admin flag, the no-store cache header, and the Chatter-unreachable
fallback (endpoint must still 200 with nulls rather than 500).
"""

from __future__ import annotations

from typing import Any, Optional

import pytest

from .conftest import make_auth_header


def _patch_chatter(monkeypatch: pytest.MonkeyPatch, payload: Optional[dict[str, Any]]) -> None:
    """Replace ``fetch_chatter_me`` in both the auth module and the router
    namespace it was imported into."""
    from projects_api import auth as auth_module
    from projects_api.routers import auth as auth_router

    async def _fake_me(token: str) -> Optional[dict[str, Any]]:
        return payload

    monkeypatch.setattr(auth_module, "fetch_chatter_me", _fake_me)
    monkeypatch.setattr(auth_router, "fetch_chatter_me", _fake_me)


@pytest.mark.asyncio
async def test_auth_me_unauthenticated_returns_401(client) -> None:
    """No session cookie or bearer header -> 401."""
    resp = await client.get("/api/auth/me")
    assert resp.status_code == 401


@pytest.mark.asyncio
async def test_auth_me_invalid_token_returns_401(client) -> None:
    """Garbage Authorization header -> 401."""
    resp = await client.get(
        "/api/auth/me",
        headers={"Authorization": "Bearer not-a-real-jwt"},
    )
    assert resp.status_code == 401


@pytest.mark.asyncio
async def test_auth_me_logged_in_returns_full_profile(
    client, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Logged-in user with reachable Chatter -> 200 with merged profile."""
    _patch_chatter(
        monkeypatch,
        {
            "id": 42,
            "username": "kevin",
            "email": "kevin@example.com",
            "account_created_at": "2024-04-01T12:00:00Z",
            "avatar_url": None,
        },
    )

    resp = await client.get("/api/auth/me", headers=make_auth_header("kevin"))
    assert resp.status_code == 200

    body = resp.json()
    assert set(body.keys()) == {
        "id",
        "username",
        "email",
        "is_admin",
        "created_at",
        "avatar_url",
    }
    assert body["id"] == 42
    assert body["username"] == "kevin"
    assert body["email"] == "kevin@example.com"
    assert body["is_admin"] is False
    assert body["created_at"] == "2024-04-01T12:00:00Z"
    assert body["avatar_url"] is None

    # Cache-Control: no-store is required (per-user, never cacheable).
    assert resp.headers.get("cache-control") == "no-store"


@pytest.mark.asyncio
async def test_auth_me_admin_flag_reflects_settings(
    client, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Username in ADMIN_USERNAMES -> is_admin: true."""
    monkeypatch.setenv("ADMIN_USERNAMES", "kevin,boss")
    _patch_chatter(monkeypatch, {"id": 1, "email": "k@example.com"})

    resp = await client.get("/api/auth/me", headers=make_auth_header("kevin"))
    assert resp.status_code == 200
    assert resp.json()["is_admin"] is True

    resp2 = await client.get("/api/auth/me", headers=make_auth_header("someone-else"))
    assert resp2.status_code == 200
    assert resp2.json()["is_admin"] is False


@pytest.mark.asyncio
async def test_auth_me_falls_back_when_chatter_unreachable(
    client, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Chatter outage -> 200 with nulls for enriched fields, not 500."""
    _patch_chatter(monkeypatch, None)

    resp = await client.get("/api/auth/me", headers=make_auth_header("kevin"))
    assert resp.status_code == 200
    body = resp.json()
    assert body["username"] == "kevin"
    assert body["id"] is None
    assert body["email"] is None
    assert body["created_at"] is None
    assert body["avatar_url"] is None
