"""Tests for the T&Cs acceptance gate (terms-gate).

Covers:

* ``GET /api/auth/me`` exposes the three terms fields with the right
  defaults and the current server-side version.
* ``POST /api/users/me/accept-terms`` happy path persists timestamp +
  version.
* Mismatched / stale version on accept-terms -> 400.
* Gated write endpoint without acceptance -> 403 with
  ``detail: "terms_not_accepted"`` and ``current_version``.
* After accepting, the same write succeeds.
* Bumping ``settings.current_terms_version`` invalidates an already-
  accepted record until the user re-accepts.
* GET endpoints are NEVER blocked by the gate.

These tests intentionally opt OUT of the default conftest bypass (which
patches ``require_terms_accepted`` / ``require_terms_accepted_aged`` to
no-ops so the existing 290-ish tests don't all need to first call
accept-terms). The fixture below restores the live dependencies on the
test app.
"""

from __future__ import annotations

import pytest

from .conftest import make_auth_header


@pytest.fixture(autouse=True)
def _restore_gate(client):
    """Strip the conftest bypass so the real terms gate runs in these tests."""
    from projects_api import auth as auth_module

    # ``client`` is the async fixture's AsyncClient; the underlying ASGI app
    # carries the dependency_overrides we want to clear. AsyncClient exposes
    # the ASGI transport as ``_transport`` (httpx) which holds the app.
    app = client._transport.app
    for dep in (
        auth_module.require_terms_accepted,
        auth_module.require_terms_accepted_aged,
    ):
        app.dependency_overrides.pop(dep, None)
    yield


@pytest.mark.asyncio
async def test_auth_me_exposes_terms_fields_with_defaults(
    client, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A fresh user has no acceptance, and current_version mirrors settings."""
    from projects_api import auth as auth_module
    from projects_api.routers import auth as auth_router

    async def _fake_me(token: str):
        return {"id": 1, "email": "x@y.z"}

    monkeypatch.setattr(auth_module, "fetch_chatter_me", _fake_me)
    monkeypatch.setattr(auth_router, "fetch_chatter_me", _fake_me)

    resp = await client.get("/api/auth/me", headers=make_auth_header("alice"))
    assert resp.status_code == 200
    body = resp.json()
    assert body["terms_accepted_at"] is None
    assert body["terms_accepted_version"] is None
    assert body["current_terms_version"] == "1.0"


@pytest.mark.asyncio
async def test_accept_terms_happy_path_persists(
    client, monkeypatch: pytest.MonkeyPatch
) -> None:
    """POSTing the current version succeeds and is reflected on /me."""
    from projects_api import auth as auth_module
    from projects_api.routers import auth as auth_router

    async def _fake_me(token: str):
        return {"id": 1, "email": "x@y.z"}

    monkeypatch.setattr(auth_module, "fetch_chatter_me", _fake_me)
    monkeypatch.setattr(auth_router, "fetch_chatter_me", _fake_me)

    headers = make_auth_header("alice")
    resp = await client.post(
        "/api/users/me/accept-terms",
        json={"version": "1.0"},
        headers=headers,
    )
    assert resp.status_code == 200, resp.text
    body = resp.json()
    assert body["version"] == "1.0"
    assert body["accepted_at"]  # ISO timestamp present

    # /me now surfaces the acceptance.
    me = await client.get("/api/auth/me", headers=headers)
    assert me.status_code == 200
    me_body = me.json()
    assert me_body["terms_accepted_version"] == "1.0"
    assert me_body["terms_accepted_at"] is not None


@pytest.mark.asyncio
async def test_accept_terms_rejects_stale_version(client) -> None:
    """A version that doesn't match settings is 400 (stale tab guard)."""
    resp = await client.post(
        "/api/users/me/accept-terms",
        json={"version": "0.9"},
        headers=make_auth_header("alice"),
    )
    assert resp.status_code == 400
    assert "does not match" in resp.text


@pytest.mark.asyncio
async def test_write_without_acceptance_returns_terms_not_accepted(
    client,
) -> None:
    """A gated POST without prior acceptance -> 403 with the contract shape."""
    resp = await client.post(
        "/api/projects",
        json={"title": "Should Be Blocked", "short_description": "x"},
        headers=make_auth_header("alice"),
    )
    assert resp.status_code == 403
    body = resp.json()
    # FastAPI wraps the dict we raise into ``detail``.
    detail = body.get("detail")
    assert isinstance(detail, dict)
    assert detail.get("detail") == "terms_not_accepted"
    assert detail.get("current_version") == "1.0"


@pytest.mark.asyncio
async def test_write_after_acceptance_succeeds(client) -> None:
    """Accept then retry: the same POST now lands."""
    headers = make_auth_header("alice")

    # First call confirms gate is active.
    blocked = await client.post(
        "/api/projects",
        json={"title": "Gate Test"},
        headers=headers,
    )
    assert blocked.status_code == 403

    # Accept the current version.
    acc = await client.post(
        "/api/users/me/accept-terms",
        json={"version": "1.0"},
        headers=headers,
    )
    assert acc.status_code == 200

    # Retry the write.
    ok = await client.post(
        "/api/projects",
        json={"title": "Gate Test"},
        headers=headers,
    )
    assert ok.status_code == 201, ok.text
    assert ok.json()["title"] == "Gate Test"


@pytest.mark.asyncio
async def test_version_bump_invalidates_prior_acceptance(
    client, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Bumping current_terms_version forces a re-accept."""
    headers = make_auth_header("alice")

    # Accept 1.0 then write — works.
    acc = await client.post(
        "/api/users/me/accept-terms",
        json={"version": "1.0"},
        headers=headers,
    )
    assert acc.status_code == 200
    ok = await client.post(
        "/api/projects", json={"title": "Pre-bump"}, headers=headers
    )
    assert ok.status_code == 201

    # Bump the server's required version. We patch get_settings so every
    # consumer (auth.py + routers/auth.py + routers/users.py) sees 1.1.
    # Wrap the real Settings rather than mocking it — the auth code reads
    # jwt_secret/jwt_algorithm too, so we need all those fields intact.
    from projects_api import auth as auth_module
    from projects_api import config as config_module
    from projects_api.routers import users as users_router

    real_settings = config_module.get_settings()

    def _fake_get_settings():
        # Re-build a fresh Settings each call (mirrors the real factory)
        # but override the one attribute we care about. ``model_copy``
        # preserves every other field including jwt_secret.
        s = config_module.Settings()
        return s.model_copy(update={"current_terms_version": "1.1"})

    monkeypatch.setattr(config_module, "get_settings", _fake_get_settings)
    monkeypatch.setattr(auth_module, "get_settings", _fake_get_settings)
    monkeypatch.setattr(users_router, "get_settings", _fake_get_settings)
    # Sanity: avoid an unused-name warning.
    assert real_settings.current_terms_version == "1.0"

    # Now the next write should 403 — the user's stored 1.0 acceptance is
    # stale.
    blocked = await client.post(
        "/api/projects", json={"title": "Post-bump"}, headers=headers
    )
    assert blocked.status_code == 403
    detail = blocked.json()["detail"]
    assert detail["detail"] == "terms_not_accepted"
    assert detail["current_version"] == "1.1"

    # Accepting 1.0 again is rejected — only the current 1.1 is valid.
    stale = await client.post(
        "/api/users/me/accept-terms",
        json={"version": "1.0"},
        headers=headers,
    )
    assert stale.status_code == 400

    # Accept 1.1 → write unblocks again.
    fresh = await client.post(
        "/api/users/me/accept-terms",
        json={"version": "1.1"},
        headers=headers,
    )
    assert fresh.status_code == 200
    ok2 = await client.post(
        "/api/projects", json={"title": "Post-bump 2"}, headers=headers
    )
    assert ok2.status_code == 201


@pytest.mark.asyncio
async def test_read_endpoints_are_never_blocked(client) -> None:
    """GET endpoints don't go through the gate, even for unaccepted users."""
    # No acceptance — but reads must still work.
    resp = await client.get("/api/projects")
    assert resp.status_code == 200
    # Featured list is also a read.
    resp2 = await client.get("/api/projects/featured")
    assert resp2.status_code == 200


@pytest.mark.asyncio
async def test_report_endpoint_remains_open(client) -> None:
    """The parts-moderation report POST is intentionally NOT gated.

    Abuse reporting must stay frictionless — users who haven't accepted
    the T&Cs are still allowed to flag a part. Here we just verify the
    handler is reachable past the gate (the actual 404 for a non-existent
    part is fine — what we're proving is that we didn't 403 first).
    """
    resp = await client.post(
        "/api/parts/no-such-part/report",
        json={"reason": "spam"},
        headers=make_auth_header("alice"),
    )
    # 404 (no part) — not 403 (terms gate).
    assert resp.status_code == 404
