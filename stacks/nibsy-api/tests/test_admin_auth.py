"""Auth-gating tests for ``/api/admin/*`` (#158).

Every admin route must return:

* ``401`` when the request has no auth at all
* ``403`` when the request is logged in as a non-admin
* ``2xx`` when the request is logged in as an admin

These tests don't care about the *content* of the underlying operation
(other suites cover that) — only that the gate works. The Chatter
``/api/me`` round-trip is shorted out by the JWT being locally
verifiable: ``conftest._env`` sets ``JWT_SECRET=test-secret`` (mirroring
projects-api), and the helper below signs tokens with that same key.

If ``fetch_chatter_me`` ever becomes load-bearing on the auth path,
this file will need to monkeypatch it to return a known dict.
"""

from __future__ import annotations

from typing import Iterable

import pytest
from jose import jwt


# Every admin route worth gating. ``categorise/export`` is a GET; the
# rest are POSTs. ``ingest`` skipped from happy-path because it relies on
# a populated NIBSY_DATA_DIR (the conftest fixture sets it, but we keep
# the parametrisation simple — the auth tests only care about 401/403/2xx,
# not the response body shape).
ADMIN_ROUTES: list[tuple[str, str, object]] = [
    ("GET", "/api/admin/status", None),
    ("POST", "/api/admin/ingest", None),
    ("POST", "/api/admin/ingest-remote", None),
    ("POST", "/api/admin/recompute-trending", None),
    ("GET", "/api/admin/categorise/export", None),
    ("POST", "/api/admin/categorise/import", []),  # body required
    ("POST", "/api/admin/regenerate-recommendations", None),
]


def _setup_admin_env(monkeypatch: pytest.MonkeyPatch, admins: Iterable[str]) -> None:
    """Configure ADMIN_USERNAMES for a test.

    ``conftest._env`` runs first (autouse), so this just appends the
    admin allow-list on top.
    """

    monkeypatch.setenv("ADMIN_USERNAMES", ",".join(admins))
    monkeypatch.setenv("JWT_SECRET", "test-secret")


def _auth_header(username: str) -> dict[str, str]:
    """Sign a JWT for ``username`` using the test secret."""

    token = jwt.encode({"sub": username}, "test-secret", algorithm="HS256")
    return {"Authorization": f"Bearer {token}"}


async def _call(client, method: str, path: str, *, headers=None, body=None):
    """Tiny dispatch helper so we can parametrise across verbs."""

    headers = headers or {}
    if method == "GET":
        return await client.get(path, headers=headers)
    if method == "POST":
        if body is None:
            return await client.post(path, headers=headers)
        return await client.post(path, json=body, headers=headers)
    raise ValueError(f"Unsupported method {method!r}")


@pytest.mark.asyncio
@pytest.mark.parametrize("method,path,body", ADMIN_ROUTES)
async def test_admin_routes_reject_anonymous(
    client, monkeypatch, method: str, path: str, body
) -> None:
    """No cookie + no Authorization header -> 401."""

    _setup_admin_env(monkeypatch, ["kev"])
    resp = await _call(client, method, path, body=body)
    assert resp.status_code == 401, (
        f"{method} {path} should reject anonymous callers, "
        f"got {resp.status_code}: {resp.text[:200]}"
    )


@pytest.mark.asyncio
@pytest.mark.parametrize("method,path,body", ADMIN_ROUTES)
async def test_admin_routes_reject_non_admin(
    client, monkeypatch, method: str, path: str, body
) -> None:
    """Logged in but not in ADMIN_USERNAMES -> 403."""

    _setup_admin_env(monkeypatch, ["kev"])
    resp = await _call(
        client, method, path, headers=_auth_header("alice"), body=body
    )
    assert resp.status_code == 403, (
        f"{method} {path} should reject non-admin 'alice', "
        f"got {resp.status_code}: {resp.text[:200]}"
    )


@pytest.mark.asyncio
@pytest.mark.parametrize("method,path,body", ADMIN_ROUTES)
async def test_admin_routes_allow_admin(
    client, monkeypatch, method: str, path: str, body
) -> None:
    """Logged in as a user in ADMIN_USERNAMES -> 2xx (never 401/403)."""

    _setup_admin_env(monkeypatch, ["kev"])
    resp = await _call(
        client, method, path, headers=_auth_header("kev"), body=body
    )
    assert resp.status_code not in (401, 403), (
        f"{method} {path} should accept admin 'kev', "
        f"got {resp.status_code}: {resp.text[:200]}"
    )
    # Most routes return 200; ingest may return 400 if no data dir is
    # configured, but conftest sets one — so we expect 200 across the board.
    # Keep the assertion loose (2xx) so individual route behaviour isn't
    # over-coupled to this auth test.
    assert 200 <= resp.status_code < 300, (
        f"{method} {path} should succeed for admin, "
        f"got {resp.status_code}: {resp.text[:200]}"
    )
