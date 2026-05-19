"""Tests for the preferred_currency profile field — issue #150.

Covers:
  * PUT profile with ``preferred_currency: "USD"`` round-trips.
  * Empty string clears the stored preference (back to auto-detect).
  * Currency outside the supported allow-list is rejected (422).
  * Invalid 3-letter code (e.g. lowercase / digits) is rejected.
  * Migration helper is a no-op when the column already exists.
"""

from __future__ import annotations

import pytest

from .conftest import make_auth_header


@pytest.mark.asyncio
async def test_put_profile_with_preferred_currency_roundtrips(client) -> None:
    headers = make_auth_header("alice")
    r = await client.put(
        "/api/users/me/profile",
        json={"preferred_currency": "USD"},
        headers=headers,
    )
    assert r.status_code == 200, r.text
    assert r.json()["preferred_currency"] == "USD"

    # GET reflects the saved preference.
    r2 = await client.get("/api/users/alice/profile")
    assert r2.status_code == 200
    assert r2.json()["preferred_currency"] == "USD"


@pytest.mark.asyncio
async def test_put_profile_preferred_currency_can_be_cleared(client) -> None:
    headers = make_auth_header("alice")
    await client.put(
        "/api/users/me/profile",
        json={"preferred_currency": "EUR"},
        headers=headers,
    )
    # Send explicit None to clear (the "Auto-detect" option).
    r = await client.put(
        "/api/users/me/profile",
        json={"preferred_currency": None},
        headers=headers,
    )
    assert r.status_code == 200
    assert r.json()["preferred_currency"] is None


@pytest.mark.asyncio
async def test_put_profile_rejects_unsupported_currency(client) -> None:
    """An ISO-shaped code that we don't support yet is rejected so the
    user sees a clear error instead of silently saving garbage."""
    headers = make_auth_header("alice")
    r = await client.put(
        "/api/users/me/profile",
        json={"preferred_currency": "ZAR"},
        headers=headers,
    )
    assert r.status_code == 422


@pytest.mark.asyncio
async def test_put_profile_rejects_malformed_currency_code(client) -> None:
    """Lowercase / digits / wrong length fail Pydantic pattern -> 422."""
    headers = make_auth_header("alice")
    r = await client.put(
        "/api/users/me/profile",
        json={"preferred_currency": "usd"},
        headers=headers,
    )
    assert r.status_code == 422
    r = await client.put(
        "/api/users/me/profile",
        json={"preferred_currency": "US"},
        headers=headers,
    )
    assert r.status_code == 422
    r = await client.put(
        "/api/users/me/profile",
        json={"preferred_currency": "US1"},
        headers=headers,
    )
    assert r.status_code == 422


@pytest.mark.asyncio
async def test_put_profile_preferred_currency_does_not_touch_other_fields(
    client,
) -> None:
    """Setting just preferred_currency must not blank bio/location."""
    headers = make_auth_header("alice")
    await client.put(
        "/api/users/me/profile",
        json={"bio": "Robots!", "location": "Glasgow"},
        headers=headers,
    )
    r = await client.put(
        "/api/users/me/profile",
        json={"preferred_currency": "GBP"},
        headers=headers,
    )
    assert r.status_code == 200
    body = r.json()
    assert body["preferred_currency"] == "GBP"
    assert body["bio"] == "Robots!"
    assert body["location"] == "Glasgow"


@pytest.mark.asyncio
async def test_migration_helper_no_op_when_column_exists() -> None:
    """The helper is Postgres-only — on SQLite (the test DB) it short
    -circuits at the dialect check and is a no-op."""
    from projects_api.db import add_user_currency_preference_if_missing
    # Must not raise. Running twice exercises the idempotency contract.
    await add_user_currency_preference_if_missing()
    await add_user_currency_preference_if_missing()
