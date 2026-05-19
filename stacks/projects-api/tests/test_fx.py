"""Tests for the FX cache + ``/api/fx/convert`` endpoint — issue #150.

Mocks the upstream rates API entirely. We never hit the real network.

Covers:
  * ``get_rate`` returns the expected float for a normal pair.
  * Cache hit avoids a second HTTP call within TTL.
  * Cache stale + upstream down -> stale value returned (graceful
    degradation).
  * Unknown currency -> returns None, never throws.
  * ``/api/fx/convert`` endpoint round-trip.
"""

from __future__ import annotations

from typing import Any

import pytest

from projects_api import fx as fx_module
from projects_api.fx import clear_fx_cache, convert, get_rate


# ---------------------------------------------------------------------------
# Fake httpx.AsyncClient (mirrors the pattern in test_badge_counters.py)
# ---------------------------------------------------------------------------


class _FakeResponse:
    def __init__(self, status_code: int, payload: Any) -> None:
        self.status_code = status_code
        self._payload = payload

    def json(self) -> Any:
        return self._payload


class _FakeAsyncClient:
    """Minimal stand-in for ``httpx.AsyncClient``.

    Returns the response produced by ``response_factory(url)`` for every
    GET, and counts calls so tests can assert the cache short-circuits.
    """

    def __init__(self, response_factory) -> None:
        self._factory = response_factory
        self.calls: list[str] = []

    async def __aenter__(self) -> "_FakeAsyncClient":
        return self

    async def __aexit__(self, *exc_info: Any) -> None:
        return None

    async def get(self, url: str) -> _FakeResponse:
        self.calls.append(url)
        return self._factory(url)


def _install_fake_client(monkeypatch: pytest.MonkeyPatch, fake: _FakeAsyncClient) -> None:
    def _factory(*_args: Any, **_kwargs: Any) -> _FakeAsyncClient:
        return fake

    monkeypatch.setattr(fx_module.httpx, "AsyncClient", _factory)


def _ok_rates_response(url: str) -> _FakeResponse:
    # Mock open.er-api.com response shape; base is GBP.
    return _FakeResponse(200, {
        "result": "success",
        "base_code": "GBP",
        "rates": {
            "GBP": 1.0,
            "USD": 1.25,
            "EUR": 1.15,
            "JPY": 190.0,
            "AUD": 1.85,
            "CAD": 1.70,
        },
    })


@pytest.fixture(autouse=True)
def _reset_cache():
    clear_fx_cache()
    yield
    clear_fx_cache()


# ---------------------------------------------------------------------------
# Unit tests on the module
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_get_rate_normal_pair(monkeypatch) -> None:
    fake = _FakeAsyncClient(_ok_rates_response)
    _install_fake_client(monkeypatch, fake)

    rate = await get_rate("GBP", "USD")
    assert rate == pytest.approx(1.25)
    # Round-trip a non-base pair: USD -> EUR via GBP.
    rate = await get_rate("USD", "EUR")
    assert rate == pytest.approx(1.15 / 1.25)


@pytest.mark.asyncio
async def test_identity_rate_is_one_without_http_call(monkeypatch) -> None:
    fake = _FakeAsyncClient(_ok_rates_response)
    _install_fake_client(monkeypatch, fake)
    rate = await get_rate("USD", "USD")
    assert rate == 1.0
    # Same-ccy short-circuits before the cache fetch.
    assert fake.calls == []


@pytest.mark.asyncio
async def test_cache_hit_avoids_second_http_call(monkeypatch) -> None:
    fake = _FakeAsyncClient(_ok_rates_response)
    _install_fake_client(monkeypatch, fake)

    r1 = await get_rate("GBP", "USD")
    r2 = await get_rate("GBP", "EUR")
    r3 = await get_rate("USD", "EUR")
    assert r1 is not None and r2 is not None and r3 is not None
    # Exactly one upstream fetch despite three rate lookups.
    assert len(fake.calls) == 1


@pytest.mark.asyncio
async def test_stale_cache_served_when_upstream_down(monkeypatch) -> None:
    """Prime the cache, then knock the upstream over and force a refresh
    by expiring the TTL. The stale value should still be returned."""
    fake_ok = _FakeAsyncClient(_ok_rates_response)
    _install_fake_client(monkeypatch, fake_ok)
    primed = await get_rate("GBP", "USD")
    assert primed == pytest.approx(1.25)

    # Expire the TTL by rewinding the cache entry's timestamp.
    from datetime import datetime, timedelta
    entry = fx_module._cache[fx_module.DEFAULT_BASE]
    entry.fetched_at = datetime.utcnow() - timedelta(days=2)

    # Swap the client for one that always 500s.
    def _down(url: str) -> _FakeResponse:
        return _FakeResponse(500, {"error": "down"})
    fake_down = _FakeAsyncClient(_down)
    _install_fake_client(monkeypatch, fake_down)

    rate = await get_rate("GBP", "USD")
    # We get the stale value back, not None.
    assert rate == pytest.approx(1.25)
    # Upstream was called once and failed; cache fell back.
    assert len(fake_down.calls) == 1


@pytest.mark.asyncio
async def test_unknown_currency_returns_none(monkeypatch) -> None:
    fake = _FakeAsyncClient(_ok_rates_response)
    _install_fake_client(monkeypatch, fake)
    assert await get_rate("GBP", "XYZ") is None
    assert await get_rate("NOT", "USD") is None
    assert await get_rate("", "USD") is None
    assert await get_rate("GBP", None) is None  # type: ignore[arg-type]


@pytest.mark.asyncio
async def test_unknown_currency_never_throws_when_upstream_down(
    monkeypatch,
) -> None:
    def _explode(url: str) -> _FakeResponse:
        raise RuntimeError("connection refused")
    fake = _FakeAsyncClient(_explode)
    _install_fake_client(monkeypatch, fake)
    # Cold cache + upstream raising must not propagate.
    assert await get_rate("GBP", "USD") is None
    assert await convert(10, "GBP", "USD") is None


@pytest.mark.asyncio
async def test_convert_returns_full_payload(monkeypatch) -> None:
    fake = _FakeAsyncClient(_ok_rates_response)
    _install_fake_client(monkeypatch, fake)
    result = await convert(10.0, "GBP", "USD")
    assert result is not None
    assert result.source_amount == 10.0
    assert result.source_currency == "GBP"
    assert result.target_currency == "USD"
    assert result.amount == 12.5
    assert result.rate == pytest.approx(1.25)


# ---------------------------------------------------------------------------
# /api/fx/convert endpoint
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_fx_convert_endpoint_roundtrip(client, monkeypatch) -> None:
    fake = _FakeAsyncClient(_ok_rates_response)
    _install_fake_client(monkeypatch, fake)
    r = await client.get("/api/fx/convert?from=GBP&to=USD&amount=12.5")
    assert r.status_code == 200, r.text
    body = r.json()
    assert body["from_currency"] == "GBP"
    assert body["to_currency"] == "USD"
    assert body["source_amount"] == 12.5
    assert body["amount"] == pytest.approx(15.62, abs=0.01)
    assert body["rate"] == pytest.approx(1.25)
    assert body["fetched_at"] is not None


@pytest.mark.asyncio
async def test_fx_convert_endpoint_rejects_bad_currency(client) -> None:
    r = await client.get("/api/fx/convert?from=GBP&to=XYZ&amount=10")
    assert r.status_code == 422
    r = await client.get("/api/fx/convert?from=GB&to=USD&amount=10")
    assert r.status_code == 422
    r = await client.get("/api/fx/convert?from=GBP&to=US1&amount=10")
    assert r.status_code == 422


@pytest.mark.asyncio
async def test_fx_convert_endpoint_returns_null_rate_when_upstream_cold(
    client, monkeypatch
) -> None:
    """Cold cache + upstream down -> 200 with rate=null so the page
    can still render with the source amount."""
    def _down(url: str) -> _FakeResponse:
        return _FakeResponse(503, {"error": "down"})
    fake = _FakeAsyncClient(_down)
    _install_fake_client(monkeypatch, fake)
    r = await client.get("/api/fx/convert?from=GBP&to=USD&amount=10")
    assert r.status_code == 200, r.text
    body = r.json()
    assert body["rate"] is None
    assert body["amount"] is None
    assert body["source_amount"] == 10.0
