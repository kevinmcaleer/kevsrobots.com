"""Polling + classification tests. No real network: we monkeypatch
``httpx.AsyncClient.get`` with a fake transport that synthesises the
response we want for each URL.
"""

from __future__ import annotations

import asyncio
import time

import httpx
import pytest

from status_service.config import get_settings
from status_service.db import history_since, init_schema
from status_service.polling import classify, poll_once


def test_classify_green() -> None:
    assert (
        classify(http_status=200, elapsed_seconds=0.05, error=None, amber_latency_seconds=2.0)
        == "green"
    )


def test_classify_amber_slow_200() -> None:
    assert (
        classify(http_status=200, elapsed_seconds=3.0, error=None, amber_latency_seconds=2.0)
        == "amber"
    )


def test_classify_amber_non_200() -> None:
    """3xx/4xx = service answered but the contract is off -> amber."""

    assert (
        classify(http_status=404, elapsed_seconds=0.1, error=None, amber_latency_seconds=2.0)
        == "amber"
    )
    assert (
        classify(http_status=301, elapsed_seconds=0.1, error=None, amber_latency_seconds=2.0)
        == "amber"
    )


def test_classify_red_5xx() -> None:
    assert (
        classify(http_status=503, elapsed_seconds=0.2, error=None, amber_latency_seconds=2.0)
        == "red"
    )


def test_classify_red_timeout() -> None:
    assert (
        classify(http_status=None, elapsed_seconds=None, error="timeout", amber_latency_seconds=2.0)
        == "red"
    )


@pytest.mark.asyncio
async def test_poll_once_writes_one_row_per_service(db_path, monkeypatch) -> None:
    """Mock httpx so a 200, a 503, and a timeout produce green/red/red."""

    # Per-URL responses keyed by URL.
    responses = {
        "http://svc/a": ("ok", 200),
        "http://svc/b": ("err", 503),
        # 'c' triggers a timeout via raise.
    }

    class FakeResponse:
        def __init__(self, body: str, status_code: int) -> None:
            self.text = body
            self.status_code = status_code

    async def fake_get(self, url, timeout=None):  # type: ignore[no-untyped-def]
        if url not in responses:
            raise httpx.TimeoutException("timed out", request=None)
        body, code = responses[url]
        return FakeResponse(body, code)

    monkeypatch.setattr(httpx.AsyncClient, "get", fake_get)

    settings = get_settings()
    await init_schema(db_path)

    services = [
        ("svc_a", "http://svc/a"),
        ("svc_b", "http://svc/b"),
        ("svc_c", "http://svc/c"),
    ]
    inserted = await poll_once(settings, services)

    by_name = {r["service"]: r for r in inserted}
    assert by_name["svc_a"]["status"] == "green"
    assert by_name["svc_a"]["http_status"] == 200
    assert by_name["svc_b"]["status"] == "red"
    assert by_name["svc_b"]["http_status"] == 503
    assert by_name["svc_c"]["status"] == "red"
    assert by_name["svc_c"]["error"] == "timeout"

    # And the rows landed in the DB.
    rows = await history_since(db_path, since_iso="1970-01-01T00:00:00")
    assert len(rows) == 3


@pytest.mark.asyncio
async def test_poll_once_classifies_slow_200_as_amber(db_path, monkeypatch) -> None:
    """A 200 that takes longer than amber_latency_seconds is amber."""

    class FakeResponse:
        status_code = 200
        text = "ok"

    async def slow_get(self, url, timeout=None):  # type: ignore[no-untyped-def]
        # Simulate a 2.5s response. The amber threshold is 2.0s by default.
        # We don't actually sleep — we cheat the clock by stubbing
        # ``time.perf_counter`` so the elapsed calc returns > 2s.
        return FakeResponse()

    # Replace perf_counter with a generator that returns t0 then t0+3
    seq = iter([100.0, 103.0])

    def fake_perf_counter() -> float:
        return next(seq)

    monkeypatch.setattr(httpx.AsyncClient, "get", slow_get)
    monkeypatch.setattr(time, "perf_counter", fake_perf_counter)

    settings = get_settings()
    await init_schema(db_path)
    inserted = await poll_once(settings, [("svc", "http://example/svc")])
    assert inserted[0]["status"] == "amber"
    assert inserted[0]["latency_ms"] == 3000


@pytest.mark.asyncio
async def test_poll_once_handles_connect_error(db_path, monkeypatch) -> None:
    async def boom(self, url, timeout=None):  # type: ignore[no-untyped-def]
        raise httpx.ConnectError("refused", request=None)

    monkeypatch.setattr(httpx.AsyncClient, "get", boom)

    settings = get_settings()
    await init_schema(db_path)
    inserted = await poll_once(settings, [("svc", "http://example/svc")])
    assert inserted[0]["status"] == "red"
    assert inserted[0]["error"] == "connect_error"
