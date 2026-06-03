"""``/api/status`` and ``/api/status/{service}`` tests.

We seed the DB directly with synthetic ``health_checks`` rows and
assert the per-service projection + overall aggregation rules:

  * all green   -> overall green
  * any red     -> overall red
  * mixed       -> overall amber
  * service with no rows -> 'unknown' (which is non-green -> amber).
"""

from __future__ import annotations

from datetime import datetime, timezone

import pytest

from status_service.db import insert_checks_bulk


def _ts() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S")


@pytest.mark.asyncio
async def test_overall_all_green(db_path, client) -> None:
    """Three green checks -> overall green."""

    ts = _ts()
    await insert_checks_bulk(
        db_path,
        [
            ("search", ts, "green", 50, 200, None),
            ("chatter", ts, "green", 60, 200, None),
            ("nibsy", ts, "green", 70, 200, None),
        ],
    )

    r = await client.get("/api/status")
    assert r.status_code == 200
    body = r.json()
    assert body["overall"] == "green"
    assert set(body["services"].keys()) == {"search", "chatter", "nibsy"}
    assert body["services"]["search"]["status"] == "green"
    assert body["services"]["search"]["latency_ms"] == 50


@pytest.mark.asyncio
async def test_overall_any_red(db_path, client) -> None:
    """One red anywhere -> overall red."""

    ts = _ts()
    await insert_checks_bulk(
        db_path,
        [
            ("search", ts, "green", 50, 200, None),
            ("chatter", ts, "red", None, None, "timeout"),
            ("nibsy", ts, "amber", 3000, 200, None),
        ],
    )

    r = await client.get("/api/status")
    body = r.json()
    assert body["overall"] == "red"
    assert body["services"]["chatter"]["status"] == "red"
    assert body["services"]["chatter"]["message"] == "timeout"


@pytest.mark.asyncio
async def test_overall_mixed_amber(db_path, client) -> None:
    """Green + amber (no red) -> overall amber."""

    ts = _ts()
    await insert_checks_bulk(
        db_path,
        [
            ("search", ts, "green", 50, 200, None),
            ("chatter", ts, "amber", 3000, 200, None),
            ("nibsy", ts, "green", 70, 200, None),
        ],
    )

    r = await client.get("/api/status")
    body = r.json()
    assert body["overall"] == "amber"


@pytest.mark.asyncio
async def test_unknown_service_counts_as_amber(db_path, client) -> None:
    """A configured service with NO rows is 'unknown' -> overall amber."""

    ts = _ts()
    await insert_checks_bulk(
        db_path,
        [
            ("search", ts, "green", 50, 200, None),
            ("chatter", ts, "green", 60, 200, None),
            # 'nibsy' deliberately absent — never polled yet.
        ],
    )

    r = await client.get("/api/status")
    body = r.json()
    assert body["services"]["nibsy"]["status"] == "unknown"
    assert body["overall"] == "amber"  # not green (because unknown)


@pytest.mark.asyncio
async def test_latest_wins(db_path, client) -> None:
    """The most-recent row is what shows up in /api/status."""

    await insert_checks_bulk(
        db_path,
        [
            ("search", "2025-01-01T00:00:00", "red", None, None, "timeout"),
            ("search", "2025-06-01T00:00:00", "green", 30, 200, None),
            ("chatter", "2025-06-01T00:00:00", "green", 40, 200, None),
            ("nibsy", "2025-06-01T00:00:00", "green", 50, 200, None),
        ],
    )

    r = await client.get("/api/status")
    body = r.json()
    assert body["services"]["search"]["status"] == "green"
    # And the overall is green (no live red).
    assert body["overall"] == "green"


@pytest.mark.asyncio
async def test_single_service_endpoint(db_path, client) -> None:
    ts = _ts()
    await insert_checks_bulk(
        db_path,
        [("search", ts, "green", 50, 200, None)],
    )

    r = await client.get("/api/status/search")
    assert r.status_code == 200
    assert r.json()["status"] == "green"


@pytest.mark.asyncio
async def test_single_service_unknown(client) -> None:
    r = await client.get("/api/status/does-not-exist")
    assert r.status_code == 404
