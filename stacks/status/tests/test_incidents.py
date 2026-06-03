"""Incident derivation tests.

A contiguous run of non-green checks for one service = one incident.
A green check closes the run. ``peak_status`` is the worst status
observed in the run.
"""

from __future__ import annotations

import pytest

from status_service.incidents import derive_incidents, uptime_per_service


def _row(service: str, ts: str, status: str) -> dict:
    return {
        "service": service,
        "checked_at": ts,
        "status": status,
        "latency_ms": None,
        "http_status": None,
        "error": None,
    }


def test_single_amber_run_is_one_incident() -> None:
    rows = [
        _row("search", "2025-06-01T00:00:00", "green"),
        _row("search", "2025-06-01T00:15:00", "amber"),
        _row("search", "2025-06-01T00:30:00", "amber"),
        _row("search", "2025-06-01T00:45:00", "green"),
    ]
    out = derive_incidents(rows)
    assert len(out) == 1
    inc = out[0]
    assert inc["service"] == "search"
    assert inc["started_at"] == "2025-06-01T00:15:00"
    assert inc["ended_at"] == "2025-06-01T00:45:00"
    assert inc["peak_status"] == "amber"


def test_gap_and_resume_is_two_incidents() -> None:
    rows = [
        _row("search", "2025-06-01T00:00:00", "amber"),
        _row("search", "2025-06-01T00:15:00", "green"),
        _row("search", "2025-06-01T00:30:00", "red"),
        _row("search", "2025-06-01T00:45:00", "green"),
    ]
    out = derive_incidents(rows)
    assert len(out) == 2
    assert out[0]["peak_status"] == "amber"
    assert out[0]["ended_at"] == "2025-06-01T00:15:00"
    assert out[1]["peak_status"] == "red"
    assert out[1]["ended_at"] == "2025-06-01T00:45:00"


def test_ongoing_incident_has_no_end() -> None:
    rows = [
        _row("search", "2025-06-01T00:00:00", "green"),
        _row("search", "2025-06-01T00:15:00", "red"),
        _row("search", "2025-06-01T00:30:00", "red"),
    ]
    out = derive_incidents(rows)
    assert len(out) == 1
    assert out[0]["ended_at"] is None
    assert out[0]["peak_status"] == "red"


def test_amber_escalates_to_red_within_one_incident() -> None:
    rows = [
        _row("search", "2025-06-01T00:00:00", "amber"),
        _row("search", "2025-06-01T00:15:00", "red"),
        _row("search", "2025-06-01T00:30:00", "amber"),
        _row("search", "2025-06-01T00:45:00", "green"),
    ]
    out = derive_incidents(rows)
    assert len(out) == 1
    assert out[0]["peak_status"] == "red"


def test_service_boundary_closes_open_incident() -> None:
    """An ongoing-red on service A must NOT absorb service B's first green."""

    rows = [
        _row("a", "2025-06-01T00:00:00", "red"),
        # Service boundary while incident A is open:
        _row("b", "2025-06-01T00:00:00", "green"),
    ]
    out = derive_incidents(rows)
    # Service A's incident is still ongoing (ended_at None), and B has none.
    a_incidents = [i for i in out if i["service"] == "a"]
    b_incidents = [i for i in out if i["service"] == "b"]
    assert len(a_incidents) == 1
    assert a_incidents[0]["ended_at"] is None
    assert b_incidents == []


def test_uptime_per_service_counts() -> None:
    rows = [
        _row("a", "t1", "green"),
        _row("a", "t2", "green"),
        _row("a", "t3", "red"),
        _row("a", "t4", "green"),
        _row("b", "t1", "amber"),
        _row("b", "t2", "amber"),
    ]
    out = uptime_per_service(rows)
    assert out["a"]["total"] == 4
    assert out["a"]["green"] == 3
    assert out["a"]["uptime_pct"] == 75.0
    assert out["b"]["total"] == 2
    assert out["b"]["green"] == 0
    assert out["b"]["uptime_pct"] == 0.0


@pytest.mark.asyncio
async def test_incidents_api_endpoint(db_path, client) -> None:
    """End-to-end: seed rows, hit /api/incidents."""

    from status_service.db import insert_checks_bulk

    await insert_checks_bulk(
        db_path,
        [
            ("search", "2025-06-01T00:00:00", "green", 50, 200, None),
            ("search", "2025-06-01T00:15:00", "red", None, None, "timeout"),
            ("search", "2025-06-01T00:30:00", "green", 60, 200, None),
        ],
    )
    # Need the rows to fall inside the requested window — use 36500 days
    # but the endpoint caps to retention_days (30). The rows are dated
    # 2025; if "today" is in 2026, they'll already be outside the window.
    # Insert one fresh row so the window catches it.
    from datetime import datetime, timezone

    fresh = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S")
    await insert_checks_bulk(
        db_path, [("chatter", fresh, "red", None, None, "timeout")]
    )

    r = await client.get("/api/incidents?days=30")
    assert r.status_code == 200
    body = r.json()
    assert body["days"] == 30
    # At least the fresh chatter row should produce an incident.
    chatter_inc = [i for i in body["incidents"] if i["service"] == "chatter"]
    assert len(chatter_inc) == 1
    assert chatter_inc[0]["peak_status"] == "red"
