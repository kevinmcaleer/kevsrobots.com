"""Per-cell timeline aggregation: oldest on the left, current on the right,
``g/a/r/_`` per cell. Worst status wins inside a cell."""

from __future__ import annotations

from datetime import datetime, timedelta, timezone

import pytest

from status_service.db import cell_status_strings, init_schema, insert_checks_bulk


def _ts(seconds_ago: int) -> str:
    dt = datetime.now(timezone.utc) - timedelta(seconds=seconds_ago)
    return dt.strftime("%Y-%m-%dT%H:%M:%S")


@pytest.mark.asyncio
async def test_cell_string_layout(tmp_path) -> None:
    """Recent cells fill from the right; older cells live to the left."""
    db = str(tmp_path / "tl.db")
    await init_schema(db)
    await insert_checks_bulk(
        db,
        [
            # Three cells of 60s each: indices 9, 9, 8, 7 from now.
            ("svc", _ts(10),  "green", 50, 200, None),  # current cell (rightmost)
            ("svc", _ts(50),  "green", 60, 200, None),  # current cell — same
            ("svc", _ts(70),  "green", 60, 200, None),  # one cell back
            ("svc", _ts(130), "green", 60, 200, None),  # two cells back
        ],
    )
    out = await cell_status_strings(
        db, services=["svc"], cell_seconds=60, cells=10
    )
    s = out["svc"]
    assert len(s) == 10
    assert s[-1] == "g", f"current cell should be green, got {s[-1]!r}"
    assert s[-2] == "g"
    assert s[-3] == "g"
    # Earlier cells unwritten.
    assert set(s[:-3]) == {"_"}


@pytest.mark.asyncio
async def test_worst_status_wins_in_a_cell(tmp_path) -> None:
    """Two probes in the same cell: red beats amber beats green."""
    db = str(tmp_path / "worst.db")
    await init_schema(db)
    await insert_checks_bulk(
        db,
        [
            ("svc", _ts(10), "green", 30, 200, None),
            ("svc", _ts(20), "amber", 2500, 200, None),
            ("svc", _ts(30), "red",   None, None, "connect_error"),
        ],
    )
    out = await cell_status_strings(
        db, services=["svc"], cell_seconds=120, cells=4
    )
    # All three checks land in the current cell — red wins.
    assert out["svc"][-1] == "r"


@pytest.mark.asyncio
async def test_unknown_service_string_all_empty(tmp_path) -> None:
    """A configured service with no checks comes back all-empty."""
    db = str(tmp_path / "empty.db")
    await init_schema(db)
    out = await cell_status_strings(
        db, services=["never_seen"], cell_seconds=900, cells=5
    )
    assert out == {"never_seen": "_____"}


@pytest.mark.asyncio
async def test_timeline_endpoint_returns_compact_strings(client) -> None:
    """End-to-end via the API client (uses the test-fixture's DB)."""
    resp = await client.get("/api/timeline?days=1")
    assert resp.status_code == 200
    body = resp.json()
    assert body["days"] == 1
    assert body["cells"] > 0
    assert "services" in body
    # Every configured service appears, each string is the requested length.
    for name, seq in body["services"].items():
        assert len(seq) == body["cells"]
        assert set(seq) <= {"g", "a", "r", "_"}
