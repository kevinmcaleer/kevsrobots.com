"""Per-day uptime aggregation + the /status.json feed the design page reads.

``daily_uptime`` rolls raw checks into one weighted uptime % per day
(green=1.0, amber=0.5, red=0.0), oldest→newest. ``/status.json`` wraps that
in the handoff schema: {updated, services:[{name, host, history}]}.
"""

from __future__ import annotations

from datetime import datetime, timedelta, timezone

import pytest

from status_service.db import daily_uptime, init_schema, insert_checks_bulk


def _ts(days_ago: int, minute: int = 0) -> str:
    dt = datetime.now(timezone.utc) - timedelta(days=days_ago)
    dt = dt.replace(minute=minute % 60, second=0, microsecond=0)
    return dt.strftime("%Y-%m-%dT%H:%M:%S")


@pytest.mark.asyncio
async def test_daily_uptime_weights_amber_half(tmp_path) -> None:
    """Two green + two amber in one day = (1+1+.5+.5)/4 = 75%."""
    db = str(tmp_path / "du.db")
    await init_schema(db)
    await insert_checks_bulk(
        db,
        [
            ("svc", _ts(1, 0), "green", 50, 200, None),
            ("svc", _ts(1, 1), "green", 50, 200, None),
            ("svc", _ts(1, 2), "amber", 2500, 200, None),
            ("svc", _ts(1, 3), "amber", 2600, 200, None),
        ],
    )
    out = await daily_uptime(db, services=["svc"], days=30)
    assert len(out["svc"]) == 1
    assert out["svc"][0]["uptime"] == 75.0


@pytest.mark.asyncio
async def test_daily_uptime_orders_oldest_first(tmp_path) -> None:
    """An older all-red day (0%) comes before a newer all-green day (100%)."""
    db = str(tmp_path / "order.db")
    await init_schema(db)
    await insert_checks_bulk(
        db,
        [
            ("svc", _ts(3, 0), "red", None, None, "connect_error"),
            ("svc", _ts(3, 1), "red", None, None, "connect_error"),
            ("svc", _ts(1, 0), "green", 40, 200, None),
        ],
    )
    hist = (await daily_uptime(db, services=["svc"], days=30))["svc"]
    assert len(hist) == 2
    assert hist[0]["uptime"] == 0.0
    assert hist[1]["uptime"] == 100.0
    assert hist[0]["date"] < hist[1]["date"]


@pytest.mark.asyncio
async def test_daily_uptime_absent_service_is_empty(tmp_path) -> None:
    """A configured service with no checks yields an empty history list."""
    db = str(tmp_path / "ghost.db")
    await init_schema(db)
    out = await daily_uptime(db, services=["ghost"], days=30)
    assert out == {"ghost": []}


@pytest.mark.asyncio
async def test_daily_uptime_excludes_rows_outside_window(tmp_path) -> None:
    """Checks older than the window are not counted."""
    db = str(tmp_path / "window.db")
    await init_schema(db)
    await insert_checks_bulk(
        db,
        [
            ("svc", _ts(40, 0), "green", 40, 200, None),  # 40d ago — outside 30d
            ("svc", _ts(2, 0), "green", 40, 200, None),   # inside
        ],
    )
    hist = (await daily_uptime(db, services=["svc"], days=30))["svc"]
    assert len(hist) == 1  # only the in-window day


@pytest.mark.asyncio
async def test_status_json_endpoint_shape(client) -> None:
    """The feed matches the design schema and is marked uncacheable."""
    resp = await client.get("/status.json")
    assert resp.status_code == 200
    body = resp.json()

    assert "updated" in body
    assert isinstance(body["services"], list) and body["services"]

    # Display names are humanised (title-cased) from the configured keys.
    names = [s["name"] for s in body["services"]]
    assert {"Search", "Chatter", "Nibsy"} <= set(names)

    for s in body["services"]:
        assert set(s) >= {"name", "host", "history"}
        assert isinstance(s["history"], list)
        # test fixture uses example.invalid URLs → a real (non-local) host
        assert s["host"] == "example.invalid"

    assert "no-store" in resp.headers.get("cache-control", "").lower()
