"""SQLite storage for health-check results.

Storage is intentionally tiny: a single ``health_checks`` table, one
row per (service, check). Everything else (overall status, incidents,
uptime) is derived on read. Per the issue, this keeps storage simple
and lets us bound disk usage with a daily vacuum.

All access is async via ``aiosqlite``. Connections are cheap and
short-lived; we open a fresh one per call rather than pooling — this
service is low-traffic and a single uvicorn worker.
"""

from __future__ import annotations

from contextlib import asynccontextmanager
from datetime import datetime, timezone
from typing import AsyncIterator, Iterable, Optional

import aiosqlite


SCHEMA_SQL = """
CREATE TABLE IF NOT EXISTS health_checks (
    id            INTEGER PRIMARY KEY AUTOINCREMENT,
    service       TEXT NOT NULL,
    checked_at    TEXT NOT NULL,        -- ISO 8601 UTC
    status        TEXT NOT NULL,        -- 'green' | 'amber' | 'red'
    latency_ms    INTEGER,              -- nullable on connect-error
    http_status   INTEGER,              -- nullable on timeout / error
    error         TEXT                  -- short message; never PII
);

CREATE INDEX IF NOT EXISTS ix_health_checks_service_time
    ON health_checks (service, checked_at);
CREATE INDEX IF NOT EXISTS ix_health_checks_time
    ON health_checks (checked_at);
"""


def utcnow_iso() -> str:
    """Return current UTC time as ISO 8601 (suitable for SQLite TEXT)."""

    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S")


@asynccontextmanager
async def connect(db_path: str) -> AsyncIterator[aiosqlite.Connection]:
    """Yield an aiosqlite connection with row factory and foreign keys on."""

    db = await aiosqlite.connect(db_path)
    try:
        db.row_factory = aiosqlite.Row
        await db.execute("PRAGMA journal_mode=WAL")
        await db.execute("PRAGMA foreign_keys=ON")
        yield db
    finally:
        await db.close()


async def init_schema(db_path: str) -> None:
    """Create tables/indexes if missing. Safe to call repeatedly."""

    async with connect(db_path) as db:
        await db.executescript(SCHEMA_SQL)
        await db.commit()


async def insert_check(
    db_path: str,
    *,
    service: str,
    status: str,
    latency_ms: Optional[int],
    http_status: Optional[int],
    error: Optional[str],
    checked_at: Optional[str] = None,
) -> int:
    """Insert one health-check row. Returns the new row id."""

    ts = checked_at or utcnow_iso()
    async with connect(db_path) as db:
        cursor = await db.execute(
            "INSERT INTO health_checks "
            "(service, checked_at, status, latency_ms, http_status, error) "
            "VALUES (?, ?, ?, ?, ?, ?)",
            (service, ts, status, latency_ms, http_status, error),
        )
        await db.commit()
        return int(cursor.lastrowid or 0)


async def insert_checks_bulk(
    db_path: str,
    rows: Iterable[tuple],
) -> int:
    """Bulk insert. Each row is
    ``(service, checked_at, status, latency_ms, http_status, error)``.

    Used by tests to seed history quickly.
    """

    rows_list = list(rows)
    if not rows_list:
        return 0
    async with connect(db_path) as db:
        await db.executemany(
            "INSERT INTO health_checks "
            "(service, checked_at, status, latency_ms, http_status, error) "
            "VALUES (?, ?, ?, ?, ?, ?)",
            rows_list,
        )
        await db.commit()
    return len(rows_list)


async def latest_per_service(
    db_path: str,
    services: Iterable[str],
) -> dict[str, dict]:
    """Return the most-recent ``health_checks`` row for each service.

    Missing services (never polled yet) are omitted from the result.
    Caller can fill in a ``status='unknown'`` placeholder.
    """

    services_list = list(services)
    if not services_list:
        return {}

    placeholders = ",".join("?" for _ in services_list)
    sql = (
        "SELECT service, checked_at, status, latency_ms, http_status, error "
        "FROM health_checks "
        "WHERE id IN ("
        "  SELECT MAX(id) FROM health_checks "
        f"  WHERE service IN ({placeholders}) "
        "  GROUP BY service"
        ")"
    )
    async with connect(db_path) as db:
        async with db.execute(sql, services_list) as cur:
            rows = await cur.fetchall()

    return {
        r["service"]: {
            "service": r["service"],
            "checked_at": r["checked_at"],
            "status": r["status"],
            "latency_ms": r["latency_ms"],
            "http_status": r["http_status"],
            "error": r["error"],
        }
        for r in rows
    }


async def history_since(
    db_path: str,
    *,
    since_iso: str,
    service: Optional[str] = None,
) -> list[dict]:
    """Return all checks since ``since_iso``, optionally for one service.

    Ordered by ``(service, checked_at)`` so callers can group contiguous
    runs in a single pass.
    """

    if service:
        sql = (
            "SELECT service, checked_at, status, latency_ms, http_status, error "
            "FROM health_checks "
            "WHERE checked_at >= ? AND service = ? "
            "ORDER BY service, checked_at"
        )
        params: tuple = (since_iso, service)
    else:
        sql = (
            "SELECT service, checked_at, status, latency_ms, http_status, error "
            "FROM health_checks "
            "WHERE checked_at >= ? "
            "ORDER BY service, checked_at"
        )
        params = (since_iso,)

    async with connect(db_path) as db:
        async with db.execute(sql, params) as cur:
            rows = await cur.fetchall()

    return [dict(r) for r in rows]


async def cell_status_strings(
    db_path: str,
    *,
    services: list[str],
    cell_seconds: int,
    cells: int,
) -> dict[str, str]:
    """Return one compact status string per service for timeline rendering.

    Each char represents one ``cell_seconds`` window:
      ``g`` = green, ``a`` = amber, ``r`` = red, ``_`` = no check landed.

    The string is left-to-right oldest→newest, length ``cells`` — i.e.
    index 0 is ``cells * cell_seconds`` seconds ago, index ``cells-1`` is
    the current cell. The frontend renders one ``<span>`` per char.

    When multiple checks land in the same cell, the WORST status wins
    (red > amber > green) — a single bad probe in a 15-minute window is
    enough to colour that cell.
    """

    if not services or cells <= 0:
        return {s: "_" * cells for s in services}

    # SQLite math: ``cell_age`` = how many cell-windows ago the check
    # happened, relative to "now". 0 = current cell, ``cells-1`` = oldest.
    # ``worst_rank`` exploits MIN() to pick the worst status per bucket
    # (red < amber < green < other).
    sql = (
        "SELECT service, "
        "  CAST("
        "    (CAST(strftime('%s','now') AS INTEGER) "
        "     - CAST(strftime('%s', checked_at) AS INTEGER)) / ? "
        "  AS INTEGER) AS cell_age, "
        "  MIN(CASE status "
        "        WHEN 'red'   THEN 1 "
        "        WHEN 'amber' THEN 2 "
        "        WHEN 'green' THEN 3 "
        "        ELSE 4 END) AS worst_rank "
        "FROM health_checks "
        "WHERE checked_at >= datetime('now', ?) "
        "  AND service IN (" + ",".join("?" * len(services)) + ") "
        "GROUP BY service, cell_age"
    )
    params: tuple = (
        cell_seconds,
        f"-{cells * cell_seconds} seconds",
        *services,
    )

    cells_by_service: dict[str, list[str]] = {s: ["_"] * cells for s in services}
    rank_to_char = {1: "r", 2: "a", 3: "g"}

    async with connect(db_path) as db:
        async with db.execute(sql, params) as cur:
            async for row in cur:
                service = row["service"]
                cell_age = int(row["cell_age"])
                rank = int(row["worst_rank"])
                if service not in cells_by_service:
                    continue
                if cell_age < 0 or cell_age >= cells:
                    continue
                # cell_age 0 → rightmost (now); cell_age = cells-1 → leftmost.
                idx = cells - 1 - cell_age
                cells_by_service[service][idx] = rank_to_char.get(rank, "_")

    return {s: "".join(chars) for s, chars in cells_by_service.items()}


async def daily_uptime(
    db_path: str,
    *,
    services: list[str],
    days: int,
) -> dict[str, list[dict]]:
    """Per-service, per-day uptime percentage for the status page.

    Returns ``{service: [{"date": "YYYY-MM-DD", "uptime": 0..100}, ...]}``
    with entries oldest→newest. Days on which a service was never polled
    are simply absent (the page pads missing days with grey "no data"
    bars), so a service with only 12 days of history yields 12 entries.

    Each check contributes a weighted score to its day —
    ``green = 1.0``, ``amber = 0.5``, ``red = 0.0`` — and the day's
    uptime is ``100 * sum(score) / count``. Weighting amber at a half
    means a fully-degraded day lands at 50% (the page's amber band:
    50–99.5% amber, ≥99.5% green, <50% red) rather than being
    misreported as a total outage.
    """

    if not services or days <= 0:
        return {s: [] for s in services}

    placeholders = ",".join("?" for _ in services)
    sql = (
        "SELECT service, date(checked_at) AS day, "
        "  COUNT(*) AS total, "
        "  SUM(CASE status "
        "        WHEN 'green' THEN 1.0 "
        "        WHEN 'amber' THEN 0.5 "
        "        ELSE 0.0 END) AS score "
        "FROM health_checks "
        "WHERE checked_at >= datetime('now', ?) "
        f"  AND service IN ({placeholders}) "
        "GROUP BY service, day "
        "ORDER BY service, day"
    )
    params: tuple = (f"-{days} days", *services)

    out: dict[str, list[dict]] = {s: [] for s in services}
    async with connect(db_path) as db:
        async with db.execute(sql, params) as cur:
            async for row in cur:
                svc = row["service"]
                if svc not in out:
                    continue
                total = int(row["total"] or 0)
                if total <= 0:
                    continue
                score = float(row["score"] or 0.0)
                out[svc].append(
                    {"date": row["day"], "uptime": round(100.0 * score / total, 2)}
                )
    return out


async def vacuum_old_rows(db_path: str, *, retention_days: int) -> int:
    """Delete rows older than ``retention_days`` then run VACUUM.

    Returns the number of rows deleted. Idempotent; safe to run more
    than once per day if something retries it.
    """

    async with connect(db_path) as db:
        cursor = await db.execute(
            "DELETE FROM health_checks WHERE checked_at < ?",
            (_cutoff_iso(retention_days),),
        )
        deleted = cursor.rowcount or 0
        await db.commit()
        # VACUUM cannot run inside a transaction; aiosqlite has already
        # committed above. Run it as a standalone statement.
        await db.execute("VACUUM")
        await db.commit()
    return int(deleted)


def _cutoff_iso(retention_days: int) -> str:
    from datetime import timedelta

    cutoff = datetime.now(timezone.utc) - timedelta(days=retention_days)
    return cutoff.strftime("%Y-%m-%dT%H:%M:%S")
