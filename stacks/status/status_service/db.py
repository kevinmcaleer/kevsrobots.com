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
