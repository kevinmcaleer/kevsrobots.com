"""The daily vacuum job removes rows older than ``retention_days``."""

from __future__ import annotations

from datetime import datetime, timedelta, timezone

import pytest

from status_service.db import (
    history_since,
    init_schema,
    insert_checks_bulk,
    vacuum_old_rows,
)


def _ts(days_ago: int) -> str:
    dt = datetime.now(timezone.utc) - timedelta(days=days_ago)
    return dt.strftime("%Y-%m-%dT%H:%M:%S")


@pytest.mark.asyncio
async def test_vacuum_deletes_only_old_rows(tmp_path) -> None:
    db = str(tmp_path / "vac.db")
    await init_schema(db)

    await insert_checks_bulk(
        db,
        [
            ("a", _ts(35), "green", 50, 200, None),    # too old
            ("a", _ts(31), "amber", 60, 200, None),    # just-too-old
            ("a", _ts(29), "green", 70, 200, None),    # keep
            ("a", _ts(1), "green", 80, 200, None),     # keep
        ],
    )

    # Sanity: 4 rows present pre-vacuum.
    rows_pre = await history_since(db, since_iso="1970-01-01T00:00:00")
    assert len(rows_pre) == 4

    deleted = await vacuum_old_rows(db, retention_days=30)
    assert deleted == 2

    rows_post = await history_since(db, since_iso="1970-01-01T00:00:00")
    assert len(rows_post) == 2
    statuses = sorted(r["status"] for r in rows_post)
    assert statuses == ["green", "green"]


@pytest.mark.asyncio
async def test_vacuum_is_idempotent(tmp_path) -> None:
    """Running the vacuum twice in a row deletes 0 the second time."""

    db = str(tmp_path / "vac2.db")
    await init_schema(db)

    await insert_checks_bulk(
        db,
        [("a", _ts(40), "red", None, None, "timeout")],
    )

    assert await vacuum_old_rows(db, retention_days=30) == 1
    assert await vacuum_old_rows(db, retention_days=30) == 0
