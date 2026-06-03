"""Derive incidents and uptime from raw ``health_checks`` rows.

We deliberately do NOT denormalise an ``incidents`` table — incidents
are just contiguous runs of non-green status per service. Computing
them on read is O(n) over the 30-day window, which is tiny:
9 services * 96 checks/day * 30 days ≈ 26k rows worst case.

An "incident" starts when the status transitions from green (or no
prior row) to non-green, and ends on the first green row after.
``peak_status`` is ``red`` if any check in the run was red, else
``amber``. ``ended_at`` is ``None`` while the incident is ongoing.
"""

from __future__ import annotations

from datetime import datetime, timedelta, timezone
from typing import Iterable, Optional

from .db import history_since


def _since_iso(days: int) -> str:
    cutoff = datetime.now(timezone.utc) - timedelta(days=days)
    return cutoff.strftime("%Y-%m-%dT%H:%M:%S")


def derive_incidents(rows: Iterable[dict]) -> list[dict]:
    """Group sorted ``rows`` into incidents.

    ``rows`` must be ordered by ``(service, checked_at)``. This matches
    what ``db.history_since`` returns, so the caller can pipe them
    straight in.
    """

    incidents: list[dict] = []
    current: Optional[dict] = None
    current_service: Optional[str] = None

    for row in rows:
        service = row["service"]
        status = row["status"]
        ts = row["checked_at"]

        # Service boundary: close any open incident from the previous
        # service. Without this an "ongoing" red on service A would
        # incorrectly absorb the first green of service B.
        if current is not None and service != current_service:
            incidents.append(current)
            current = None
            current_service = None

        if status != "green":
            if current is None:
                current = {
                    "service": service,
                    "started_at": ts,
                    "ended_at": None,
                    "peak_status": status,
                }
                current_service = service
            else:
                # Escalate amber -> red if we see a red mid-incident.
                if status == "red":
                    current["peak_status"] = "red"
        else:
            # Green closes any open incident.
            if current is not None:
                current["ended_at"] = ts
                incidents.append(current)
                current = None
                current_service = None

    if current is not None:
        # Open incident still active at end of window.
        incidents.append(current)

    return incidents


def uptime_per_service(rows: Iterable[dict]) -> dict[str, dict]:
    """Compute green-share per service over ``rows``.

    Returns ``{service: {total, green, uptime_pct}}``. Services with no
    checks are omitted; caller can fill in 0/0/None if needed.
    """

    counts: dict[str, dict] = {}
    for row in rows:
        s = row["service"]
        bucket = counts.setdefault(s, {"total": 0, "green": 0})
        bucket["total"] += 1
        if row["status"] == "green":
            bucket["green"] += 1

    out: dict[str, dict] = {}
    for service, c in counts.items():
        total = c["total"]
        green = c["green"]
        pct = round((green / total) * 100.0, 2) if total else None
        out[service] = {"total": total, "green": green, "uptime_pct": pct}
    return out


async def incidents_for_window(
    db_path: str,
    *,
    days: int = 30,
    service: Optional[str] = None,
) -> list[dict]:
    """High-level helper: load history then derive incidents."""

    rows = await history_since(
        db_path, since_iso=_since_iso(days), service=service
    )
    return derive_incidents(rows)


async def uptime_for_window(
    db_path: str,
    *,
    days: int = 30,
    service: Optional[str] = None,
) -> dict[str, dict]:
    """High-level helper: load history then compute uptime."""

    rows = await history_since(
        db_path, since_iso=_since_iso(days), service=service
    )
    return uptime_per_service(rows)
