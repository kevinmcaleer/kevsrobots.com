"""Background supplier-link health check (issue #122 Phase 2).

A daily APScheduler job walks every ``part_suppliers`` row, fires a HEAD
request at its URL with a short timeout, and updates the health-status
columns. After 3 consecutive non-2xx checks a supplier is marked
``is_broken=True`` so the frontend can show a warning pill. The first
2xx clears both the broken flag and the consecutive-failure counter.

Concurrency is capped via an asyncio semaphore so a slow supplier
doesn't block the whole run. A single misbehaving URL costs at most one
request's timeout (currently 5s).

The same helper is exposed via ``POST /api/admin/parts/{slug}/recheck-
suppliers`` for ad-hoc verification. That admin path bypasses the
scheduler and runs synchronously for a single part — useful for the
"force re-check" button on the wiki page.

Test isolation
--------------
``schedule_health_check`` is only called in non-test environments. The
test fixture's lifespan runs in ``ENVIRONMENT=test`` (or with the
``DISABLE_BACKGROUND_JOBS=1`` env var) and never starts a scheduler.
"""

from __future__ import annotations

import asyncio
import logging
import os
from datetime import datetime, timezone
from typing import Iterable, Optional

import httpx
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession, async_sessionmaker

from .db import get_sessionmaker
from .models import Part, PartSupplier
from .parts_lifecycle import compute_part_status

logger = logging.getLogger(__name__)

# Tunables. Kept module-level so tests can monkeypatch easily.
HEALTH_CHECK_TIMEOUT_SECONDS = 5.0
HEALTH_CHECK_CONCURRENCY = 10
BROKEN_THRESHOLD = 3  # consecutive failures before is_broken flips true


def _utcnow_naive() -> datetime:
    return datetime.now(timezone.utc).replace(tzinfo=None)


def _is_2xx(status_code: Optional[int]) -> bool:
    return status_code is not None and 200 <= status_code < 300


async def _probe_url(client: httpx.AsyncClient, url: str) -> Optional[int]:
    """HEAD-check ``url`` and return the status code, or None on error.

    Some sites disallow HEAD; we fall back to a streamed GET that is
    cancelled as soon as headers arrive. Either path returns the status
    code observed, or None if the request raised (timeout / DNS /
    connection refused).
    """
    try:
        resp = await client.head(url, follow_redirects=True)
        # Some CDNs return 405 for HEAD on otherwise-fine pages — retry
        # with a streaming GET and bail as soon as we see headers.
        if resp.status_code in (405, 501):
            async with client.stream("GET", url, follow_redirects=True) as r:
                return r.status_code
        return resp.status_code
    except Exception as exc:  # noqa: BLE001 — best-effort probe
        logger.debug("supplier probe failed for %s: %s", url, exc)
        return None


async def _apply_result(
    supplier: PartSupplier,
    status_code: Optional[int],
    *,
    now: datetime,
) -> None:
    """Update the supplier row in-place based on the probe result.

    Caller owns the session / commit.
    """
    supplier.last_checked_at = now
    supplier.last_status_code = status_code
    if _is_2xx(status_code):
        # Success — reset the failure counter and clear the broken flag.
        supplier.consecutive_failures = 0
        supplier.is_broken = False
        supplier.last_status = "ok"
    else:
        supplier.consecutive_failures = int(supplier.consecutive_failures or 0) + 1
        if supplier.consecutive_failures >= BROKEN_THRESHOLD:
            supplier.is_broken = True
            supplier.last_status = "broken"
        else:
            supplier.last_status = "unknown"


async def _check_supplier(
    client: httpx.AsyncClient,
    semaphore: asyncio.Semaphore,
    supplier: PartSupplier,
    now: datetime,
) -> None:
    async with semaphore:
        status_code = await _probe_url(client, supplier.url)
        await _apply_result(supplier, status_code, now=now)


async def run_health_check_for_suppliers(
    session: AsyncSession,
    suppliers: Iterable[PartSupplier],
    *,
    client: Optional[httpx.AsyncClient] = None,
) -> int:
    """Run the health check for ``suppliers``, persisting results.

    Returns the number of suppliers processed. ``client`` is injected so
    tests can pass a transport that simulates 200 / 500 / timeout.
    """
    suppliers_list = list(suppliers)
    if not suppliers_list:
        return 0

    semaphore = asyncio.Semaphore(HEALTH_CHECK_CONCURRENCY)
    now = _utcnow_naive()
    if client is None:
        async with httpx.AsyncClient(timeout=HEALTH_CHECK_TIMEOUT_SECONDS) as http:
            await asyncio.gather(
                *(_check_supplier(http, semaphore, s, now) for s in suppliers_list),
                return_exceptions=False,
            )
    else:
        await asyncio.gather(
            *(_check_supplier(client, semaphore, s, now) for s in suppliers_list),
            return_exceptions=False,
        )

    # Refresh lifecycle status for any part whose suppliers we just
    # touched. A newly-healthy supplier might be the missing signal for
    # promotion; a freshly-broken set might trigger demotion (Phase 3).
    touched_part_ids = {s.part_id for s in suppliers_list}
    if touched_part_ids:
        parts = (
            await session.scalars(
                select(Part).where(Part.id.in_(touched_part_ids))
            )
        ).all()
        for part in parts:
            await compute_part_status(session, part)

    await session.commit()
    return len(suppliers_list)


async def run_full_health_check(
    sessionmaker: Optional[async_sessionmaker[AsyncSession]] = None,
) -> int:
    """Walk every ``part_suppliers`` row and update health status.

    This is the entry point for the APScheduler daily job.
    """
    sm = sessionmaker or get_sessionmaker()
    async with sm() as session:
        suppliers = list(
            (await session.scalars(select(PartSupplier))).all()
        )
        try:
            count = await run_health_check_for_suppliers(session, suppliers)
            logger.info("supplier health check processed %d row(s)", count)
            return count
        except Exception:  # noqa: BLE001 — scheduler must keep running
            logger.exception("supplier health check failed")
            await session.rollback()
            return 0


async def recheck_part_suppliers(
    session: AsyncSession, part_id: int
) -> int:
    """Synchronously re-check every supplier on a single part.

    Used by the admin ``recheck-suppliers`` endpoint. Returns the number
    of suppliers checked.
    """
    suppliers = list(
        (
            await session.scalars(
                select(PartSupplier).where(PartSupplier.part_id == part_id)
            )
        ).all()
    )
    return await run_health_check_for_suppliers(session, suppliers)


def background_jobs_disabled() -> bool:
    """Return True when the daily scheduler should not start.

    Triggered by either ``DISABLE_BACKGROUND_JOBS=1`` (what tests set) or
    a sniff of ``PYTEST_CURRENT_TEST`` so a stray invocation under pytest
    never spins one up.
    """
    if os.environ.get("DISABLE_BACKGROUND_JOBS", "").strip() in {"1", "true", "TRUE"}:
        return True
    if os.environ.get("PYTEST_CURRENT_TEST"):
        return True
    return False
