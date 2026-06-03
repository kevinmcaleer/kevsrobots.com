"""Poll service ``/health`` endpoints and classify results.

Classification rule (per issue #203):
  * ``green`` — 200 within ``amber_latency_seconds``
  * ``amber`` — 200 but slow (latency above ``amber_latency_seconds``
    but ≤ ``poll_timeout_seconds``) **or** any non-200 / non-5xx
    HTTP response (4xx == "service answered but unhappy")
  * ``red`` — timeout, connection error, or 5xx

Errors are stored as a short string (e.g. ``"timeout"``,
``"connect_error"``). We never persist response bodies or request
headers — the dashboard is public.
"""

from __future__ import annotations

import asyncio
import logging
import time
from typing import Iterable, Optional, Tuple

import httpx

from .config import Settings
from .db import insert_check, utcnow_iso

logger = logging.getLogger(__name__)

STATUS_GREEN = "green"
STATUS_AMBER = "amber"
STATUS_RED = "red"


def classify(
    *,
    http_status: Optional[int],
    elapsed_seconds: Optional[float],
    error: Optional[str],
    amber_latency_seconds: float,
) -> str:
    """Map raw probe result -> 'green' | 'amber' | 'red'.

    Pure function so tests can drive it directly without hitting httpx.
    """

    if error or http_status is None:
        return STATUS_RED
    if 500 <= http_status < 600:
        return STATUS_RED
    if http_status != 200:
        # Treat 3xx/4xx as "service is up but the /health contract is
        # off". Amber rather than red — it didn't fail, it disagreed.
        return STATUS_AMBER
    # 200 — latency tier.
    if elapsed_seconds is not None and elapsed_seconds > amber_latency_seconds:
        return STATUS_AMBER
    return STATUS_GREEN


async def probe_one(
    client: httpx.AsyncClient,
    url: str,
    *,
    timeout_seconds: float,
) -> Tuple[Optional[int], Optional[float], Optional[str]]:
    """Probe a single URL. Returns ``(http_status, elapsed_seconds, error)``.

    ``error`` is ``None`` on a clean HTTP exchange (including 4xx/5xx —
    that's the server replying, not a transport failure).
    """

    start = time.perf_counter()
    try:
        resp = await client.get(url, timeout=timeout_seconds)
        elapsed = time.perf_counter() - start
        return resp.status_code, elapsed, None
    except httpx.TimeoutException:
        return None, None, "timeout"
    except httpx.ConnectError:
        return None, None, "connect_error"
    except httpx.HTTPError as exc:  # noqa: BLE001 — any transport hiccup
        return None, None, f"http_error:{exc.__class__.__name__}"
    except Exception as exc:  # noqa: BLE001 — never let the scheduler die
        logger.exception("unexpected probe error for %s", url)
        return None, None, f"error:{exc.__class__.__name__}"


async def poll_once(
    settings: Settings,
    services: Iterable[Tuple[str, str]],
    *,
    client: Optional[httpx.AsyncClient] = None,
) -> list[dict]:
    """Poll every service in ``services`` and persist one row each.

    Returns the list of inserted rows (as dicts) for logging / tests.
    ``client`` is injected so tests can pass a fake transport.
    """

    own_client = False
    if client is None:
        client = httpx.AsyncClient(timeout=settings.poll_timeout_seconds)
        own_client = True

    services_list = list(services)
    try:
        results = await asyncio.gather(
            *(
                probe_one(client, url, timeout_seconds=settings.poll_timeout_seconds)
                for _, url in services_list
            )
        )
    finally:
        if own_client:
            await client.aclose()

    inserted: list[dict] = []
    ts = utcnow_iso()
    for (name, _url), (http_status, elapsed, error) in zip(services_list, results):
        status = classify(
            http_status=http_status,
            elapsed_seconds=elapsed,
            error=error,
            amber_latency_seconds=settings.amber_latency_seconds,
        )
        latency_ms = int(elapsed * 1000) if elapsed is not None else None
        await insert_check(
            settings.status_db_path,
            service=name,
            status=status,
            latency_ms=latency_ms,
            http_status=http_status,
            error=error,
            checked_at=ts,
        )
        inserted.append(
            {
                "service": name,
                "checked_at": ts,
                "status": status,
                "latency_ms": latency_ms,
                "http_status": http_status,
                "error": error,
            }
        )

    return inserted


def overall_status(per_service: dict[str, dict]) -> str:
    """Aggregate per-service statuses into a single overall state.

    Issue rule: all green → green; any red → red; else amber.
    Unknown services (never polled) are treated as amber so the
    dashboard never shows green when a configured service has no data.
    """

    if not per_service:
        return STATUS_AMBER

    has_red = False
    has_non_green = False
    for entry in per_service.values():
        s = entry.get("status")
        if s == STATUS_RED:
            has_red = True
        if s != STATUS_GREEN:
            has_non_green = True

    if has_red:
        return STATUS_RED
    if has_non_green:
        return STATUS_AMBER
    return STATUS_GREEN
