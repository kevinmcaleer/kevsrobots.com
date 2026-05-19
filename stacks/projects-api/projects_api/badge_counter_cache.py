"""TTL cache for badge-counter values fetched from external services.

Issue #142: ``_count_likes_received`` and ``_count_comments`` in
``badges.py`` need to call Chatter to get live numbers. Without a cache,
every project-create would fan out an HTTP call to Chatter per project
the user owns (likes are queried per-URL), which is wasteful given that
badge thresholds rarely change second-to-second.

Design
------

* In-process dict keyed by ``(username, kind)`` where ``kind`` is the
  badge ``threshold_type`` (``"likes_received"`` or ``"comments"``).
* Each entry stores ``(value, fetched_at_monotonic)``.
* On lookup:
    - MISS (no entry): call the fetch coroutine, store, return.
    - HIT within TTL: return cached value, no fetch.
    - HIT but stale: re-fetch; if the fetch raises or returns ``None``,
      return the stale value rather than 0 (graceful degradation per the
      issue's acceptance criteria — "if either external service is down,
      evaluation returns the previously-cached value rather than 500-ing").
* TTL comes from ``settings.badge_counter_cache_ttl_seconds`` (default
  300s) and is read on every call so a tweak via env var takes effect
  without a restart of this module.

Scope
-----

Projects-api runs as a single container per Pi today and the badge
catalog is small, so a process-local dict is sufficient. If we ever
scale to multiple workers per Pi or want cross-Pi consistency the cache
can be swapped for Redis with the same coroutine signature; no caller
needs to change.
"""

from __future__ import annotations

import logging
import time
from typing import Awaitable, Callable, Optional

from .config import get_settings

logger = logging.getLogger(__name__)


# (username, kind) -> (value, fetched_at_monotonic)
_CACHE: dict[tuple[str, str], tuple[int, float]] = {}


FetchFn = Callable[[], Awaitable[Optional[int]]]


async def get_cached_counter(
    username: str,
    kind: str,
    fetch: FetchFn,
) -> int:
    """Return the cached counter value, fetching via ``fetch`` if needed.

    ``fetch`` is a zero-arg coroutine that returns the freshly-computed
    integer, or ``None`` if the external lookup failed. The cache uses
    ``time.monotonic`` so timestamps are immune to wall-clock jumps.

    Stale-on-error semantics: if a refresh fails (``fetch`` raises or
    returns ``None``), the previous cached value is returned. If there
    is no previous value and the fetch fails, the cache stays empty and
    we return 0 — this is the "first call ever, service down" case;
    nothing better we can do.
    """
    settings = get_settings()
    ttl = max(0, int(settings.badge_counter_cache_ttl_seconds))
    now = time.monotonic()
    key = (username, kind)
    cached = _CACHE.get(key)

    if cached is not None and (now - cached[1]) < ttl:
        return cached[0]

    # Either MISS or stale — try to refresh.
    try:
        fresh = await fetch()
    except Exception as exc:  # noqa: BLE001 — defensive
        logger.warning(
            "Badge counter fetch raised for (%s, %s): %s — using stale value",
            username, kind, exc,
        )
        fresh = None

    if fresh is None:
        if cached is not None:
            # Graceful degradation: return last good value without
            # advancing the timestamp, so we'll try again next call.
            return cached[0]
        return 0

    _CACHE[key] = (int(fresh), now)
    return int(fresh)


def clear_badge_counter_cache() -> None:
    """Drop all cached counter values. Intended for tests."""
    _CACHE.clear()
