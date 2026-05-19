"""Foreign-exchange rates with in-memory caching — issue #150.

Source: https://open.er-api.com/v6/latest/{base}

Chosen because:
  * No API key required (we deploy on a small Pi cluster, key
    management would be friction).
  * Daily-refresh rates are accurate enough for "show roughly what
    this part costs in my currency" UX. We're not pricing trades.
  * Open licence, no surprise quotas for low traffic.

Failure mode: if the upstream fetch fails (network, 5xx, JSON shape
change) we serve the **last-known** rates from the cache, even if past
their nominal TTL. ``get_rate`` and ``convert`` return ``None`` only
when we have *no* data at all — never throw, never crash a render.

This module deliberately knows nothing about the SQL schema. It is
called from a public ``/api/fx/convert`` endpoint and, via the
frontend, from any page that wants to display a converted price.
"""

from __future__ import annotations

import asyncio
import logging
from dataclasses import dataclass
from datetime import datetime, timedelta
from typing import Any, Optional

import httpx

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

#: ISO 4217 codes we accept on the API surface. Kept short on purpose —
#: the UI dropdown only offers these. Validation lives here so the
#: profile route and the fx route share the same allow-list.
SUPPORTED_CURRENCIES: frozenset[str] = frozenset({
    "GBP", "USD", "EUR", "JPY", "AUD", "CAD",
})

#: How long a successful fetch stays "fresh". A second call within the
#: TTL is served from cache without hitting the network.
CACHE_TTL = timedelta(hours=24)

#: Hard cap on the upstream call. The endpoint must never block a page
#: render for more than this.
HTTP_TIMEOUT_SECONDS = 3.0

#: Default base currency to fetch. We pivot conversions through this
#: base so a single fetch covers every supported pair.
DEFAULT_BASE = "GBP"

#: Upstream endpoint. Public, no auth, daily-refresh.
RATES_URL_TEMPLATE = "https://open.er-api.com/v6/latest/{base}"


# ---------------------------------------------------------------------------
# Cache state
# ---------------------------------------------------------------------------


@dataclass
class _CacheEntry:
    """A single cached rates blob keyed by base currency."""

    rates: dict[str, float]
    fetched_at: datetime


_cache: dict[str, _CacheEntry] = {}
_cache_lock = asyncio.Lock()


def clear_fx_cache() -> None:
    """Drop every cached entry. Exposed for tests."""
    _cache.clear()


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


@dataclass
class ConvertedAmount:
    """The return shape from :func:`convert` — display-friendly."""

    amount: float
    source_amount: float
    source_currency: str
    target_currency: str
    rate: float
    fetched_at: datetime


def _normalise(ccy: Optional[str]) -> Optional[str]:
    if not ccy:
        return None
    upper = ccy.strip().upper()
    if len(upper) != 3 or not upper.isalpha():
        return None
    return upper


async def _fetch_rates(base: str) -> Optional[dict[str, float]]:
    """Fetch the latest rates for ``base`` from the upstream.

    Returns ``None`` on any failure (network, non-200, JSON shape).
    The caller is responsible for falling back to a stale cache value.
    """
    url = RATES_URL_TEMPLATE.format(base=base)
    try:
        async with httpx.AsyncClient(timeout=HTTP_TIMEOUT_SECONDS) as http:
            resp = await http.get(url)
        if resp.status_code != 200:
            logger.warning(
                "FX upstream returned %s for base=%s", resp.status_code, base
            )
            return None
        payload: Any = resp.json()
    except Exception as exc:  # noqa: BLE001 — any failure is "down".
        logger.warning("FX upstream call failed for base=%s: %s", base, exc)
        return None

    rates = payload.get("rates") if isinstance(payload, dict) else None
    if not isinstance(rates, dict):
        logger.warning("FX upstream returned unexpected shape for base=%s", base)
        return None

    # Coerce to floats and drop entries that aren't numeric. ``base``
    # itself is always 1.0; some providers omit it.
    cleaned: dict[str, float] = {}
    for code, value in rates.items():
        if not isinstance(code, str):
            continue
        try:
            cleaned[code.upper()] = float(value)
        except (TypeError, ValueError):
            continue
    cleaned.setdefault(base, 1.0)
    return cleaned


async def _get_cached(base: str) -> Optional[_CacheEntry]:
    """Return the cached entry for ``base``, refreshing if stale.

    Refresh failures fall back to the (possibly stale) cached value.
    Concurrent callers share a single in-flight refresh via the lock.
    """
    now = datetime.utcnow()
    entry = _cache.get(base)
    if entry is not None and (now - entry.fetched_at) < CACHE_TTL:
        return entry

    async with _cache_lock:
        # Re-check under lock — another coroutine may have just refreshed.
        entry = _cache.get(base)
        if entry is not None and (now - entry.fetched_at) < CACHE_TTL:
            return entry

        fresh = await _fetch_rates(base)
        if fresh is not None:
            entry = _CacheEntry(rates=fresh, fetched_at=datetime.utcnow())
            _cache[base] = entry
            return entry

        # Upstream is down. Serve whatever we have, even if stale.
        return _cache.get(base)


async def get_rate(from_ccy: str, to_ccy: str) -> Optional[float]:
    """Return ``rate`` such that ``amount_to = amount_from * rate``.

    Returns ``None`` if either currency is unknown or we have no data
    at all for the base. Identity (`from == to`) is a fast 1.0.
    """
    src = _normalise(from_ccy)
    dst = _normalise(to_ccy)
    if src is None or dst is None:
        return None
    if src not in SUPPORTED_CURRENCIES or dst not in SUPPORTED_CURRENCIES:
        return None
    if src == dst:
        return 1.0

    entry = await _get_cached(DEFAULT_BASE)
    if entry is None:
        return None

    rates = entry.rates
    src_per_base = rates.get(src)
    dst_per_base = rates.get(dst)
    if not src_per_base or not dst_per_base:
        return None
    try:
        return dst_per_base / src_per_base
    except ZeroDivisionError:
        return None


async def convert(
    amount: float, from_ccy: str, to_ccy: str
) -> Optional[ConvertedAmount]:
    """Convert ``amount`` from ``from_ccy`` to ``to_ccy``.

    Returns ``None`` if the rate is unavailable (unknown currency or
    cold cache + upstream down). Otherwise returns a display-friendly
    blob including the rate and the timestamp the rate was fetched.
    """
    rate = await get_rate(from_ccy, to_ccy)
    if rate is None:
        return None
    src = _normalise(from_ccy)
    dst = _normalise(to_ccy)
    assert src is not None and dst is not None  # narrowed by get_rate

    # ``fetched_at`` comes from the cache entry for the base currency.
    entry = _cache.get(DEFAULT_BASE)
    fetched_at = entry.fetched_at if entry is not None else datetime.utcnow()

    try:
        converted = float(amount) * rate
    except (TypeError, ValueError):
        return None

    return ConvertedAmount(
        amount=round(converted, 2),
        source_amount=float(amount),
        source_currency=src,
        target_currency=dst,
        rate=rate,
        fetched_at=fetched_at,
    )
