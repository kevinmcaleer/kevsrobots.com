"""Public FX-conversion endpoint — issue #150.

The frontend calls this to render "converted from £12.50" tooltips on
part / project pages. Read-only, no auth, no DB access. Backed by the
``projects_api.fx`` cache module.

Returns 422 for unsupported / malformed currency codes. Returns the
converted amount otherwise. If the upstream rates source is cold and
we have no cached data at all, returns ``rate: null`` so the caller
can degrade to displaying the source amount only.
"""

from __future__ import annotations

from datetime import datetime
from typing import Optional

from fastapi import APIRouter, HTTPException, Query, status
from pydantic import BaseModel, Field

from ..fx import SUPPORTED_CURRENCIES, convert

router = APIRouter(tags=["fx"])


class FxConvertResponse(BaseModel):
    """Conversion result. ``rate`` / ``fetched_at`` may be null if the
    rates source is unavailable and the cache is cold — callers should
    fall back to showing the source amount only.
    """

    amount: Optional[float] = None
    from_currency: str = Field(..., min_length=3, max_length=3)
    to_currency: str = Field(..., min_length=3, max_length=3)
    rate: Optional[float] = None
    fetched_at: Optional[datetime] = None
    source_amount: float


@router.get("/api/fx/convert", response_model=FxConvertResponse)
async def convert_endpoint(
    from_: str = Query(..., alias="from", min_length=3, max_length=3),
    to: str = Query(..., min_length=3, max_length=3),
    amount: float = Query(...),
) -> FxConvertResponse:
    """Convert ``amount`` from ``from`` to ``to``.

    422 if the currency codes aren't 3-letter alpha or aren't in the
    supported set. 200 with ``rate=None`` if the rates upstream is
    cold and unreachable — the caller can decide whether to retry or
    display the source amount alone.
    """
    src = (from_ or "").strip().upper()
    dst = (to or "").strip().upper()
    if (
        len(src) != 3
        or len(dst) != 3
        or not src.isalpha()
        or not dst.isalpha()
        or src not in SUPPORTED_CURRENCIES
        or dst not in SUPPORTED_CURRENCIES
    ):
        raise HTTPException(
            status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail="Unsupported currency code. "
                   f"Supported: {sorted(SUPPORTED_CURRENCIES)}",
        )
    if amount != amount or amount in (float("inf"), float("-inf")):
        # NaN / inf — reject up front.
        raise HTTPException(
            status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail="amount must be a finite number",
        )

    result = await convert(amount, src, dst)
    if result is None:
        # Upstream cold and unreachable. Surface a 200 with null rate so
        # the page still renders — the frontend just displays the
        # source amount untransformed.
        return FxConvertResponse(
            amount=None,
            from_currency=src,
            to_currency=dst,
            rate=None,
            fetched_at=None,
            source_amount=float(amount),
        )

    return FxConvertResponse(
        amount=result.amount,
        from_currency=result.source_currency,
        to_currency=result.target_currency,
        rate=result.rate,
        fetched_at=result.fetched_at,
        source_amount=result.source_amount,
    )
