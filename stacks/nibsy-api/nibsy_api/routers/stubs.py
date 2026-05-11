"""501 stubs for endpoints deferred to follow-up PRs.

Each route returns HTTP 501 Not Implemented and points to the GitHub issue
that will land it. Returning 501 (rather than letting FastAPI 404) means
the widget integration (#71) gets a clear signal during early integration.
"""

from __future__ import annotations

from fastapi import APIRouter, HTTPException, status

router = APIRouter(prefix="/api", tags=["stubs"])


def _not_implemented(issue: str) -> None:
    raise HTTPException(
        status_code=status.HTTP_501_NOT_IMPLEMENTED,
        detail=f"Not implemented — see {issue}",
    )


@router.get("/recommendations")
async def get_recommendations() -> None:
    """Read precomputed recs out of `nibsy_recommendations` (see #67 / #73a)."""

    # The table exists (added in this PR for #73a) but the read path itself
    # lands in #67, and the generator that populates it lands in #73a/#73b.
    _not_implemented("#67 / #73a")


@router.get("/trending")
async def get_trending() -> None:
    """Trending-by-clicks endpoint — see #67 / #72."""

    _not_implemented("#67 / #72")


@router.get("/related/{content_id}")
async def get_related(content_id: int) -> None:
    """Related items for a given content row — see #67."""

    _not_implemented("#67")


@router.post("/track/click")
async def track_click() -> None:
    """Click-tracking ingress — see #68."""

    _not_implemented("#68")


@router.post("/track/impression")
async def track_impression() -> None:
    """Impression-tracking ingress — see #68."""

    _not_implemented("#68")
