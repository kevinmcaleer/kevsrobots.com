"""Public recommendations endpoint (issue #67, #74).

Thin read against the precomputed `nibsy_recommendations` table. The
generator (#74) keeps the table populated; this handler just looks up by
`source_url` and applies `limit` and `exclude` filters in-memory.

We deliberately return 200 with an empty list when the source page has
no recommendations yet, rather than 404. Nibsy is best-effort by design —
the widget should degrade gracefully, never throw.
"""

from __future__ import annotations

import logging
from typing import Optional

from fastapi import APIRouter, Depends, Query
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select

from ..db import get_session
from ..models import NibsyRecommendation
from ..schemas import RecommendationItem, RecommendationsResponse

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api", tags=["recommendations"])


# Match the TOP_N from the generator so we never claim to return more
# than what was precomputed.
_MAX_LIMIT = 6


def _parse_exclude(raw: Optional[str]) -> set[int]:
    """Parse `?exclude=1,2,3` into a set of ints, tolerantly."""

    if not raw:
        return set()
    out: set[int] = set()
    for part in raw.split(","):
        part = part.strip()
        if not part:
            continue
        try:
            out.add(int(part))
        except ValueError:
            # Silently ignore non-int tokens — the caller is the widget,
            # not a developer, and we don't want a bad URL to break the page.
            continue
    return out


@router.get("/recommendations", response_model=RecommendationsResponse)
async def get_recommendations(
    page: str = Query(..., description="URL path of the current page"),
    limit: int = Query(5, ge=1, le=_MAX_LIMIT),
    exclude: Optional[str] = Query(
        None, description="Comma-separated content IDs to exclude"
    ),
    session: AsyncSession = Depends(get_session),
) -> RecommendationsResponse:
    """Return the precomputed top-N recommendations for a source page."""

    row = await session.scalar(
        select(NibsyRecommendation).where(
            NibsyRecommendation.source_url == page
        )
    )
    if row is None:
        # No precomputed entry — graceful 200 with an empty list so the
        # widget can render its fallback without special-casing 404s.
        logger.debug("recommendations: no row for source_url=%s", page)
        return RecommendationsResponse(source_url=page)

    excluded_ids = _parse_exclude(exclude)
    items: list[RecommendationItem] = []
    for entry in row.recommendations or []:
        if entry.get("content_id") in excluded_ids:
            continue
        items.append(
            RecommendationItem(
                content_id=entry["content_id"],
                url=entry["url"],
                title=entry["title"],
                type=entry["type"],
                reason=entry.get("reason", ""),
                score=entry.get("score"),
            )
        )
        if len(items) >= limit:
            break

    return RecommendationsResponse(
        source_url=row.source_url,
        recommendations=items,
        generated_at=row.generated_at,
        generator_version=row.generator_version,
    )
