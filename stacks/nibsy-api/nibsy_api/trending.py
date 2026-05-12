"""Trending score computation (#72).

Combines multiple signals into a composite trending score per content item:
- Nibsy click-throughs (high signal, 7-day window)
- Page views from the pagecount database (direct query, no HTTP)
- YouTube view counts from popular_videos metadata
- Recency boost for newer content

Scores are precomputed hourly and cached in nibsy_trending.
"""

from __future__ import annotations

import logging
import time
from datetime import datetime, timedelta, timezone
from typing import Optional

from sqlalchemy import create_engine, text, delete, func, select
from sqlalchemy.ext.asyncio import AsyncSession

from .models import NibsyClick, NibsyContent, NibsyTrending

logger = logging.getLogger(__name__)

W_PAGE_VIEWS = 1.0
W_NIBSY_CLICKS = 5.0
W_YOUTUBE_VIEWS = 0.001
W_RECENCY = 50.0


def _recency_boost(date_published: Optional[datetime], now: datetime) -> float:
    if date_published is None:
        return 0.0
    days = max(0, (now - date_published).days)
    return min(1.0, 1.0 / (1.0 + days / 90.0))


def _fetch_all_page_views(pagecount_db_url: str) -> dict[str, int]:
    """Query the pagecount database directly for visit counts per URL."""

    if not pagecount_db_url:
        return {}
    try:
        engine = create_engine(pagecount_db_url)
        with engine.connect() as conn:
            rows = conn.execute(
                text("SELECT page_url, COUNT(*) as views FROM page_visits GROUP BY page_url")
            ).fetchall()
        engine.dispose()
        return {row[0]: row[1] for row in rows}
    except Exception as exc:
        logger.warning("trending: could not query pagecount DB: %s", exc)
        return {}


async def compute_trending(
    session: AsyncSession,
    page_count_url: str,
) -> dict:
    start = time.monotonic()
    now = datetime.now(timezone.utc).replace(tzinfo=None)
    cutoff_7d = now - timedelta(days=7)

    contents = (await session.scalars(select(NibsyContent))).all()
    if not contents:
        return {"computed": 0, "duration_ms": 0}

    click_counts: dict[str, int] = {}
    click_rows = (
        await session.execute(
            select(
                NibsyClick.content_url,
                func.count(NibsyClick.id).label("clicks"),
            )
            .where(NibsyClick.clicked_at >= cutoff_7d)
            .group_by(NibsyClick.content_url)
        )
    ).all()
    for row in click_rows:
        click_counts[row.content_url] = row.clicks

    # Query pagecount DB directly — single query for all URLs.
    # Reuse the same Postgres host but the pagecount database.
    pagecount_db_url = page_count_url
    page_view_counts = _fetch_all_page_views(pagecount_db_url)

    await session.execute(delete(NibsyTrending))

    computed = 0
    for content in contents:
        nibsy_clicks = click_counts.get(content.url, 0)

        meta = content.content_metadata or {}
        youtube_views = 0
        if content.content_type == "video":
            youtube_views = meta.get("views", 0) or 0

        full_url = f"https://www.kevsrobots.com{content.url}"
        page_views = page_view_counts.get(full_url, 0)

        recency = _recency_boost(content.date_published, now)

        score = (
            W_PAGE_VIEWS * page_views
            + W_NIBSY_CLICKS * nibsy_clicks
            + W_YOUTUBE_VIEWS * youtube_views
            + W_RECENCY * recency
        )

        if score > 0:
            session.add(NibsyTrending(
                content_id=content.id,
                trending_score=round(score, 4),
                nibsy_clicks=nibsy_clicks,
                page_views=page_views,
                youtube_views=youtube_views,
                computed_at=now,
            ))
            computed += 1

    await session.commit()
    duration_ms = int((time.monotonic() - start) * 1000)
    logger.info("trending: computed %s scores in %sms", computed, duration_ms)
    return {"computed": computed, "duration_ms": duration_ms}
