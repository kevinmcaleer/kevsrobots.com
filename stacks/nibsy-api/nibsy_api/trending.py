"""Trending score computation (#72).

Combines multiple signals into a composite trending score per content item:
- Nibsy click-throughs (high signal, 7-day window)
- Page views from the last 30 days in the pagecount database (direct query, no HTTP)
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
from .recommender import normalise_url

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


# Page views older than this do not count towards "trending".
PAGE_VIEW_WINDOW_DAYS = 30


def _fetch_all_page_views(pagecount_db_url: str) -> dict[str, int]:
    """Recent visit counts from the pagecount database, keyed by page.

    The counter stores whatever ``window.location.href`` was (with or
    without ``www``, with query strings and fragments), so every stored URL
    is folded to the same key the catalogue uses before counting. Falls
    back to all-time counts if the table has no usable timestamp column.
    """

    if not pagecount_db_url:
        logger.warning(
            "trending: PAGE_COUNT_URL is not set — trending has no page views"
        )
        return {}
    cutoff = datetime.now(timezone.utc).replace(tzinfo=None) - timedelta(
        days=PAGE_VIEW_WINDOW_DAYS
    )
    queries = (
        (
            "SELECT page_url, COUNT(*) FROM page_visits "
            "WHERE timestamp >= :cutoff GROUP BY page_url",
            {"cutoff": cutoff},
        ),
        ("SELECT page_url, COUNT(*) FROM page_visits GROUP BY page_url", {}),
    )
    try:
        engine = create_engine(pagecount_db_url)
    except Exception as exc:  # noqa: BLE001
        logger.warning("trending: bad PAGE_COUNT_URL: %s", exc)
        return {}
    rows = None
    try:
        for sql, params in queries:
            try:
                with engine.connect() as conn:
                    rows = conn.execute(text(sql), params).fetchall()
                break
            except Exception as exc:  # noqa: BLE001
                logger.warning("trending: pagecount query failed (%s): %s", sql, exc)
    finally:
        engine.dispose()

    counts: dict[str, int] = {}
    for page_url, views in rows or []:
        if not page_url:
            continue
        key = normalise_url(page_url)
        counts[key] = counts.get(key, 0) + int(views)
    return counts


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

        page_views = page_view_counts.get(normalise_url(content.url), 0)

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
    with_views = sum(
        1 for c in contents if page_view_counts.get(normalise_url(c.url))
    )
    if not with_views:
        logger.warning(
            "trending: no catalogue page matched any page view — "
            "scores are recency and clicks only"
        )
    logger.info(
        "trending: computed %s scores in %sms (%s pages with views)",
        computed,
        duration_ms,
        with_views,
    )
    return {
        "computed": computed,
        "duration_ms": duration_ms,
        "pages_with_views": with_views,
    }
