"""Trending score computation (#72).

Combines multiple signals into a composite trending score per content item:
- Nibsy click-throughs (high signal, 7-day window)
- Page views from the page_count service
- YouTube view counts from popular_videos metadata
- Recency boost for newer content

Scores are precomputed hourly and cached in nibsy_trending.
"""

from __future__ import annotations

import logging
import time
from datetime import datetime, timedelta, timezone
from typing import Optional

import httpx
from sqlalchemy import delete, func, select
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


async def _fetch_page_views(
    page_count_url: str, url: str
) -> int:
    try:
        async with httpx.AsyncClient(timeout=5) as client:
            resp = await client.get(
                page_count_url,
                params={"url": f"https://www.kevsrobots.com{url}"},
            )
            if resp.status_code == 200:
                data = resp.json()
                return data.get("visits", 0)
    except Exception:
        pass
    return 0


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

    await session.execute(delete(NibsyTrending))

    computed = 0
    for content in contents:
        nibsy_clicks = click_counts.get(content.url, 0)

        meta = content.content_metadata or {}
        youtube_views = 0
        if content.content_type == "video":
            youtube_views = meta.get("views", 0) or 0

        page_views = await _fetch_page_views(page_count_url, content.url)

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
