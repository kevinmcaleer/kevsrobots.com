"""Analytics endpoints for click/impression data (#68)."""

from __future__ import annotations

from datetime import datetime, timedelta, timezone

from fastapi import APIRouter, Depends, Query
from sqlalchemy import func, select
from sqlalchemy.ext.asyncio import AsyncSession

from ..db import get_session
from ..models import NibsyClick, NibsyContent, NibsyImpression

router = APIRouter(prefix="/api/analytics", tags=["analytics"])

_PERIOD_MAP = {"24h": 1, "7d": 7, "30d": 30}


def _cutoff(period: str) -> datetime:
    days = _PERIOD_MAP.get(period, 7)
    return datetime.now(timezone.utc).replace(tzinfo=None) - timedelta(days=days)


@router.get("/top-clicked")
async def top_clicked(
    period: str = Query("7d", pattern="^(24h|7d|30d)$"),
    limit: int = Query(10, ge=1, le=50),
    session: AsyncSession = Depends(get_session),
) -> dict:
    cutoff = _cutoff(period)

    rows = (
        await session.execute(
            select(
                NibsyContent.id,
                NibsyContent.title,
                NibsyContent.url,
                func.count(NibsyClick.id).label("clicks"),
            )
            .join(NibsyClick, NibsyClick.content_url == NibsyContent.url)
            .where(NibsyClick.clicked_at >= cutoff)
            .group_by(NibsyContent.id, NibsyContent.title, NibsyContent.url)
            .order_by(func.count(NibsyClick.id).desc())
            .limit(limit)
        )
    ).all()

    items = []
    for row in rows:
        imp_count = await session.scalar(
            select(func.count())
            .select_from(NibsyImpression)
            .where(
                NibsyImpression.content_id == row.id,
                NibsyImpression.shown_at >= cutoff,
            )
        ) or 0
        clicks = row.clicks
        items.append({
            "content_id": row.id,
            "title": row.title,
            "url": row.url,
            "clicks": clicks,
            "impressions": imp_count,
            "ctr": round(clicks / imp_count, 3) if imp_count else 0,
        })

    return {"top_clicked": items}


@router.get("/nibsy-stats")
async def nibsy_stats(
    session: AsyncSession = Depends(get_session),
) -> dict:
    today_start = datetime.now(timezone.utc).replace(
        hour=0, minute=0, second=0, microsecond=0, tzinfo=None
    )

    clicks_today = await session.scalar(
        select(func.count())
        .select_from(NibsyClick)
        .where(NibsyClick.clicked_at >= today_start)
    ) or 0

    impressions_today = await session.scalar(
        select(func.count())
        .select_from(NibsyImpression)
        .where(NibsyImpression.shown_at >= today_start)
    ) or 0

    click_by_type_rows = (
        await session.execute(
            select(NibsyContent.content_type, func.count())
            .join(NibsyClick, NibsyClick.content_url == NibsyContent.url)
            .where(NibsyClick.clicked_at >= today_start)
            .group_by(NibsyContent.content_type)
        )
    ).all()

    top_source_rows = (
        await session.execute(
            select(NibsyClick.source_page, func.count().label("c"))
            .where(NibsyClick.clicked_at >= today_start)
            .group_by(NibsyClick.source_page)
            .order_by(func.count().desc())
            .limit(5)
        )
    ).all()

    return {
        "total_clicks_today": clicks_today,
        "total_impressions_today": impressions_today,
        "overall_ctr": round(clicks_today / impressions_today, 3) if impressions_today else 0,
        "top_source_pages": [{"page": r[0], "clicks": r[1]} for r in top_source_rows],
        "click_by_content_type": {r[0]: r[1] for r in click_by_type_rows},
    }
