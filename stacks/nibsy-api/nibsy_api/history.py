"""Per-user viewing history (the input to personalised recommendations).

Only logged-in users are recorded, one row per page they have read. See
``models.NibsyPageView`` for exactly what is kept.
"""

from __future__ import annotations

import logging
from datetime import datetime, timedelta

from sqlalchemy import delete, select
from sqlalchemy.ext.asyncio import AsyncSession

from .config import get_settings
from .models import NibsyPageView
from .recommender import View, get_index

logger = logging.getLogger(__name__)

# Most recent pages considered when building a profile.
MAX_HISTORY = 200


def _cutoff(now: datetime) -> datetime:
    return now - timedelta(days=get_settings().view_history_days)


async def record_view(
    session: AsyncSession, username: str, page: str, now: datetime
) -> bool:
    """Note that ``username`` viewed ``page``. False if it is not a catalogue page.

    A lesson counts as a view of its course, so history stays one row per
    thing-you-might-recommend rather than one per URL.
    """

    index = await get_index(session)
    item, _basis = index.resolve(page)
    if item is None:
        return False
    row = await session.get(NibsyPageView, (username, item.id))
    if row is None:
        session.add(
            NibsyPageView(
                username=username,
                content_id=item.id,
                view_count=1,
                last_viewed_at=now,
            )
        )
    else:
        row.view_count += 1
        row.last_viewed_at = now
    await session.commit()
    return True


async def load_history(
    session: AsyncSession, username: str, now: datetime
) -> list[View]:
    rows = (
        await session.scalars(
            select(NibsyPageView)
            .where(
                NibsyPageView.username == username,
                NibsyPageView.last_viewed_at >= _cutoff(now),
            )
            .order_by(NibsyPageView.last_viewed_at.desc())
            .limit(MAX_HISTORY)
        )
    ).all()
    return [View(r.content_id, r.last_viewed_at, r.view_count) for r in rows]


async def clear_history(session: AsyncSession, username: str) -> int:
    result = await session.execute(
        delete(NibsyPageView).where(NibsyPageView.username == username)
    )
    await session.commit()
    return result.rowcount or 0


async def purge_old_views(session: AsyncSession, now: datetime) -> int:
    """Delete history older than the retention window (run daily)."""

    result = await session.execute(
        delete(NibsyPageView).where(NibsyPageView.last_viewed_at < _cutoff(now))
    )
    await session.commit()
    if result.rowcount:
        logger.info("history: purged %s expired page views", result.rowcount)
    return result.rowcount or 0
