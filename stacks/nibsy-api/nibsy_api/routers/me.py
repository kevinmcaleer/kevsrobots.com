"""A logged-in user's own Nibsy data: see it, clear it."""

from __future__ import annotations

from datetime import datetime, timezone

from fastapi import APIRouter, Depends
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import get_current_user
from ..db import get_session
from ..history import clear_history
from ..models import NibsyContent, NibsyPageView

router = APIRouter(prefix="/api/me", tags=["me"])


@router.get("/history")
async def my_history(
    session: AsyncSession = Depends(get_session),
    user: str = Depends(get_current_user),
) -> dict:
    """Everything Nibsy has recorded about what this user has viewed."""

    rows = (
        await session.execute(
            select(
                NibsyContent.title,
                NibsyContent.url,
                NibsyPageView.view_count,
                NibsyPageView.last_viewed_at,
            )
            .join(NibsyContent, NibsyContent.id == NibsyPageView.content_id)
            .where(NibsyPageView.username == user)
            .order_by(NibsyPageView.last_viewed_at.desc())
        )
    ).all()
    return {
        "history": [
            {
                "title": r.title,
                "url": r.url,
                "view_count": r.view_count,
                "last_viewed_at": r.last_viewed_at.replace(tzinfo=timezone.utc).isoformat(),
            }
            for r in rows
        ]
    }


@router.delete("/history")
async def clear_my_history(
    session: AsyncSession = Depends(get_session),
    user: str = Depends(get_current_user),
) -> dict:
    """Forget everything this user has viewed."""

    return {"deleted": await clear_history(session, user)}
