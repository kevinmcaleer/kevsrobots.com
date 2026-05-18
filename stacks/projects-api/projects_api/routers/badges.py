"""Badges & Achievements endpoints — issue #106.

Public endpoints:

* ``GET /api/badges`` — list the catalog of all defined badges.
* ``GET /api/users/{username}/badges`` — list badges a user has earned.

Authenticated endpoint:

* ``POST /api/badges/evaluate/{username}`` — trigger badge re-evaluation
  for a user. Allowed if the caller is the user themselves OR an admin.
  Returns ``newly_awarded`` so the caller can show a toast.

A separate admin-only retro-award route is also provided to backfill
badges for everyone after a deploy.
"""

from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import get_current_user
from ..badges import (
    evaluate_user,
    list_user_badges,
    retro_award_all_users,
)
from ..config import get_settings
from ..db import get_session
from ..models import BadgeDefinition
from ..schemas import (
    BadgeDefinitionResponse,
    BadgeEvaluationResponse,
    EarnedBadgeResponse,
)

router = APIRouter(tags=["badges"])


def _is_admin(username: str) -> bool:
    return username in get_settings().admin_usernames_list


@router.get("/api/badges", response_model=list[BadgeDefinitionResponse])
async def list_badge_catalog(
    session: AsyncSession = Depends(get_session),
) -> list[BadgeDefinitionResponse]:
    """List every badge in the catalog (public).

    Ordered by category then threshold_value ascending so the gallery can
    render tiered families top-to-bottom (bronze -> silver -> gold).
    """
    rows = (
        await session.scalars(
            select(BadgeDefinition).order_by(
                BadgeDefinition.category,
                BadgeDefinition.threshold_value,
            )
        )
    ).all()
    return [
        BadgeDefinitionResponse(
            id=r.id,
            slug=r.slug,
            name=r.name,
            description=r.description,
            icon=r.icon,
            category=r.category,
            threshold_type=r.threshold_type,
            threshold_value=r.threshold_value,
            tier=r.tier,
        )
        for r in rows
    ]


@router.get(
    "/api/users/{username}/badges",
    response_model=list[EarnedBadgeResponse],
)
async def get_user_badges(
    username: str,
    session: AsyncSession = Depends(get_session),
) -> list[EarnedBadgeResponse]:
    """List the badges a given user has earned (public)."""
    return await list_user_badges(session, username)


@router.post(
    "/api/badges/evaluate/{username}",
    response_model=BadgeEvaluationResponse,
)
async def trigger_evaluation(
    username: str,
    user: str = Depends(get_current_user),
    session: AsyncSession = Depends(get_session),
) -> BadgeEvaluationResponse:
    """Re-evaluate badges for ``username``.

    Auth: the caller must either be the same user, or an admin. We don't
    let arbitrary callers fire evaluation for anyone — even though it's
    idempotent and read-mostly, it triggers a small fan of DB reads and
    we'd rather not expose that as a public DoS vector.
    """
    if user != username and not _is_admin(user):
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Can only evaluate your own badges",
        )
    new_badges = await evaluate_user(session, username)
    all_badges = await list_user_badges(session, username)
    return BadgeEvaluationResponse(
        username=username,
        newly_awarded=new_badges,
        total_earned=len(all_badges),
    )


@router.post(
    "/api/admin/badges/retro-award",
    response_model=dict[str, int],
)
async def admin_retro_award(
    user: str = Depends(get_current_user),
    session: AsyncSession = Depends(get_session),
) -> dict[str, int]:
    """Run ``evaluate_user`` against every known username (admin only).

    Returns ``{username: newly_awarded_count}``. Useful after a fresh
    deploy of the badges feature to backfill recognition for existing
    activity.
    """
    if not _is_admin(user):
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Admin access required",
        )
    return await retro_award_all_users(session)
