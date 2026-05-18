"""User follow / unfollow endpoints and badge computation — issue #140.

Endpoints live under ``/api/users/...`` to slot in beside the existing
``/api/users/{username}/makes`` route.

Permissions:
  * POST/DELETE follow require auth.
  * Counts and badges are public reads.
  * You may not follow yourself (400).
  * POST follow is idempotent — posting the same follow twice returns
    200, not 500.

No 14-day account-age gate: follow is a low-stakes, fully-reversible
social action. The gate exists to slow drive-by edits to the parts
catalog, not to gate every interaction.
"""

from __future__ import annotations

from typing import Optional

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy import delete as sql_delete, func, select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import get_current_user
from ..db import get_session
from ..models import Project, UserFollow
from ..schemas import (
    FollowCountResponse,
    FollowingListResponse,
    FollowToggleResponse,
)

router = APIRouter(tags=["follows"])


# --- Helpers -------------------------------------------------------------


async def _is_following(session: AsyncSession, follower: str, followee: str) -> bool:
    row = (
        await session.execute(
            select(UserFollow.follower_id).where(
                UserFollow.follower_id == follower,
                UserFollow.followee_id == followee,
            )
        )
    ).first()
    return row is not None


async def get_following_set(session: AsyncSession, follower: str) -> set[str]:
    """Return the set of usernames ``follower`` follows. Empty set if any
    error or no rows. Used by the projects list endpoint to boost
    followed authors."""
    rows = (
        await session.execute(
            select(UserFollow.followee_id).where(UserFollow.follower_id == follower)
        )
    ).scalars().all()
    return set(rows)


# --- Follow / unfollow ---------------------------------------------------


@router.post(
    "/api/users/{username}/follow",
    response_model=FollowToggleResponse,
)
async def follow_user(
    username: str,
    user: str = Depends(get_current_user),
    session: AsyncSession = Depends(get_session),
) -> FollowToggleResponse:
    if username == user:
        raise HTTPException(
            status.HTTP_400_BAD_REQUEST,
            detail="You cannot follow yourself",
        )

    existing = await _is_following(session, user, username)
    if not existing:
        session.add(UserFollow(follower_id=user, followee_id=username))
        try:
            await session.commit()
        except Exception:
            # Race-condition belt-and-braces: another request created the
            # row between our SELECT and INSERT. Treat as a successful
            # idempotent follow.
            await session.rollback()
    return FollowToggleResponse(
        follower=user, followee=username, following=True
    )


@router.delete(
    "/api/users/{username}/follow",
    response_model=FollowToggleResponse,
)
async def unfollow_user(
    username: str,
    user: str = Depends(get_current_user),
    session: AsyncSession = Depends(get_session),
) -> FollowToggleResponse:
    await session.execute(
        sql_delete(UserFollow).where(
            UserFollow.follower_id == user,
            UserFollow.followee_id == username,
        )
    )
    await session.commit()
    return FollowToggleResponse(
        follower=user, followee=username, following=False
    )


@router.get(
    "/api/users/{username}/follow",
    response_model=FollowToggleResponse,
)
async def get_follow_status(
    username: str,
    user: str = Depends(get_current_user),
    session: AsyncSession = Depends(get_session),
) -> FollowToggleResponse:
    """Whether the calling user currently follows ``username``.

    Requires auth — callers without a session don't need this signal,
    and we don't want to leak follow-graph data to drive-by scrapers.
    """
    following = await _is_following(session, user, username)
    return FollowToggleResponse(
        follower=user, followee=username, following=following
    )


@router.get(
    "/api/users/{username}/followers/count",
    response_model=FollowCountResponse,
)
async def followers_count(
    username: str,
    session: AsyncSession = Depends(get_session),
) -> FollowCountResponse:
    n = await session.scalar(
        select(func.count(UserFollow.follower_id)).where(
            UserFollow.followee_id == username
        )
    )
    return FollowCountResponse(username=username, count=int(n or 0))


@router.get(
    "/api/users/{username}/following/count",
    response_model=FollowCountResponse,
)
async def following_count(
    username: str,
    session: AsyncSession = Depends(get_session),
) -> FollowCountResponse:
    n = await session.scalar(
        select(func.count(UserFollow.followee_id)).where(
            UserFollow.follower_id == username
        )
    )
    return FollowCountResponse(username=username, count=int(n or 0))


@router.get(
    "/api/users/me/following",
    response_model=FollowingListResponse,
)
async def my_following(
    user: str = Depends(get_current_user),
    session: AsyncSession = Depends(get_session),
) -> FollowingListResponse:
    rows = (
        await session.execute(
            select(UserFollow.followee_id)
            .where(UserFollow.follower_id == user)
            .order_by(UserFollow.created_at.desc())
        )
    ).scalars().all()
    return FollowingListResponse(follower=user, following=list(rows))


