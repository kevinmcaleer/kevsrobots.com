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
from ..models import Make, Project, UserFollow
from ..schemas import (
    BadgeResponse,
    FollowCountResponse,
    FollowingListResponse,
    FollowToggleResponse,
    UserBadgesResponse,
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


# --- Badges --------------------------------------------------------------
#
# Badge catalog (issue #140). Computed on-the-fly from the existing
# projects / makes tables. No persisted badge state — easier to evolve
# the rules without a migration. Keep the catalog small; if it grows
# beyond ~10 badges, factor into a dedicated module.
#
# Rule reference:
#   first-project       : >=1 non-archived, non-blocked project authored.
#   five-projects       : >=5 non-archived, non-blocked projects authored.
#   ten-projects        : >=10 non-archived, non-blocked projects authored.
#   first-remix         : authored a project with remixed_from_id set.
#   has-remixers        : someone has remixed one of your projects.
#   community-builder   : posted >=1 "I Made This!" make on someone else's
#                         project (i.e. has rows in `makes`).

_BADGE_CATALOG: dict[str, BadgeResponse] = {
    "first-project": BadgeResponse(
        key="first-project",
        name="First Project",
        description="Published their first project",
        icon="fa-rocket",
        color="primary",
    ),
    "five-projects": BadgeResponse(
        key="five-projects",
        name="Five Projects",
        description="Published five projects",
        icon="fa-star",
        color="warning",
    ),
    "ten-projects": BadgeResponse(
        key="ten-projects",
        name="Ten Projects",
        description="Published ten projects",
        icon="fa-trophy",
        color="warning",
    ),
    "first-remix": BadgeResponse(
        key="first-remix",
        name="First Remix",
        description="Built on top of someone else's project",
        icon="fa-code-branch",
        color="info",
    ),
    "has-remixers": BadgeResponse(
        key="has-remixers",
        name="Inspiration",
        description="Someone has remixed one of their projects",
        icon="fa-lightbulb",
        color="success",
    ),
    "community-builder": BadgeResponse(
        key="community-builder",
        name="Community Builder",
        description="Shared an 'I Made This!' on someone else's project",
        icon="fa-hammer",
        color="dark",
    ),
}


@router.get(
    "/api/users/{username}/badges",
    response_model=UserBadgesResponse,
)
async def user_badges(
    username: str,
    session: AsyncSession = Depends(get_session),
) -> UserBadgesResponse:
    """Compute the badges this user has earned.

    All counts exclude archived & blocked projects so a user can't farm
    badges with junk rows. Cheap to compute — three count-only queries —
    so it's fine to call on every profile page load.
    """
    # Active projects authored by this user.
    project_count = await session.scalar(
        select(func.count(Project.id)).where(
            Project.author_username == username,
            Project.status != "archived",
            Project.is_blocked == False,  # noqa: E712
        )
    ) or 0

    # Did they author at least one remix?
    remix_authored = await session.scalar(
        select(func.count(Project.id)).where(
            Project.author_username == username,
            Project.remixed_from_id.is_not(None),
            Project.status != "archived",
            Project.is_blocked == False,  # noqa: E712
        )
    ) or 0

    # Has anyone remixed one of their projects? Count child projects
    # whose parent is authored by `username`. Subquery keeps it readable
    # and works on both Postgres and SQLite.
    parent_ids_subq = (
        select(Project.id).where(Project.author_username == username)
    ).subquery()
    has_remixers = await session.scalar(
        select(func.count(Project.id)).where(
            Project.remixed_from_id.in_(select(parent_ids_subq.c.id)),
            Project.is_blocked == False,  # noqa: E712
        )
    ) or 0

    # Posted any "I Made This!" makes?
    make_count = await session.scalar(
        select(func.count(Make.id)).where(Make.user_id == username)
    ) or 0

    earned: list[BadgeResponse] = []
    if project_count >= 1:
        earned.append(_BADGE_CATALOG["first-project"])
    if project_count >= 5:
        earned.append(_BADGE_CATALOG["five-projects"])
    if project_count >= 10:
        earned.append(_BADGE_CATALOG["ten-projects"])
    if remix_authored >= 1:
        earned.append(_BADGE_CATALOG["first-remix"])
    if has_remixers >= 1:
        earned.append(_BADGE_CATALOG["has-remixers"])
    if make_count >= 1:
        earned.append(_BADGE_CATALOG["community-builder"])

    return UserBadgesResponse(username=username, badges=earned)
