"""Public user profile endpoints — issue #111.

Provides:
  * ``GET  /api/users/{username}/profile``  — public profile + stats
  * ``PUT  /api/users/me/profile``          — self-edit profile fields
  * ``GET  /api/users/{username}/activity`` — paginated activity feed
  * ``GET  /api/users/{username}/followers``— paginated follower list
  * ``GET  /api/users/{username}/following``— paginated following list
  * ``GET  /api/users/me/follows/{username}``— "am I following X?" probe

The follow toggle endpoints live in ``follows.py`` and are still
mounted (their POST/DELETE on ``/api/users/{username}/follow``). This
router is the *profile* surface.

Identity model: we don't own a users table; Chatter is authoritative for
"does this user exist?". A ``user_profiles`` row only exists once a user
has visited the edit page. ``GET /profile`` therefore always returns a
200 with sensible defaults, even when no row exists — the page renders
just the stats + projects, and the editor can save the first row.
"""

from __future__ import annotations

import re
from datetime import datetime
from typing import Optional

from fastapi import APIRouter, Depends, HTTPException, Query, status
from sqlalchemy import func, select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import get_current_user
from ..db import get_session
from ..models import (
    Download,
    Make,
    Project,
    UserActivity,
    UserFollow,
    UserProfile,
)
from ..schemas import (
    FollowCheckResponse,
    UserActivityItem,
    UserActivityResponse,
    UserListItem,
    UserListResponse,
    UserProfileResponse,
    UserProfileStats,
    UserProfileUpdate,
    UserSocialLinks,
)

router = APIRouter(tags=["users"])


# --- URL validation ------------------------------------------------------
#
# Pydantic's HttpUrl rejects empty strings, but the UI needs a way to
# clear a previously-set URL by saving "". We therefore validate the
# shape ourselves: empty / None is allowed (= clear), otherwise it must
# start with http(s):// and be syntactically reasonable. Anything else
# raises 400 — we don't try to fetch the URL.

_URL_RE = re.compile(r"^https?://[^\s/$.?#].[^\s]*$", re.IGNORECASE)


def _validate_optional_url(value: Optional[str], field: str) -> Optional[str]:
    if value is None:
        return None
    trimmed = value.strip()
    if trimmed == "":
        return None
    if not _URL_RE.match(trimmed):
        raise HTTPException(
            status.HTTP_400_BAD_REQUEST,
            detail=f"{field} must be a valid http(s) URL",
        )
    if len(trimmed) > 200:
        raise HTTPException(
            status.HTTP_400_BAD_REQUEST,
            detail=f"{field} is too long (max 200 chars)",
        )
    return trimmed


# --- Profile GET ---------------------------------------------------------


async def _compute_stats(session: AsyncSession, username: str) -> UserProfileStats:
    """Aggregate visible counts for the profile page.

    All stats exclude blocked projects so they match the public listings.
    Archived projects are *included* in the count — they're still
    something the user made; we just don't surface them on the home page.

    Likes are tracked in Chatter (separate service); we return 0 here and
    let the frontend overlay Chatter's count if it wants to.
    """
    projects = await session.scalar(
        select(func.count(Project.id)).where(
            Project.author_username == username,
            Project.is_blocked == False,  # noqa: E712
        )
    ) or 0
    makes = await session.scalar(
        select(func.count(Make.id)).where(Make.user_id == username)
    ) or 0

    # Total downloads = sum of downloads across all this user's projects.
    project_ids_subq = (
        select(Project.id).where(Project.author_username == username)
    ).subquery()
    downloads = await session.scalar(
        select(func.count(Download.id)).where(
            Download.project_id.in_(select(project_ids_subq.c.id))
        )
    ) or 0

    followers = await session.scalar(
        select(func.count(UserFollow.follower_id)).where(
            UserFollow.followee_id == username
        )
    ) or 0
    following = await session.scalar(
        select(func.count(UserFollow.followee_id)).where(
            UserFollow.follower_id == username
        )
    ) or 0

    return UserProfileStats(
        projects=int(projects),
        makes=int(makes),
        downloads=int(downloads),
        likes=0,
        followers=int(followers),
        following=int(following),
    )


@router.get(
    "/api/users/{username}/profile",
    response_model=UserProfileResponse,
)
async def get_profile(
    username: str,
    session: AsyncSession = Depends(get_session),
) -> UserProfileResponse:
    """Public profile.

    A 404 is returned only when the user has neither a ``user_profiles``
    row nor any project / make activity tracked in this API. Once a user
    has at least one project we know they exist (Chatter created them)
    and we surface zeros for fields they haven't filled in.
    """
    profile = await session.get(UserProfile, username)
    stats = await _compute_stats(session, username)

    # If there's no profile row and no activity, treat as unknown.
    if profile is None and stats.projects == 0 and stats.makes == 0:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="User not found")

    social = (
        UserSocialLinks(**profile.social_links)
        if profile and profile.social_links
        else UserSocialLinks()
    )
    featured = (
        list(profile.featured_badge_slugs)[:3]
        if profile and profile.featured_badge_slugs
        else []
    )

    return UserProfileResponse(
        username=username,
        avatar_url=None,  # Frontend overlays Chatter's avatar.
        bio=profile.bio if profile else None,
        location=profile.location if profile else None,
        website_url=profile.website_url if profile else None,
        social_links=social,
        joined_at=profile.created_at if profile else None,
        featured_badge_slugs=featured,
        stats=stats,
    )


# --- Profile PUT (self only) --------------------------------------------


@router.put(
    "/api/users/me/profile",
    response_model=UserProfileResponse,
)
async def update_my_profile(
    body: UserProfileUpdate,
    user: str = Depends(get_current_user),
    session: AsyncSession = Depends(get_session),
) -> UserProfileResponse:
    """Patch the caller's own profile.

    Validates URL shape (allowing empty string as "clear") and trims
    every string. Creates the row lazily on first call.
    """
    # Validate URL shapes.
    website = _validate_optional_url(body.website_url, "website_url")

    social: Optional[dict] = None
    if body.social_links is not None:
        cleaned: dict = {}
        for field in ("github", "twitter", "youtube", "mastodon"):
            value = getattr(body.social_links, field, None)
            normalised = _validate_optional_url(value, f"social_links.{field}")
            if normalised is not None:
                cleaned[field] = normalised
        social = cleaned

    # Trim bio / location; treat all-whitespace as "clear".
    bio: Optional[str] = None
    if body.bio is not None:
        bio = body.bio.strip()
        if bio == "":
            bio = None
        if bio is not None and len(bio) > 500:
            # Pydantic's max_length should already catch this, but assert
            # defensively for any path that bypasses it.
            raise HTTPException(
                status.HTTP_400_BAD_REQUEST,
                detail="bio is too long (max 500 chars)",
            )
    location: Optional[str] = None
    if body.location is not None:
        location = body.location.strip()
        if location == "":
            location = None

    # Validate featured_badge_slugs: at most 3, each ≤80 chars,
    # case-insensitive uniqueness preserved in insertion order.
    featured: Optional[list[str]] = None
    if body.featured_badge_slugs is not None:
        seen: set[str] = set()
        cleaned_slugs: list[str] = []
        for slug in body.featured_badge_slugs:
            if not isinstance(slug, str):
                continue
            s = slug.strip()
            if s == "" or len(s) > 80:
                continue
            key = s.lower()
            if key in seen:
                continue
            seen.add(key)
            cleaned_slugs.append(s)
        if len(cleaned_slugs) > 3:
            raise HTTPException(
                status.HTTP_400_BAD_REQUEST,
                detail="featured_badge_slugs may contain at most 3 slugs",
            )
        featured = cleaned_slugs

    profile = await session.get(UserProfile, user)
    now = datetime.utcnow()
    if profile is None:
        profile = UserProfile(
            username=user,
            bio=bio,
            location=location,
            website_url=website,
            social_links=social,
            featured_badge_slugs=featured,
            created_at=now,
            updated_at=now,
        )
        session.add(profile)
    else:
        # Only overwrite fields the caller explicitly sent. Pydantic
        # gives us "set" vs "unset" via model_fields_set.
        sent = body.model_fields_set
        if "bio" in sent:
            profile.bio = bio
        if "location" in sent:
            profile.location = location
        if "website_url" in sent:
            profile.website_url = website
        if "social_links" in sent:
            profile.social_links = social
        if "featured_badge_slugs" in sent:
            profile.featured_badge_slugs = featured
        profile.updated_at = now

    await session.commit()
    await session.refresh(profile)

    return await get_profile(user, session=session)


# --- Activity feed -------------------------------------------------------


def _activity_item(row: UserActivity) -> UserActivityItem:
    return UserActivityItem(
        id=row.id,
        kind=row.kind,
        subject_id=row.subject_id,
        subject_title=row.subject_title,
        subject_url=row.subject_url,
        created_at=row.created_at,
    )


@router.get(
    "/api/users/{username}/activity",
    response_model=UserActivityResponse,
)
async def get_activity(
    username: str,
    limit: int = Query(20, ge=1, le=100),
    offset: int = Query(0, ge=0),
    session: AsyncSession = Depends(get_session),
) -> UserActivityResponse:
    rows = (
        await session.execute(
            select(UserActivity)
            .where(UserActivity.user_id == username)
            .order_by(UserActivity.created_at.desc(), UserActivity.id.desc())
            .limit(limit)
            .offset(offset)
        )
    ).scalars().all()
    return UserActivityResponse(
        username=username,
        items=[_activity_item(r) for r in rows],
        limit=limit,
        offset=offset,
    )


# --- Follower / following lists -----------------------------------------


@router.get(
    "/api/users/{username}/followers",
    response_model=UserListResponse,
)
async def list_followers(
    username: str,
    limit: int = Query(50, ge=1, le=200),
    offset: int = Query(0, ge=0),
    session: AsyncSession = Depends(get_session),
) -> UserListResponse:
    total = int(await session.scalar(
        select(func.count(UserFollow.follower_id)).where(
            UserFollow.followee_id == username
        )
    ) or 0)
    rows = (
        await session.execute(
            select(UserFollow.follower_id)
            .where(UserFollow.followee_id == username)
            .order_by(UserFollow.created_at.desc())
            .limit(limit)
            .offset(offset)
        )
    ).scalars().all()
    return UserListResponse(
        username=username,
        users=[UserListItem(username=u) for u in rows],
        total=total,
        limit=limit,
        offset=offset,
    )


@router.get(
    "/api/users/{username}/following",
    response_model=UserListResponse,
)
async def list_following(
    username: str,
    limit: int = Query(50, ge=1, le=200),
    offset: int = Query(0, ge=0),
    session: AsyncSession = Depends(get_session),
) -> UserListResponse:
    total = int(await session.scalar(
        select(func.count(UserFollow.followee_id)).where(
            UserFollow.follower_id == username
        )
    ) or 0)
    rows = (
        await session.execute(
            select(UserFollow.followee_id)
            .where(UserFollow.follower_id == username)
            .order_by(UserFollow.created_at.desc())
            .limit(limit)
            .offset(offset)
        )
    ).scalars().all()
    return UserListResponse(
        username=username,
        users=[UserListItem(username=u) for u in rows],
        total=total,
        limit=limit,
        offset=offset,
    )


# --- Follow probe (am I following X?) -----------------------------------


@router.get(
    "/api/users/me/follows/{username}",
    response_model=FollowCheckResponse,
)
async def check_follow(
    username: str,
    user: str = Depends(get_current_user),
    session: AsyncSession = Depends(get_session),
) -> FollowCheckResponse:
    """Quick yes/no for the follow button state.

    Mirrors the existing ``GET /api/users/{username}/follow`` in follows.py
    but uses a cleaner URL shape so the JS can stop probing once it knows
    it's not auth'd (this endpoint returns 401, the other returns 401
    too — both are fine).
    """
    row = (
        await session.execute(
            select(UserFollow.follower_id).where(
                UserFollow.follower_id == user,
                UserFollow.followee_id == username,
            )
        )
    ).first()
    return FollowCheckResponse(following=row is not None)


# --- Activity emission helper -------------------------------------------
#
# Imported by other routers (projects, makes) to log activity events.
# Kept in this module so the list of valid `kind` values lives next to
# the schema that consumes them. Routes that emit must call this within
# the same DB transaction so partial commits aren't possible.


_VALID_KINDS = frozenset({
    "project_published",
    "project_updated",
    "comment_posted",
    "make_posted",
    "badge_earned",
    "collection_created",
})


async def log_activity(
    session: AsyncSession,
    *,
    user_id: str,
    kind: str,
    subject_id: Optional[int] = None,
    subject_title: Optional[str] = None,
    subject_url: Optional[str] = None,
) -> None:
    """Insert a single activity row. Caller commits.

    Silently no-ops on unknown kinds rather than 500ing — activity is a
    nice-to-have, not a hard requirement; an emit-site mistake should
    not break the user-facing operation.
    """
    if kind not in _VALID_KINDS:
        return
    session.add(UserActivity(
        user_id=user_id,
        kind=kind,
        subject_id=subject_id,
        subject_title=(subject_title or "")[:200] or None,
        subject_url=(subject_url or "")[:400] or None,
    ))
