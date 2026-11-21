"""Learning recommendations for the site's own pages (#632).

- ``/api/learn/for-page``   "Learn the skills": the courses that teach what a
                            blog post or project page uses.
- ``/api/learn/follow-on``  the end of a course: the next course in each of its
                            learning pathways, and projects to build next.

Both put the page's pins first (a post's ``courses:`` front matter, a
course's ``projects:`` in course.yml), then fill from the catalogue by
similarity. Neither pads with unrelated trending items: an empty list means
nothing fits, and the page shows nothing.
"""

from __future__ import annotations

from datetime import datetime, timezone
from typing import Optional

from fastapi import APIRouter, Depends, Query, Response
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import get_optional_user
from ..db import get_session
from ..history import load_history
from ..recommender import (
    Item,
    Pick,
    build_profile,
    get_index,
    next_in_pathways,
    related_of_kind,
)

router = APIRouter(prefix="/api/learn", tags=["learn"])

_MAX_LIMIT = 6


def card(item: Item, reason: str = "") -> dict:
    """What a page needs to draw a course or project card."""

    meta = item.meta
    return {
        "content_id": item.id,
        "type": item.type,
        "title": item.title,
        "url": item.url,
        "cover": meta.get("cover"),
        "level": meta.get("level"),
        "duration": meta.get("duration"),
        "reason": reason,
    }


def _cards(picks: list[Pick]) -> list[dict]:
    return [card(p.item, p.reason) for p in picks]


@router.get("/for-page")
async def courses_for_page(
    response: Response,
    page: str = Query(..., description="URL path of the post or project page"),
    limit: int = Query(2, ge=1, le=_MAX_LIMIT),
    session: AsyncSession = Depends(get_session),
    user: Optional[str] = Depends(get_optional_user),
) -> dict:
    """Courses that teach the skills a post or project page uses."""

    response.headers["Cache-Control"] = "private, no-store"
    index = await get_index(session)
    source, basis = index.resolve(page)
    if source is None:
        return {"page": page, "basis": basis, "courses": []}

    now = datetime.now(timezone.utc).replace(tzinfo=None)
    profile = None
    if user:
        profile = build_profile(index, await load_history(session, user, now), now)
    picks = related_of_kind(
        index,
        source,
        lambda item: item.type == "course",
        limit,
        set(),
        now,
        pins=source.meta.get("pins") or [],
        profile=profile,
    )
    return {"page": page, "basis": basis, "courses": _cards(picks)}


@router.get("/follow-on")
async def follow_on(
    response: Response,
    course: str = Query(..., description="URL of the course or any of its lessons"),
    limit: int = Query(3, ge=1, le=_MAX_LIMIT),
    session: AsyncSession = Depends(get_session),
) -> dict:
    """What to do after a course: the next course in each path, and projects to build."""

    response.headers["Cache-Control"] = "public, max-age=3600"
    index = await get_index(session)
    source, _basis = index.resolve(course)
    if source is None or source.type != "course":
        return {"course": course, "next": [], "projects": []}

    now = datetime.now(timezone.utc).replace(tzinfo=None)
    nexts = [
        {
            "pathway": {"slug": entry.get("slug"), "title": entry.get("title"), "url": entry.get("url")},
            "course": card(item, f"Next in {entry.get('title')}"),
        }
        for entry, item in next_in_pathways(index, source)
    ]
    projects = related_of_kind(
        index,
        source,
        lambda item: item.is_project,
        limit,
        set(),
        now,
        pins=source.meta.get("pins") or [],
    )
    return {
        "course": card(source),
        "pathways": source.meta.get("pathways") or [],
        "next": nexts,
        "projects": _cards(projects),
    }
