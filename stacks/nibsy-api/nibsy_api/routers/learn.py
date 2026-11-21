"""Learning recommendations for the site's own pages (#632).

- ``/api/learn/for-page``   "Learn the skills": the courses that teach what a
                            blog post or project page uses.
- ``/api/learn/follow-on``  the end of a course: the next course in each of its
                            learning pathways, and projects to build next.
- ``/api/learn/helpful``    "Was this lesson helpful?" votes (#637): POST one,
                            GET the tallies per course for the health page.

Both put the page's pins first (a post's ``courses:`` front matter, a
course's ``projects:`` in course.yml), then fill from the catalogue by
similarity. Neither pads with unrelated trending items: an empty list means
nothing fits, and the page shows nothing.
"""

from __future__ import annotations

from datetime import datetime, timezone
from typing import Optional

from fastapi import APIRouter, Body, Depends, HTTPException, Query, Response
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import get_optional_user
from ..db import get_session
from ..history import load_history
from ..models import NibsyLessonVote
from ..progress import course_key
from ..recommender import (
    Item,
    Pick,
    build_profile,
    get_index,
    next_in_pathways,
    normalise_url,
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


@router.post("/helpful")
async def vote_helpful(
    body: dict = Body(...),
    session: AsyncSession = Depends(get_session),
) -> dict:
    """Count one "Was this lesson helpful?" answer.

    ``previous`` (the reader's earlier answer, which the page remembers)
    is taken back first, so changing your mind doesn't count twice.
    """

    helpful = body.get("helpful")
    previous = body.get("previous")
    if not isinstance(helpful, bool) or (previous is not None and not isinstance(previous, bool)):
        raise HTTPException(status_code=422, detail="helpful must be true or false")
    lesson = normalise_url(str(body.get("page") or ""))
    course = course_key(lesson)
    index = await get_index(session)
    source, _basis = index.resolve(lesson)
    if course is None or len(lesson) > 300 or source is None or source.type != "course":
        raise HTTPException(status_code=422, detail="not a lesson")

    now = datetime.now(timezone.utc).replace(tzinfo=None)
    row = await session.get(NibsyLessonVote, lesson)
    if row is None:
        row = NibsyLessonVote(lesson=lesson, course=course, up=0, down=0, updated_at=now)
        session.add(row)
    if previous is True:
        row.up = max(0, row.up - 1)
    elif previous is False:
        row.down = max(0, row.down - 1)
    if helpful:
        row.up += 1
    else:
        row.down += 1
    row.updated_at = now
    await session.commit()
    return {"lesson": lesson, "helpful": helpful}


@router.get("/helpful")
async def helpful_tallies(
    response: Response,
    session: AsyncSession = Depends(get_session),
) -> dict:
    """Votes per course, with each lesson's, for the course health page."""

    response.headers["Cache-Control"] = "public, max-age=600"
    courses: dict[str, dict] = {}
    for row in (await session.scalars(select(NibsyLessonVote))).all():
        entry = courses.setdefault(row.course, {"up": 0, "down": 0, "lessons": {}})
        entry["up"] += row.up
        entry["down"] += row.down
        entry["lessons"][row.lesson] = {"up": row.up, "down": row.down}
    return {"courses": courses}
