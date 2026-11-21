"""A logged-in user's own Nibsy data: see it, clear it.

- ``/api/me/history``   pages viewed, for personalised recommendations.
- ``/api/me/progress``  course progress (#633), synced from lesson pages.
- ``/api/me/learning-settings``  whether finished courses show on the
  learner's public profile (#638); off by default.
- ``/api/learners/<username>/courses``  those finished courses, public, and
  empty unless the learner turned it on.
"""

from __future__ import annotations

from datetime import datetime, timezone

from fastapi import APIRouter, Body, Depends, HTTPException, Query, Response
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import get_current_user
from ..db import get_session
from ..history import clear_history
from ..progress import clear_progress, course_key, load_progress, merge_progress
from ..models import NibsyContent, NibsyCourseProgress, NibsyLearnerSettings, NibsyPageView
from ..recommender import get_index

router = APIRouter(prefix="/api/me", tags=["me"])
public_router = APIRouter(prefix="/api/learners", tags=["me"])


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


@router.get("/progress")
async def my_progress(
    response: Response,
    session: AsyncSession = Depends(get_session),
    user: str = Depends(get_current_user),
) -> dict:
    """Every course this learner has started, most recently active first."""

    response.headers["Cache-Control"] = "private, no-store"
    return {"courses": await load_progress(session, user)}


@router.post("/progress")
async def sync_my_progress(
    response: Response,
    body: dict = Body(...),
    session: AsyncSession = Depends(get_session),
    user: str = Depends(get_current_user),
) -> dict:
    """Merge the browser's course records into the stored ones.

    Lesson pages send their course after each lesson and quiz answer; the
    first visit after signing in sends every course in the browser. The
    reply is the learner's full stored progress, so the browser can pick up
    what was done on another device.
    """

    response.headers["Cache-Control"] = "private, no-store"
    records = body.get("courses")
    if not isinstance(records, list):
        raise HTTPException(status_code=422, detail="courses must be a list")
    now = datetime.now(timezone.utc).replace(tzinfo=None)
    return {"courses": await merge_progress(session, user, records, now)}


@router.delete("/progress")
async def clear_my_progress(
    course: str | None = Query(None, description="Course to reset; all courses when left out"),
    session: AsyncSession = Depends(get_session),
    user: str = Depends(get_current_user),
) -> dict:
    """Reset one course, or forget all course progress."""

    key = None
    if course is not None:
        key = course_key(course)
        if key is None:
            raise HTTPException(status_code=422, detail="not a course URL")
    return {"deleted": await clear_progress(session, user, key)}


async def _show_courses(session: AsyncSession, user: str) -> bool:
    row = await session.get(NibsyLearnerSettings, user)
    return bool(row and row.show_courses)


@router.get("/learning-settings")
async def my_learning_settings(
    response: Response,
    session: AsyncSession = Depends(get_session),
    user: str = Depends(get_current_user),
) -> dict:
    response.headers["Cache-Control"] = "private, no-store"
    return {"username": user, "show_courses": await _show_courses(session, user)}


@router.post("/learning-settings")
async def set_my_learning_settings(
    response: Response,
    body: dict = Body(...),
    session: AsyncSession = Depends(get_session),
    user: str = Depends(get_current_user),
) -> dict:
    """Turn "show my finished courses on my profile" on or off."""

    response.headers["Cache-Control"] = "private, no-store"
    show = body.get("show_courses")
    if not isinstance(show, bool):
        raise HTTPException(status_code=422, detail="show_courses must be true or false")
    now = datetime.now(timezone.utc).replace(tzinfo=None)
    row = await session.get(NibsyLearnerSettings, user)
    if row is None:
        session.add(NibsyLearnerSettings(username=user, show_courses=show, updated_at=now))
    else:
        row.show_courses = show
        row.updated_at = now
    await session.commit()
    return {"username": user, "show_courses": show}


@public_router.get("/{username}/courses")
async def learner_courses(
    username: str,
    response: Response,
    session: AsyncSession = Depends(get_session),
) -> dict:
    """The courses a learner has finished, if they chose to show them."""

    response.headers["Cache-Control"] = "public, max-age=300"
    if not await _show_courses(session, username):
        return {"username": username, "courses": []}
    rows = (
        await session.scalars(
            select(NibsyCourseProgress)
            .where(
                NibsyCourseProgress.username == username,
                NibsyCourseProgress.completed_at.is_not(None),
            )
            .order_by(NibsyCourseProgress.completed_at.desc())
        )
    ).all()
    index = await get_index(session)
    courses = []
    for row in rows:
        item, _basis = index.resolve(row.course)
        if item is None or item.type != "course":
            continue
        courses.append(
            {
                "title": item.title,
                "url": item.url,
                "cover": item.meta.get("cover"),
                "level": item.meta.get("level"),
                "completed": row.completed_at.replace(tzinfo=timezone.utc).date().isoformat(),
            }
        )
    return {"username": username, "courses": courses}
