"""Learner course progress (#633): merge a browser's record with the stored one.

The browser record (course-progress.js) looks like::

    {"course": "/learn/micropython/", "visited": ["00_intro", ...],
     "answers": {"q1": true}, "last": "03_leds", "updated": <ms>,
     "completed": <ms or null>}

Merging never loses progress: visited lessons are a union, the first stored
answer to a question wins (only the first try counts), the most recent
``last`` lesson wins, and the earliest completion is kept.
"""

from __future__ import annotations

import re
from datetime import datetime, timezone
from typing import Any, Optional

from sqlalchemy import delete, select
from sqlalchemy.ext.asyncio import AsyncSession

from .models import NibsyCourseProgress

MAX_COURSES = 200
MAX_LESSONS = 400
MAX_ANSWERS = 400

_COURSE = re.compile(r"^/learn/[A-Za-z0-9_\-./]{1,250}/$")
_ID = re.compile(r"^[A-Za-z0-9_\-.]{1,150}$")


def course_key(url: str) -> Optional[str]:
    """The course folder for a course or lesson URL, or None if it isn't one."""

    if not isinstance(url, str):
        return None
    key = url if url.endswith("/") else url[: url.rfind("/") + 1]
    if ".." in key or not _COURSE.match(key) or key.startswith("/learn/learning_pathways/"):
        return None
    return key


def _when(ms: Any) -> Optional[datetime]:
    try:
        value = float(ms)
    except (TypeError, ValueError):
        return None
    if value <= 0:
        return None
    try:
        return datetime.fromtimestamp(value / 1000, tz=timezone.utc).replace(tzinfo=None)
    except (OverflowError, OSError, ValueError):
        return None


def _ms(when: Optional[datetime]) -> Optional[int]:
    if when is None:
        return None
    return int(when.replace(tzinfo=timezone.utc).timestamp() * 1000)


def clean(record: Any, now: datetime) -> Optional[dict]:
    """A validated browser record, or None if it can't be used."""

    if not isinstance(record, dict):
        return None
    key = course_key(record.get("course", ""))
    if key is None:
        return None
    raw_visited = record.get("visited") or []
    if isinstance(raw_visited, dict):
        raw_visited = [k for k, v in raw_visited.items() if v]
    visited = [v for v in raw_visited if isinstance(v, str) and _ID.match(v) and v != "index"]
    raw_answers = record.get("answers") or {}
    answers = {
        k: bool(v)
        for k, v in (raw_answers.items() if isinstance(raw_answers, dict) else [])
        if isinstance(k, str) and _ID.match(k) and isinstance(v, bool)
    }
    last = record.get("last")
    # Never trust a clock from the future.
    updated = min(_when(record.get("updated")) or now, now)
    completed = _when(record.get("completed"))
    return {
        "course": key,
        "visited": visited[:MAX_LESSONS],
        "answers": dict(list(answers.items())[:MAX_ANSWERS]),
        "last": last if isinstance(last, str) and _ID.match(last) else None,
        "updated": updated,
        "completed": min(completed, now) if completed else None,
    }


def as_record(row: NibsyCourseProgress) -> dict:
    """The stored row in the browser's shape."""

    return {
        "course": row.course,
        "visited": list(row.visited or []),
        "answers": dict(row.answers or {}),
        "last": row.last_lesson,
        "started": _ms(row.started_at),
        "updated": _ms(row.updated_at),
        "completed": _ms(row.completed_at),
    }


def _merge_into(row: NibsyCourseProgress, rec: dict) -> None:
    visited = list(row.visited or [])
    seen = set(visited)
    visited += [v for v in rec["visited"] if v not in seen]
    row.visited = visited[:MAX_LESSONS]
    answers = dict(rec["answers"])
    answers.update(row.answers or {})  # what was stored first wins
    row.answers = dict(list(answers.items())[:MAX_ANSWERS])
    if rec["last"] and (row.last_lesson is None or rec["updated"] >= row.updated_at):
        row.last_lesson = rec["last"]
    row.updated_at = max(row.updated_at, rec["updated"])
    row.started_at = min(row.started_at, rec["updated"])
    if rec["completed"] and (row.completed_at is None or rec["completed"] < row.completed_at):
        row.completed_at = rec["completed"]


async def merge_progress(session: AsyncSession, user: str, records: list, now: datetime) -> list[dict]:
    """Merge browser records into the user's stored progress; return what is stored."""

    cleaned = [c for c in (clean(r, now) for r in records[:MAX_COURSES]) if c]
    if cleaned:
        existing = {
            row.course: row
            for row in (
                await session.scalars(
                    select(NibsyCourseProgress).where(
                        NibsyCourseProgress.username == user,
                        NibsyCourseProgress.course.in_({c["course"] for c in cleaned}),
                    )
                )
            ).all()
        }
        for rec in cleaned:
            row = existing.get(rec["course"])
            if row is None:
                if not rec["visited"] and not rec["answers"]:
                    continue
                row = NibsyCourseProgress(
                    username=user,
                    course=rec["course"],
                    visited=[],
                    answers={},
                    last_lesson=None,
                    started_at=rec["updated"],
                    updated_at=rec["updated"],
                )
                session.add(row)
                existing[rec["course"]] = row
            _merge_into(row, rec)
        await session.commit()
    return await load_progress(session, user)


async def load_progress(session: AsyncSession, user: str) -> list[dict]:
    rows = (
        await session.scalars(
            select(NibsyCourseProgress)
            .where(NibsyCourseProgress.username == user)
            .order_by(NibsyCourseProgress.updated_at.desc())
        )
    ).all()
    return [as_record(r) for r in rows]


async def clear_progress(session: AsyncSession, user: str, course: Optional[str] = None) -> int:
    stmt = delete(NibsyCourseProgress).where(NibsyCourseProgress.username == user)
    if course is not None:
        stmt = stmt.where(NibsyCourseProgress.course == course)
    result = await session.execute(stmt)
    await session.commit()
    return result.rowcount or 0
