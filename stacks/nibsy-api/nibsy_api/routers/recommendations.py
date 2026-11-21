"""Public recommendations endpoints (issue #67, #74).

`/api/recommendations` is computed live for the page the visitor is on
(see `recommender.py`). It always answers 200, with an empty list only
when the catalogue itself is empty — Nibsy is best-effort by design and
the widget should degrade gracefully, never throw.
"""

from __future__ import annotations

import logging
from datetime import datetime, timedelta, timezone
from typing import Optional

from fastapi import APIRouter, Depends, HTTPException, Query, Response, status
from sqlalchemy import func, select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import get_optional_user
from ..config import get_settings
from ..db import get_session
from ..history import load_history
from ..generator import _jaccard, _affinity, _recency
from ..models import NibsyClick, NibsyContent, NibsyTrending
from ..recommender import (
    ALGORITHM_VERSION,
    Pick,
    build_profile,
    get_index,
    load_trending,
    merge_views,
    next_in_pathways,
    recommend_for_page,
    related_of_kind,
    views_from_pages,
)
from ..schemas import (
    RecommendationItem,
    RecommendationsRequest,
    RecommendationsResponse,
    SeenPage,
)

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api", tags=["recommendations"])


# The widget shows a handful of tips; there is no reason to return more.
_MAX_LIMIT = 6


def _parse_exclude(raw: Optional[str]) -> set[int]:
    """Parse `?exclude=1,2,3` into a set of ints, tolerantly."""

    if not raw:
        return set()
    out: set[int] = set()
    for part in raw.split(","):
        part = part.strip()
        if not part:
            continue
        try:
            out.add(int(part))
        except ValueError:
            # Silently ignore non-int tokens — the caller is the widget,
            # not a developer, and we don't want a bad URL to break the page.
            continue
    return out


def _card(pick: Pick) -> RecommendationItem:
    """One recommendation with what a page needs to draw it as a card."""

    item, meta = pick.item, pick.item.meta
    video_id = meta.get("video_id")
    if not video_id:
        videos = meta.get("videos")
        if isinstance(videos, list) and videos:
            video_id = str(videos[0])
        elif isinstance(videos, str) and videos:
            video_id = videos
    cover = meta.get("cover")
    if not cover and item.type == "video" and video_id:
        cover = f"https://img.youtube.com/vi/{video_id}/mqdefault.jpg"
    return RecommendationItem(
        content_id=item.id,
        url=item.url,
        title=item.title,
        type=item.type,
        reason=pick.reason,
        score=round(pick.score, 4),
        description=item.description,
        cover=cover,
        date=item.date.strftime("%Y-%m-%d") if item.date else None,
        video_id=video_id,
        level=meta.get("level"),
        duration=meta.get("duration"),
    )


def _seen_pages(seen: list[SeenPage], now: datetime) -> list[tuple[str, datetime]]:
    """A browser's reading list as ``(page, when)``, dropping expired views."""

    cutoff = now - timedelta(days=get_settings().view_history_days)
    out: list[tuple[str, datetime]] = []
    for entry in seen:
        when = now
        if entry.at:
            try:
                when = min(
                    now,
                    datetime.fromtimestamp(entry.at / 1000, tz=timezone.utc).replace(
                        tzinfo=None
                    ),
                )
            except (OverflowError, OSError, ValueError):
                continue
        if when >= cutoff:
            out.append((entry.page, when))
    return out


async def _recommend(
    session: AsyncSession,
    user: Optional[str],
    page: str,
    limit: int,
    excluded: set[int],
    types: Optional[set[str]] = None,
    seen: Optional[list[SeenPage]] = None,
    course_limit: int = 0,
) -> RecommendationsResponse:
    """Shared by the GET (widget) and POST (read-next section) forms."""

    index = await get_index(session)
    if not index.items:
        return RecommendationsResponse(source_url=page, logged_in=user is not None)

    now = datetime.now(timezone.utc).replace(tzinfo=None)
    history = await load_history(session, user, now) if user else []
    if seen:
        # What this browser read is combined with any stored history for
        # this one answer, and is not written anywhere.
        history = merge_views(history, views_from_pages(index, _seen_pages(seen, now)))
    picks, basis = recommend_for_page(
        index,
        page,
        limit,
        excluded,
        await load_trending(session),
        now,
        history,
        types or None,
    )

    courses: list[Pick] = []
    if course_limit:
        source, _basis = index.resolve(page)
        profile = build_profile(index, history, now) if history else None
        courses = related_of_kind(
            index,
            source,
            lambda item: item.type == "course",
            course_limit,
            set(excluded),
            now,
            pins=(source.meta.get("pins") or []) if source is not None else [],
            profile=profile,
        )

    return RecommendationsResponse(
        logged_in=user is not None,
        personalised=any(p.personal for p in picks),
        source_url=page,
        recommendations=[_card(p) for p in picks],
        courses=[_card(p) for p in courses],
        generated_at=now,
        generator_version=ALGORITHM_VERSION,
        basis=basis,
    )


@router.get("/recommendations", response_model=RecommendationsResponse)
async def get_recommendations(
    response: Response,
    page: str = Query(..., description="URL path of the current page"),
    limit: int = Query(5, ge=1, le=_MAX_LIMIT),
    exclude: Optional[str] = Query(
        None, description="Comma-separated content IDs to exclude"
    ),
    session: AsyncSession = Depends(get_session),
    user: Optional[str] = Depends(get_optional_user),
) -> RecommendationsResponse:
    """Return recommendations for the page the visitor is on.

    Computed on every request from the current catalogue — see
    ``recommender.py`` for how pages are compared. For a logged-in
    reader the picks are also shaped by their viewing history.
    """

    # Personalised responses must never be cached and served to someone else.
    response.headers["Cache-Control"] = "private, no-store"
    return await _recommend(session, user, page, limit, _parse_exclude(exclude))


@router.post("/recommendations", response_model=RecommendationsResponse)
async def post_recommendations(
    response: Response,
    body: RecommendationsRequest,
    session: AsyncSession = Depends(get_session),
    user: Optional[str] = Depends(get_optional_user),
) -> RecommendationsResponse:
    """Recommendations with options: the "read next" section on blog posts.

    Same picks as the GET form, plus:

    - ``types`` keeps only those content types (the section asks for posts),
    - ``seen`` is the reading history an anonymous reader's browser keeps.
      It shapes this answer only and is never stored; it rides in the body
      so it never lands in an access log,
    - ``courses`` also returns that many related courses.
    """

    response.headers["Cache-Control"] = "private, no-store"
    return await _recommend(
        session,
        user,
        body.page,
        body.limit,
        set(body.exclude),
        {t for t in body.types if t},
        body.seen,
        body.courses,
    )


@router.get("/related/{content_id}")
async def get_related(
    content_id: int,
    limit: int = Query(5, ge=1, le=10),
    session: AsyncSession = Depends(get_session),
) -> dict:
    source = await session.get(NibsyContent, content_id)
    if source is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Content not found")

    now = datetime.now(timezone.utc).replace(tzinfo=None)
    all_content = (await session.scalars(select(NibsyContent))).all()

    scored = []
    for target in all_content:
        if target.id == source.id:
            continue
        tag_overlap = _jaccard(source.tags, target.tags)
        affinity = _affinity(source.content_type, target.content_type)
        recency = _recency(target.date_published, now)
        score = 1.0 * tag_overlap + 0.5 * affinity + 0.3 * recency
        if score <= 0:
            continue
        scored.append((score, target))

    scored.sort(key=lambda t: t[0], reverse=True)

    return {
        "related": [
            {
                "id": t.id,
                "type": t.content_type,
                "title": t.title,
                "url": t.url,
                "score": round(s, 4),
            }
            for s, t in scored[:limit]
        ]
    }


@router.get("/recommendations/next-course")
async def get_next_course(
    current: str = Query(..., description="URL or content ID of the current course"),
    session: AsyncSession = Depends(get_session),
) -> dict:
    """Recommend the next course to take after the current one (#76).

    If the current course belongs to a learning pathway, return the next
    course in it. Otherwise, fall back to
    the top related courses by tag overlap and recency.
    """

    # Resolve by URL or content ID.
    try:
        content_id = int(current)
        source = await session.get(NibsyContent, content_id)
    except ValueError:
        source = await session.scalar(
            select(NibsyContent).where(NibsyContent.url == current)
        )

    if source is None or source.content_type != "course":
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Course not found")

    # The next course in the first learning pathway that continues past
    # this one (pathways come from the site catalogue, #632).
    index = await get_index(session)
    item = index.by_id.get(source.id)
    nexts = next_in_pathways(index, item) if item is not None else []
    if nexts:
        entry, nxt = nexts[0]
        pathway_name = entry.get("title") or entry.get("slug")
        return {
            "source": "pathway",
            "pathway": pathway_name,
            "pathway_url": entry.get("url"),
            "next_course": {
                "id": nxt.id,
                "title": nxt.title,
                "url": nxt.url,
                "description": (await session.get(NibsyContent, nxt.id)).description,
                "reason": f"Next in {pathway_name}",
            },
        }

    # Fallback: top related courses by tag/affinity scoring.
    now = datetime.now(timezone.utc).replace(tzinfo=None)
    all_courses = (
        await session.scalars(
            select(NibsyContent).where(NibsyContent.content_type == "course")
        )
    ).all()

    scored = []
    for target in all_courses:
        if target.id == source.id:
            continue
        tag_overlap = _jaccard(source.tags, target.tags)
        recency = _recency(target.date_published, now)
        score = 1.0 * tag_overlap + 0.3 * recency
        if score <= 0:
            continue
        scored.append((score, target))

    scored.sort(key=lambda t: t[0], reverse=True)
    top = scored[:3]

    return {
        "source": "related",
        "next_courses": [
            {
                "id": t.id,
                "title": t.title,
                "url": t.url,
                "description": t.description,
                "reason": "Related course",
                "score": round(s, 4),
            }
            for s, t in top
        ],
    }


@router.get("/trending")
async def get_trending(
    limit: int = Query(5, ge=1, le=20),
    content_type: Optional[str] = Query(None, description="Filter by content type"),
    site_only: bool = Query(True, description="Exclude external URLs (YouTube etc.)"),
    session: AsyncSession = Depends(get_session),
) -> dict:
    query = (
        select(
            NibsyContent.id,
            NibsyContent.content_type,
            NibsyContent.title,
            NibsyContent.url,
            NibsyTrending.trending_score,
            NibsyTrending.nibsy_clicks,
            NibsyTrending.page_views,
            NibsyTrending.youtube_views,
        )
        .join(NibsyTrending, NibsyTrending.content_id == NibsyContent.id)
        .order_by(NibsyTrending.trending_score.desc())
        .limit(limit)
    )
    if content_type:
        query = query.where(NibsyContent.content_type == content_type)
    if site_only:
        query = query.where(NibsyContent.url.like("/%"))

    rows = (await session.execute(query)).all()

    if not rows:
        # Fallback: if no precomputed trending data, use raw click counts.
        cutoff = datetime.now(timezone.utc).replace(tzinfo=None) - timedelta(days=7)
        fallback_query = (
            select(
                NibsyContent.id,
                NibsyContent.content_type,
                NibsyContent.title,
                NibsyContent.url,
                func.count(NibsyClick.id).label("views"),
            )
            .join(NibsyClick, NibsyClick.content_url == NibsyContent.url)
            .where(NibsyClick.clicked_at >= cutoff)
            .group_by(NibsyContent.id)
            .order_by(func.count(NibsyClick.id).desc())
            .limit(limit)
        )
        if content_type:
            fallback_query = fallback_query.where(NibsyContent.content_type == content_type)
        fallback_rows = (await session.execute(fallback_query)).all()
        return {
            "trending": [
                {
                    "id": r.id,
                    "type": r.content_type,
                    "title": r.title,
                    "url": r.url,
                    "score": r.views,
                }
                for r in fallback_rows
            ]
        }

    return {
        "trending": [
            {
                "id": r.id,
                "type": r.content_type,
                "title": r.title,
                "url": r.url,
                "score": round(r.trending_score, 2),
                "nibsy_clicks": r.nibsy_clicks,
                "page_views": r.page_views,
                "youtube_views": r.youtube_views,
            }
            for r in rows
        ]
    }
