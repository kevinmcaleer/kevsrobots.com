"""Parts talk-pages endpoints (issue #122 Phase 2).

Wikipedia-style discussion threads attached to a Part. Each thread is an
append-only sequence of posts. The first post is written atomically with
the thread on creation; subsequent posts go through
``POST .../posts``.

Auth + age gating
-----------------
* Thread *creation* requires the 14-day account-age gate (via
  ``get_current_user_aged``) — talk pages are a vector for noise and we
  want the same friction as edits.
* Post creation on an existing thread uses the *plain* ``get_current_user``
  dependency intentionally: once someone has created a thread to discuss
  a part, the conversation should be open to all logged-in members,
  including any who joined within the last 14 days. The age gate is
  about wiki vandalism; talk pages are a comment surface.
* Edit a post — original author OR admin. Closing a thread — original
  poster OR admin. Both checks are inline in the route handler.
"""

from __future__ import annotations

import logging
from datetime import datetime, timezone
from typing import Optional

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy import func, select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import (
    require_terms_accepted,
    require_terms_accepted_aged,
)
from ..config import get_settings
from ..db import get_session
from ..models import Part, PartTalkPost, PartTalkThread
from ..schemas import (
    PartTalkPostCreate,
    PartTalkPostResponse,
    PartTalkPostUpdate,
    PartTalkThreadCreate,
    PartTalkThreadDetail,
    PartTalkThreadSummary,
    PartTalkThreadUpdate,
)

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/parts", tags=["parts-talk"])


def _utcnow_naive() -> datetime:
    return datetime.now(timezone.utc).replace(tzinfo=None)


async def _get_part_by_slug(session: AsyncSession, slug: str) -> Part:
    part = await session.scalar(select(Part).where(Part.slug == slug))
    if part is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Part not found")
    return part


def _is_admin(user: str) -> bool:
    return user in get_settings().admin_usernames_list


def _post_to_response(p: PartTalkPost) -> PartTalkPostResponse:
    return PartTalkPostResponse(
        id=p.id,
        thread_id=p.thread_id,
        author_username=p.author_username,
        content_md=p.content_md,
        created_at=p.created_at,
        edited_at=p.edited_at,
    )


@router.get("/{slug}/talk", response_model=list[PartTalkThreadSummary])
async def list_threads(
    slug: str,
    session: AsyncSession = Depends(get_session),
) -> list[PartTalkThreadSummary]:
    """List discussion threads on this part, newest activity first."""
    part = await _get_part_by_slug(session, slug)
    threads = (
        await session.scalars(
            select(PartTalkThread)
            .where(PartTalkThread.part_id == part.id)
            .order_by(PartTalkThread.updated_at.desc(), PartTalkThread.id.desc())
        )
    ).all()
    if not threads:
        return []

    thread_ids = [t.id for t in threads]
    count_rows = (
        await session.execute(
            select(PartTalkPost.thread_id, func.count(PartTalkPost.id))
            .where(PartTalkPost.thread_id.in_(thread_ids))
            .group_by(PartTalkPost.thread_id)
        )
    ).all()
    counts = {tid: int(c) for tid, c in count_rows}

    # Last poster per thread: cheap-enough nested query (corpus is small).
    last_posters: dict[int, str] = {}
    for tid in thread_ids:
        row = await session.execute(
            select(PartTalkPost.author_username)
            .where(PartTalkPost.thread_id == tid)
            .order_by(PartTalkPost.created_at.desc(), PartTalkPost.id.desc())
            .limit(1)
        )
        first = row.first()
        if first is not None:
            last_posters[tid] = first[0]

    return [
        PartTalkThreadSummary(
            id=t.id,
            title=t.title,
            created_by=t.created_by,
            created_at=t.created_at,
            updated_at=t.updated_at,
            closed=bool(t.closed),
            post_count=counts.get(t.id, 0),
            last_poster=last_posters.get(t.id),
        )
        for t in threads
    ]


@router.post("/{slug}/talk", response_model=PartTalkThreadDetail, status_code=201)
async def create_thread(
    slug: str,
    body: PartTalkThreadCreate,
    user: str = Depends(require_terms_accepted_aged),
    session: AsyncSession = Depends(get_session),
) -> PartTalkThreadDetail:
    """Open a new discussion thread on a part.

    Writes the thread + its first post atomically.
    """
    part = await _get_part_by_slug(session, slug)
    now = _utcnow_naive()
    thread = PartTalkThread(
        part_id=part.id,
        title=body.title.strip(),
        created_by=user,
        created_at=now,
        updated_at=now,
        closed=False,
    )
    session.add(thread)
    await session.flush()

    post = PartTalkPost(
        thread_id=thread.id,
        author_username=user,
        content_md=body.opening_post_content_md,
        created_at=now,
    )
    session.add(post)
    await session.flush()

    await session.commit()
    await session.refresh(thread)
    await session.refresh(post)

    return PartTalkThreadDetail(
        id=thread.id,
        part_id=part.id,
        part_slug=part.slug,
        title=thread.title,
        created_by=thread.created_by,
        created_at=thread.created_at,
        updated_at=thread.updated_at,
        closed=bool(thread.closed),
        closed_by=thread.closed_by,
        closed_at=thread.closed_at,
        posts=[_post_to_response(post)],
    )


async def _load_thread_or_404(
    session: AsyncSession, part: Part, thread_id: int
) -> PartTalkThread:
    thread = await session.get(PartTalkThread, thread_id)
    if thread is None or thread.part_id != part.id:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Thread not found")
    return thread


@router.get(
    "/{slug}/talk/{thread_id}",
    response_model=PartTalkThreadDetail,
)
async def get_thread(
    slug: str,
    thread_id: int,
    session: AsyncSession = Depends(get_session),
) -> PartTalkThreadDetail:
    part = await _get_part_by_slug(session, slug)
    thread = await _load_thread_or_404(session, part, thread_id)
    posts = (
        await session.scalars(
            select(PartTalkPost)
            .where(PartTalkPost.thread_id == thread.id)
            .order_by(PartTalkPost.created_at.asc(), PartTalkPost.id.asc())
        )
    ).all()
    return PartTalkThreadDetail(
        id=thread.id,
        part_id=part.id,
        part_slug=part.slug,
        title=thread.title,
        created_by=thread.created_by,
        created_at=thread.created_at,
        updated_at=thread.updated_at,
        closed=bool(thread.closed),
        closed_by=thread.closed_by,
        closed_at=thread.closed_at,
        posts=[_post_to_response(p) for p in posts],
    )


@router.post(
    "/{slug}/talk/{thread_id}/posts",
    response_model=PartTalkPostResponse,
    status_code=201,
)
async def add_post(
    slug: str,
    thread_id: int,
    body: PartTalkPostCreate,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> PartTalkPostResponse:
    """Append a post to a thread.

    Intentionally uses ``get_current_user`` (no age gate) — the gate is
    only on thread *creation*. Once a thread is open, any logged-in user
    can join the conversation.
    """
    part = await _get_part_by_slug(session, slug)
    thread = await _load_thread_or_404(session, part, thread_id)
    if thread.closed:
        raise HTTPException(
            status.HTTP_400_BAD_REQUEST,
            detail="This thread is closed and not accepting new posts.",
        )
    now = _utcnow_naive()
    post = PartTalkPost(
        thread_id=thread.id,
        author_username=user,
        content_md=body.content_md,
        created_at=now,
    )
    session.add(post)
    thread.updated_at = now
    await session.flush()
    await session.commit()
    await session.refresh(post)
    return _post_to_response(post)


@router.patch(
    "/{slug}/talk/{thread_id}",
    response_model=PartTalkThreadDetail,
)
async def update_thread(
    slug: str,
    thread_id: int,
    body: PartTalkThreadUpdate,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> PartTalkThreadDetail:
    """Toggle the closed state on a thread.

    Original poster OR an admin only.
    """
    part = await _get_part_by_slug(session, slug)
    thread = await _load_thread_or_404(session, part, thread_id)
    if thread.created_by != user and not _is_admin(user):
        raise HTTPException(
            status.HTTP_403_FORBIDDEN,
            detail="Only the original poster or an admin can close a thread.",
        )
    now = _utcnow_naive()
    thread.closed = bool(body.closed)
    if thread.closed:
        thread.closed_by = user
        thread.closed_at = now
    else:
        thread.closed_by = None
        thread.closed_at = None
    thread.updated_at = now
    await session.flush()
    await session.commit()

    posts = (
        await session.scalars(
            select(PartTalkPost)
            .where(PartTalkPost.thread_id == thread.id)
            .order_by(PartTalkPost.created_at.asc(), PartTalkPost.id.asc())
        )
    ).all()
    return PartTalkThreadDetail(
        id=thread.id,
        part_id=part.id,
        part_slug=part.slug,
        title=thread.title,
        created_by=thread.created_by,
        created_at=thread.created_at,
        updated_at=thread.updated_at,
        closed=bool(thread.closed),
        closed_by=thread.closed_by,
        closed_at=thread.closed_at,
        posts=[_post_to_response(p) for p in posts],
    )


@router.patch(
    "/{slug}/talk/{thread_id}/posts/{post_id}",
    response_model=PartTalkPostResponse,
)
async def update_post(
    slug: str,
    thread_id: int,
    post_id: int,
    body: PartTalkPostUpdate,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> PartTalkPostResponse:
    """Edit a post — original author OR admin only.

    Sets ``edited_at`` to now. No edit history is retained for talk
    (unlike the part itself, which has full revision snapshots).
    """
    part = await _get_part_by_slug(session, slug)
    thread = await _load_thread_or_404(session, part, thread_id)
    post = await session.get(PartTalkPost, post_id)
    if post is None or post.thread_id != thread.id:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Post not found")
    if post.author_username != user and not _is_admin(user):
        raise HTTPException(
            status.HTTP_403_FORBIDDEN,
            detail="You can only edit your own posts.",
        )
    now = _utcnow_naive()
    post.content_md = body.content_md
    post.edited_at = now
    thread.updated_at = now
    await session.flush()
    await session.commit()
    await session.refresh(post)
    return _post_to_response(post)
