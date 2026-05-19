"""YouTube video endpoints (issue #171).

A project may carry multiple YouTube videos that the public view page
embeds above the description. Videos are stored as ``project_videos``
rows (id, project_id, youtube_id, title, sort_order, created_at) and
are queryable independently of the generic ``project_links`` table.

CRUD shape mirrors the BOM router:

* ``GET    /api/projects/{id}/videos`` — public read, ordered by
  ``sort_order`` then ``id``.
* ``POST   /api/projects/{id}/videos`` — owner-only. Body
  ``{ url_or_id, title? }``. Server extracts the 11-char id (422 on
  malformed). Falls in at the end of the list (sort_order = max + 1).
* ``PUT    /api/projects/{id}/videos/{video_id}`` — owner-only. Body
  ``{ title?, sort_order? }``. The URL itself is immutable; to swap
  the underlying video the user deletes + adds.
* ``DELETE /api/projects/{id}/videos/{video_id}`` — owner-only.

The youtube id extractor lives in ``projects_api.youtube`` so the
unit tests can hit it directly without spinning up a client.
"""

from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy import func, select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import get_current_user
from ..db import get_session
from ..models import Project, ProjectVideo
from ..schemas import VideoCreate, VideoResponse, VideoUpdate
from ..youtube import extract_youtube_id

router = APIRouter(prefix="/api/projects/{project_id}/videos", tags=["videos"])


async def _check_owner(session: AsyncSession, project_id: int, user: str) -> Project:
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Project not found")
    if project.author_username != user:
        raise HTTPException(status.HTTP_403_FORBIDDEN, detail="Not the project owner")
    return project


@router.get("", response_model=list[VideoResponse])
async def list_videos(
    project_id: int,
    session: AsyncSession = Depends(get_session),
) -> list[VideoResponse]:
    """Public read. Order is ``sort_order`` ASC then ``id`` ASC so
    rows with the same sort key fall back to insertion order."""
    items = (
        await session.scalars(
            select(ProjectVideo)
            .where(ProjectVideo.project_id == project_id)
            .order_by(ProjectVideo.sort_order, ProjectVideo.id)
        )
    ).all()
    return [VideoResponse.model_validate(v, from_attributes=True) for v in items]


@router.post("", response_model=VideoResponse, status_code=201)
async def add_video(
    project_id: int,
    body: VideoCreate,
    user: str = Depends(get_current_user),
    session: AsyncSession = Depends(get_session),
) -> VideoResponse:
    """Owner-only. The extractor raises ``ValueError`` for malformed
    input; we map that to 422 with a clear detail message so the
    frontend can surface "not a YouTube URL" without parsing the
    underlying exception."""
    await _check_owner(session, project_id, user)
    try:
        youtube_id = extract_youtube_id(body.url_or_id)
    except ValueError as exc:
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail=str(exc),
        )
    # Append to the end of the existing ordering. Each project keeps
    # its own sort_order sequence; we don't try to be clever with
    # gaps — the editor's up/down buttons re-PUT explicit values.
    max_sort = await session.scalar(
        select(func.max(ProjectVideo.sort_order)).where(
            ProjectVideo.project_id == project_id
        )
    )
    next_sort = 0 if max_sort is None else int(max_sort) + 1
    video = ProjectVideo(
        project_id=project_id,
        youtube_id=youtube_id,
        title=body.title,
        sort_order=next_sort,
    )
    session.add(video)
    await session.commit()
    await session.refresh(video)
    return VideoResponse.model_validate(video, from_attributes=True)


@router.put("/{video_id}", response_model=VideoResponse)
async def update_video(
    project_id: int,
    video_id: int,
    body: VideoUpdate,
    user: str = Depends(get_current_user),
    session: AsyncSession = Depends(get_session),
) -> VideoResponse:
    """Partial-update title and / or sort_order. The youtube_id is
    intentionally immutable — to swap the video, delete + add."""
    await _check_owner(session, project_id, user)
    video = await session.get(ProjectVideo, video_id)
    if video is None or video.project_id != project_id:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Video not found")
    payload = body.model_dump(exclude_unset=True)
    for field, value in payload.items():
        setattr(video, field, value)
    await session.commit()
    await session.refresh(video)
    return VideoResponse.model_validate(video, from_attributes=True)


@router.delete("/{video_id}", status_code=204)
async def delete_video(
    project_id: int,
    video_id: int,
    user: str = Depends(get_current_user),
    session: AsyncSession = Depends(get_session),
) -> None:
    await _check_owner(session, project_id, user)
    video = await session.get(ProjectVideo, video_id)
    if video is None or video.project_id != project_id:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Video not found")
    await session.delete(video)
    await session.commit()
