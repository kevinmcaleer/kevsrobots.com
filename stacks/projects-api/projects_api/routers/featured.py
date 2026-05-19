"""Featured-project endpoints (issue #115).

Three concerns live here:

* ``POST /api/admin/projects/{id}/feature`` — admin sets the four
  feature columns on a project. Idempotent: re-featuring an already-
  featured project returns 200 with the latest editor note.
* ``DELETE /api/admin/projects/{id}/feature`` — admin clears all four
  columns. Also idempotent.
* ``GET /api/projects/featured`` — public list, newest-feature-first,
  excluding archived / blocked projects.

The per-project feature endpoints intentionally live in this dedicated
router rather than ``moderation.py`` to keep the URL prefix consistent
with ``/api/admin/projects/{id}/...`` and the file boundary aligned with
the issue (#115).
"""

from __future__ import annotations

import logging
from datetime import datetime, timezone
from typing import Optional

from fastapi import APIRouter, Depends, HTTPException, Query, status
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from ..db import get_session
from ..models import Project, ProjectTag
from ..schemas import (
    FeatureProjectRequest,
    FeaturedProjectResponse,
)
from .moderation import get_admin_user

logger = logging.getLogger(__name__)
router = APIRouter(tags=["featured"])


async def _tags_for(session: AsyncSession, project_id: int) -> list[str]:
    rows = (
        await session.execute(
            select(ProjectTag.tag).where(ProjectTag.project_id == project_id)
        )
    ).scalars().all()
    return list(rows)


def _to_response(project: Project, tags: list[str]) -> FeaturedProjectResponse:
    return FeaturedProjectResponse(
        id=project.id,
        slug=getattr(project, "slug", None),
        title=project.title,
        short_description=project.short_description,
        difficulty=project.difficulty,
        estimated_minutes=project.estimated_minutes,
        status=project.status,
        author_username=project.author_username,
        cover_image=project.cover_image,
        tags=tags,
        created_at=project.created_at,
        is_featured=bool(project.is_featured),
        featured_at=project.featured_at,
        featured_by=project.featured_by,
        featured_note=project.featured_note,
    )


@router.post(
    "/api/admin/projects/{project_id}/feature",
    response_model=FeaturedProjectResponse,
)
async def feature_project(
    project_id: int,
    body: FeatureProjectRequest,
    admin: str = Depends(get_admin_user),
    session: AsyncSession = Depends(get_session),
) -> FeaturedProjectResponse:
    """Mark a project as featured. Idempotent — re-featuring updates the
    note + ``featured_by`` and refreshes ``featured_at`` so the carousel
    can surface the most recent editorial pass.
    """
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Project not found")

    # Trim note defensively even though pydantic capped at 200.
    note = (body.note or "").strip() or None
    project.is_featured = True
    project.featured_at = datetime.now(timezone.utc).replace(tzinfo=None)
    project.featured_by = admin
    project.featured_note = note
    await session.commit()
    await session.refresh(project)

    # Structured log so we have an audit trail until a user_activity table
    # lands (issue #111 may add one). Keys are stable so a later cron can
    # back-fill an activity row by scraping logs.
    logger.warning(
        "project_featured project_id=%d admin=%s author=%s note=%r",
        project.id, admin, project.author_username, note,
    )

    tags = await _tags_for(session, project.id)
    return _to_response(project, tags)


@router.delete(
    "/api/admin/projects/{project_id}/feature",
    response_model=FeaturedProjectResponse,
)
async def unfeature_project(
    project_id: int,
    admin: str = Depends(get_admin_user),
    session: AsyncSession = Depends(get_session),
) -> FeaturedProjectResponse:
    """Clear all four feature columns. Idempotent — unfeaturing a project
    that's already unfeatured is a 200 OK no-op.
    """
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Project not found")

    project.is_featured = False
    project.featured_at = None
    project.featured_by = None
    project.featured_note = None
    await session.commit()
    await session.refresh(project)

    logger.warning(
        "project_unfeatured project_id=%d admin=%s author=%s",
        project.id, admin, project.author_username,
    )

    tags = await _tags_for(session, project.id)
    return _to_response(project, tags)


@router.get(
    "/api/projects/featured",
    response_model=list[FeaturedProjectResponse],
)
async def list_featured_projects(
    limit: int = Query(20, ge=1, le=100),
    session: AsyncSession = Depends(get_session),
) -> list[FeaturedProjectResponse]:
    """Public endpoint: featured projects newest-feature-first.

    Excludes archived + blocked projects so unfeaturing a project is the
    only way to remove it from the carousel — admins don't have to also
    remember to mark it as not-featured before archiving. ``featured_at``
    is the sort key (not ``created_at``) because the editorial decision
    is the meaningful timestamp.
    """
    query = (
        select(Project)
        .where(
            Project.is_featured == True,  # noqa: E712
            Project.status != "archived",
            Project.is_blocked == False,  # noqa: E712
        )
        .order_by(Project.featured_at.desc().nullslast(), Project.id.desc())
        .limit(limit)
    )
    projects = list((await session.scalars(query)).all())
    out: list[FeaturedProjectResponse] = []
    for p in projects:
        tags = await _tags_for(session, p.id)
        out.append(_to_response(p, tags))
    return out
