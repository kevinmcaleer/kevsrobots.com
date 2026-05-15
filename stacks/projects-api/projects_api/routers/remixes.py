"""Remix / fork endpoints (issue #108).

Provides:

* ``POST /api/projects/{project_id}/remix`` — create a NEW project owned
  by the calling user, pre-linked as a remix of ``project_id``.
* ``GET  /api/projects/{project_id}/remixes`` — list direct remixes of a
  project.
* ``GET  /api/projects/{project_id}/remix-chain`` — ancestry chain from
  the original ancestor down to this project, with cycle/depth safety.

All write paths refuse to clear or change remix attribution once set
(see ``routers/projects.py::update_project``).
"""

from __future__ import annotations

from typing import Optional

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import get_current_user
from ..db import get_session
from ..models import Project, ProjectTag
from ..schemas import (
    ProjectListItem,
    ProjectResponse,
    ProjectRemixCreate,
)
from .projects import _get_tags, _project_response, _set_tags

router = APIRouter(prefix="/api/projects", tags=["remixes"])

# Defensive cap so we never walk an infinite remix chain. 20 is well above
# any realistic ancestry depth and matches the spec.
MAX_CHAIN_DEPTH = 20


def _list_item_from_project(project: Project, tags: list[str]) -> ProjectListItem:
    return ProjectListItem(
        id=project.id,
        title=project.title,
        short_description=project.short_description,
        difficulty=project.difficulty,
        estimated_minutes=project.estimated_minutes,
        status=project.status,
        author_username=project.author_username,
        cover_image=project.cover_image,
        tags=tags,
        created_at=project.created_at,
        is_remix=getattr(project, "remixed_from_id", None) is not None,
    )


@router.post(
    "/{project_id}/remix",
    response_model=ProjectResponse,
    status_code=201,
)
async def create_remix(
    project_id: int,
    body: ProjectRemixCreate,
    user: str = Depends(get_current_user),
    session: AsyncSession = Depends(get_session),
) -> ProjectResponse:
    """Create a remix of an existing project.

    The new project is owned by the caller and pre-linked to the original
    via ``remixed_from_id``. Fields like description and tags are inherited
    from the original to give the remixer a starting point; they can edit
    everything afterwards via the normal editor flow.
    """
    original = await session.get(Project, project_id)
    if original is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Project not found")
    if getattr(original, "is_blocked", False):
        raise HTTPException(
            status.HTTP_403_FORBIDDEN,
            detail="This project is blocked and cannot be remixed",
        )

    title = (body.title or f"Remix of {original.title}").strip()
    if len(title) < 5:
        title = f"Remix of {original.title}"

    remix = Project(
        title=title[:200],
        short_description=original.short_description,
        content_md=original.content_md,
        difficulty=original.difficulty,
        estimated_minutes=original.estimated_minutes,
        # Don't inherit the original code_repo_url — the remixer's fork
        # will live somewhere different. They can fill it in later.
        code_repo_url=None,
        status="wip",
        author_username=user,
        remixed_from_id=original.id,
        remix_description=body.remix_description.strip(),
    )
    session.add(remix)
    await session.flush()

    # Inherit tags from the original so the remix shows up in the same
    # filters by default. Remixer can edit these later.
    inherited_tags = await _get_tags(session, original.id)
    if inherited_tags:
        await _set_tags(session, remix.id, inherited_tags)

    # TODO(#106): Remixer badge eval hook here — when the badges service
    # lands, fire an event so the caller can level up their Remixer tier.

    await session.commit()
    await session.refresh(remix)
    return await _project_response(session, remix)


@router.get("/{project_id}/remixes", response_model=list[ProjectListItem])
async def list_remixes(
    project_id: int,
    session: AsyncSession = Depends(get_session),
) -> list[ProjectListItem]:
    """List direct (one-hop) remixes of a given project."""
    # 404 the parent itself so callers get a clear signal.
    parent = await session.get(Project, project_id)
    if parent is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Project not found")

    remixes = (
        await session.scalars(
            select(Project)
            .where(Project.remixed_from_id == project_id)
            .where(Project.is_blocked == False)  # noqa: E712
            .order_by(Project.created_at.desc())
        )
    ).all()

    items: list[ProjectListItem] = []
    for r in remixes:
        tags = await _get_tags(session, r.id)
        items.append(_list_item_from_project(r, tags))
    return items


@router.get("/{project_id}/remix-chain", response_model=list[ProjectListItem])
async def remix_chain(
    project_id: int,
    session: AsyncSession = Depends(get_session),
) -> list[ProjectListItem]:
    """Return the ancestry chain ``[original, ..., this]``.

    Walks ``remixed_from_id`` upwards from the given project, then reverses
    so the original ancestor appears first. Caps at :data:`MAX_CHAIN_DEPTH`
    and breaks on cycles by tracking visited ids — both are defensive
    against malformed data, not expected in normal operation.
    """
    current: Optional[Project] = await session.get(Project, project_id)
    if current is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Project not found")

    chain: list[Project] = []
    visited: set[int] = set()
    while current is not None:
        if current.id in visited:
            # Cycle detected — bail out. Should never happen with normal
            # creation flow but defend the endpoint just in case.
            break
        visited.add(current.id)
        chain.append(current)
        if len(chain) >= MAX_CHAIN_DEPTH:
            break
        parent_id = getattr(current, "remixed_from_id", None)
        if parent_id is None:
            break
        current = await session.get(Project, parent_id)

    # We walked child -> parent; reverse so the response is original -> this.
    chain.reverse()

    items: list[ProjectListItem] = []
    for p in chain:
        tags = await _get_tags(session, p.id)
        items.append(_list_item_from_project(p, tags))
    return items
