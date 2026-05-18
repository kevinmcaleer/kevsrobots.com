"""Project CRUD endpoints."""

from __future__ import annotations

from datetime import datetime, timezone
from typing import Optional

from fastapi import APIRouter, Depends, HTTPException, Query, Request, status
from sqlalchemy import delete, func, select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import get_current_user, get_optional_user
from ..badges import evaluate_user
from ..db import get_session
from ..models import Download, Project, ProjectTag
from ..schemas import (
    ProjectCreate,
    ProjectListItem,
    ProjectResponse,
    ProjectUpdate,
    RemixedFromRef,
)


async def _download_count(session: AsyncSession, project_id: int) -> int:
    """Total downloads across all files for a project.

    v1 implementation: correlated count on every read. This is fine for
    listing pages (n+1 in the dozens, not thousands) and aggregates a
    small indexed table. If listings start showing latency, denormalise
    onto ``projects.download_count`` and update inside ``_log_download``.
    """
    result = await session.execute(
        select(func.count(Download.id)).where(Download.project_id == project_id)
    )
    return int(result.scalar_one() or 0)

router = APIRouter(prefix="/api/projects", tags=["projects"])


async def _get_tags(session: AsyncSession, project_id: int) -> list[str]:
    rows = (
        await session.execute(
            select(ProjectTag.tag).where(ProjectTag.project_id == project_id)
        )
    ).scalars().all()
    return list(rows)


async def _set_tags(session: AsyncSession, project_id: int, tags: list[str]) -> None:
    await session.execute(
        delete(ProjectTag).where(ProjectTag.project_id == project_id)
    )
    for tag in tags:
        session.add(ProjectTag(project_id=project_id, tag=tag.strip().lower()))


async def _project_response(session: AsyncSession, project: Project) -> ProjectResponse:
    tags = await _get_tags(session, project.id)

    # Issue #108: compute remix attribution & counts.
    remixed_from_ref: Optional[RemixedFromRef] = None
    parent_id = getattr(project, "remixed_from_id", None)
    if parent_id is not None:
        parent = await session.get(Project, parent_id)
        if parent is not None:
            remixed_from_ref = RemixedFromRef(
                id=parent.id,
                title=parent.title,
                author_username=parent.author_username,
            )

    remixes_count = (
        await session.scalar(
            select(func.count(Project.id)).where(Project.remixed_from_id == project.id)
        )
    ) or 0

    return ProjectResponse(
        id=project.id,
        title=project.title,
        short_description=project.short_description,
        content_md=project.content_md,
        difficulty=project.difficulty,
        estimated_minutes=project.estimated_minutes,
        code_repo_url=project.code_repo_url,
        status=project.status,
        author_username=project.author_username,
        cover_image=project.cover_image,
        tags=tags,
        created_at=project.created_at,
        updated_at=project.updated_at,
        remixed_from=remixed_from_ref,
        remix_description=getattr(project, "remix_description", None),
        remixes_count=int(remixes_count),
        is_remix=remixed_from_ref is not None,
        download_count=await _download_count(session, project.id),
    )


@router.get("", response_model=list[ProjectListItem])
async def list_projects(
    difficulty: Optional[str] = Query(None),
    tag: Optional[str] = Query(None),
    search: Optional[str] = Query(None),
    limit: int = Query(20, ge=1, le=100),
    offset: int = Query(0, ge=0),
    user: Optional[str] = Depends(get_optional_user),
    session: AsyncSession = Depends(get_session),
) -> list[ProjectListItem]:
    query = select(Project).where(
        Project.status != "archived",
        Project.is_blocked == False,  # noqa: E712
    )
    if difficulty:
        query = query.where(Project.difficulty == difficulty)
    if search:
        query = query.where(
            Project.title.ilike(f"%{search}%")
            | Project.short_description.ilike(f"%{search}%")
        )
    query = query.order_by(Project.created_at.desc()).limit(limit).offset(offset)

    projects = (await session.scalars(query)).all()

    if tag:
        tagged_ids = set(
            (
                await session.execute(
                    select(ProjectTag.project_id).where(ProjectTag.tag == tag.lower())
                )
            ).scalars().all()
        )
        projects = [p for p in projects if p.id in tagged_ids]

    items = []
    for p in projects:
        tags = await _get_tags(session, p.id)
        items.append(
            ProjectListItem(
                id=p.id,
                title=p.title,
                short_description=p.short_description,
                difficulty=p.difficulty,
                estimated_minutes=p.estimated_minutes,
                status=p.status,
                author_username=p.author_username,
                cover_image=p.cover_image,
                tags=tags,
                created_at=p.created_at,
                is_remix=getattr(p, "remixed_from_id", None) is not None,
                download_count=await _download_count(session, p.id),
            )
        )
    return items


@router.get("/{project_id}", response_model=ProjectResponse)
async def get_project(
    project_id: int,
    user: Optional[str] = Depends(get_optional_user),
    session: AsyncSession = Depends(get_session),
) -> ProjectResponse:
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Project not found")
    return await _project_response(session, project)


@router.post("", response_model=ProjectResponse, status_code=201)
async def create_project(
    body: ProjectCreate,
    user: str = Depends(get_current_user),
    session: AsyncSession = Depends(get_session),
) -> ProjectResponse:
    project = Project(
        title=body.title,
        short_description=body.short_description,
        content_md=body.content_md,
        difficulty=body.difficulty,
        estimated_minutes=body.estimated_minutes,
        code_repo_url=body.code_repo_url,
        status="wip",
        author_username=user,
    )
    session.add(project)
    await session.flush()
    if body.tags:
        await _set_tags(session, project.id, body.tags)
    await session.commit()
    await session.refresh(project)
    # Issue #106: evaluate badges synchronously after the project row is
    # safely persisted. evaluate_user is best-effort — wrap in a try so a
    # badge bug can never break project creation.
    newly = []
    try:
        newly = await evaluate_user(session, user)
    except Exception:  # noqa: BLE001 — never crash project create
        pass
    resp = await _project_response(session, project)
    resp.newly_awarded_badges = newly
    return resp


@router.put("/{project_id}", response_model=ProjectResponse)
async def update_project(
    project_id: int,
    body: ProjectUpdate,
    request: Request,
    user: str = Depends(get_current_user),
    session: AsyncSession = Depends(get_session),
) -> ProjectResponse:
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Project not found")
    if project.author_username != user:
        raise HTTPException(status.HTTP_403_FORBIDDEN, detail="Not the project owner")

    # Issue #108: remix attribution is permanent. Even though ProjectUpdate
    # does not expose remixed_from_id / remix_description, defensively
    # inspect the raw request body and reject any explicit attempt to
    # change them. Surface as 400 with a clear message.
    try:
        raw_body = await request.json()
    except Exception:  # pragma: no cover - body may not be JSON
        raw_body = {}
    if isinstance(raw_body, dict):
        for forbidden in ("remixed_from_id", "remix_description"):
            if forbidden in raw_body:
                raise HTTPException(
                    status.HTTP_400_BAD_REQUEST,
                    detail=(
                        "Remix attribution is permanent and cannot be modified. "
                        f"Field '{forbidden}' may not be changed via PUT."
                    ),
                )

    update_data = body.model_dump(exclude_unset=True)
    tags = update_data.pop("tags", None)
    for field, value in update_data.items():
        setattr(project, field, value)
    project.updated_at = datetime.now(timezone.utc).replace(tzinfo=None)

    if tags is not None:
        await _set_tags(session, project.id, tags)

    await session.commit()
    await session.refresh(project)
    return await _project_response(session, project)


@router.delete("/{project_id}", status_code=204)
async def delete_project(
    project_id: int,
    user: str = Depends(get_current_user),
    session: AsyncSession = Depends(get_session),
) -> None:
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Project not found")
    if project.author_username != user:
        raise HTTPException(status.HTTP_403_FORBIDDEN, detail="Not the project owner")
    await session.delete(project)
    await session.commit()


@router.get("/my/list", response_model=list[ProjectListItem])
async def my_projects(
    user: str = Depends(get_current_user),
    session: AsyncSession = Depends(get_session),
) -> list[ProjectListItem]:
    projects = (
        await session.scalars(
            select(Project)
            .where(Project.author_username == user)
            .order_by(Project.updated_at.desc())
        )
    ).all()
    items = []
    for p in projects:
        tags = await _get_tags(session, p.id)
        items.append(
            ProjectListItem(
                id=p.id,
                title=p.title,
                short_description=p.short_description,
                difficulty=p.difficulty,
                estimated_minutes=p.estimated_minutes,
                status=p.status,
                author_username=p.author_username,
                cover_image=p.cover_image,
                tags=tags,
                created_at=p.created_at,
                is_remix=getattr(p, "remixed_from_id", None) is not None,
                download_count=await _download_count(session, p.id),
            )
        )
    return items
