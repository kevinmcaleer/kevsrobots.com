"""Project CRUD endpoints."""

from __future__ import annotations

from datetime import datetime, timezone
from typing import Optional

import logging

from fastapi import APIRouter, Depends, HTTPException, Query, Request, Response, status
from sqlalchemy import case, delete, func, select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import get_current_user, get_optional_user, require_terms_accepted
from ..badges import evaluate_user
from ..db import get_session
from ..models import (
    Download,
    Project,
    ProjectBOMItem,
    ProjectSlugHistory,
    ProjectTag,
    ProjectVersion,
    UserFollow,
)
from ..schemas import (
    ProjectCreate,
    ProjectListItem,
    ProjectResponse,
    ProjectUpdate,
    RemixedFromRef,
)
from ..sections import canonical_sections
from ..slugs import slugify_title, unique_slug_for_author
from ..visibility import is_draft, not_draft
from .. import versions as versions_mod
from ..versions import LiveView, View, live_views, resolve_view
from .users import log_activity  # Issue #111: profile activity feed.

logger = logging.getLogger(__name__)


async def resolve_project_id(
    session: AsyncSession,
    owner: str,
    slug: str,
) -> int:
    """Look up a project's numeric id from its ``(owner, slug)`` URL pair.

    Raises :class:`HTTPException` with 404 when no project matches.

    Issue #190: if the (owner, slug) doesn't match a *current* project we
    fall back to ``project_slug_history`` so old shared links resolve
    silently. The fallback is silent here because every machine-facing
    nested-resource route (BOM, images, files, links, journal, makes)
    calls this helper — those callers don't need a redirect, they just
    need the row. Page-facing routes that want to nudge the browser
    toward the canonical URL call :func:`resolve_project_id_with_redirect`
    instead.

    This is the central helper for the issue #152 by-slug routes — every
    nested resource handler (BOM, images, files, links, journal, makes)
    wraps the existing id-based router by calling here first and then
    forwarding to the id-based path. Keeping the lookup in one place
    means a change to the slug-resolution rules (case-folding, alias
    table, history fallback, …) only has to happen here.
    """
    project_id = await session.scalar(
        select(Project.id).where(
            Project.author_username == owner,
            Project.slug == slug,
        )
    )
    if project_id is not None:
        return int(project_id)

    # Issue #190: fall back to the slug-history table. A project may have
    # used this slug in the past and been renamed; we still want the link
    # to resolve. Scope by author via a join on ``projects`` so a slug
    # retired under one user doesn't bleed across to another's namespace.
    historical_id = await session.scalar(
        select(ProjectSlugHistory.project_id)
        .join(Project, Project.id == ProjectSlugHistory.project_id)
        .where(
            Project.author_username == owner,
            ProjectSlugHistory.slug == slug,
        )
        .order_by(ProjectSlugHistory.retired_at.desc())
        .limit(1)
    )
    if historical_id is not None:
        return int(historical_id)

    raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Project not found")


async def resolve_project_id_with_redirect(
    session: AsyncSession,
    owner: str,
    slug: str,
) -> tuple[int, Optional[str]]:
    """Like :func:`resolve_project_id` but signals whether the caller's
    slug is historical.

    Returns ``(project_id, canonical_slug)`` where ``canonical_slug`` is:

    * ``None`` when the supplied slug IS the project's current slug (no
      redirect needed).
    * the project's current slug when the supplied slug matched only via
      the ``project_slug_history`` fallback — the caller should respond
      with a 301 to the canonical ``(owner, canonical_slug)`` URL.

    Used by the public-facing GET routes in ``by_slug.py`` so browsers
    update their URL bar and SEO consolidates onto the latest slug.
    """
    current = await session.execute(
        select(Project.id, Project.slug).where(
            Project.author_username == owner,
            Project.slug == slug,
        )
    )
    row = current.first()
    if row is not None:
        return int(row[0]), None

    historical = await session.execute(
        select(Project.id, Project.slug)
        .join(ProjectSlugHistory, ProjectSlugHistory.project_id == Project.id)
        .where(
            Project.author_username == owner,
            ProjectSlugHistory.slug == slug,
        )
        .order_by(ProjectSlugHistory.retired_at.desc())
        .limit(1)
    )
    hrow = historical.first()
    if hrow is not None:
        return int(hrow[0]), hrow[1]

    raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Project not found")


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

async def _sections(session: AsyncSession, project: Project) -> list[str]:
    """Canonical sections for a list card (issue #259): headings in the
    Markdown, plus the Bill of Materials when the structured BOM has rows.
    A :class:`LiveView` answers from its live version."""
    if isinstance(project, LiveView):
        return canonical_sections(project.content_md, has_bom=project.live_has_bom)
    has_bom = (
        await session.execute(
            select(ProjectBOMItem.id).where(ProjectBOMItem.project_id == project.id).limit(1)
        )
    ).first() is not None
    return canonical_sections(project.content_md, has_bom=has_bom)

router = APIRouter(prefix="/api/projects", tags=["projects"])


async def tags_for(session: AsyncSession, project) -> list[str]:
    """A card's tags: the live version's for a :class:`LiveView`."""
    if isinstance(project, LiveView):
        return list(project.live_tags)
    return await _get_tags(session, project.id)


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


async def _project_response(
    session: AsyncSession, project: Project, view: Optional[View] = None
) -> ProjectResponse:
    """The project as ``view`` shows it (epic #433). Without a view it is
    the author's working copy (the write endpoints)."""
    snap = view.snapshot if view is not None else None
    is_author = view.is_author if view is not None else True
    tags = sorted(snap.get("tags") or []) if snap is not None else await _get_tags(session, project.id)

    def field(name: str, default=None):
        if snap is not None and name in snap:
            return snap[name]
        return getattr(project, name, default)

    # Issue #108: compute remix attribution & counts.
    remixed_from_ref: Optional[RemixedFromRef] = None
    parent_id = getattr(project, "remixed_from_id", None)
    if parent_id is not None:
        parent = await session.get(Project, parent_id)
        if parent is not None:
            shown = await versions_mod.live_view(session, parent)
            remixed_from_ref = RemixedFromRef(
                id=parent.id,
                slug=getattr(parent, "slug", None),
                title=shown.title,
                author_username=parent.author_username,
            )

    remixes_count = (
        await session.scalar(
            select(func.count(Project.id)).where(
                Project.remixed_from_id == project.id, not_draft()
            )
        )
    ) or 0

    live = await versions_mod.live_version(session, project)
    shown_version = view.version if view is not None else None
    updated_at = project.updated_at
    if snap is not None and shown_version is not None:
        updated_at = shown_version.published_at or shown_version.created_at
    live_number = live.number if (is_author and live is not None) else None
    unpublished = None
    if is_author:
        unpublished = live is not None and await versions_mod.has_unpublished_changes(session, project, live)

    return ProjectResponse(
        id=project.id,
        slug=getattr(project, "slug", None),
        title=field("title"),
        short_description=field("short_description"),
        content_md=field("content_md"),
        content_mode=field("content_mode", "markdown") or "markdown",
        difficulty=field("difficulty"),
        estimated_minutes=field("estimated_minutes"),
        code_repo_url=field("code_repo_url"),
        files_view=field("files_view") or "filmstrip",
        status=project.status,
        author_username=project.author_username,
        cover_image=field("cover_image"),
        tags=tags,
        created_at=project.created_at,
        updated_at=updated_at,
        remixed_from=remixed_from_ref,
        remix_description=getattr(project, "remix_description", None),
        remixes_count=int(remixes_count),
        is_remix=remixed_from_ref is not None,
        download_count=await _download_count(session, project.id),
        # Issue #115: surface featured metadata on the detail view.
        is_featured=bool(getattr(project, "is_featured", False)),
        featured_at=getattr(project, "featured_at", None),
        featured_by=getattr(project, "featured_by", None),
        featured_note=getattr(project, "featured_note", None),
        published_at=await versions_mod.first_published_at(session, project.id),
        live_number=live_number,
        has_unpublished_changes=unpublished,
        viewing=view.label if view is not None else "working",
    )


async def list_item(session: AsyncSession, p, **extra) -> ProjectListItem:
    """A card for ``p`` (a project row, or a :class:`LiveView` of one)."""
    return ProjectListItem(
        id=p.id,
        slug=getattr(p, "slug", None),
        title=p.title,
        short_description=p.short_description,
        difficulty=p.difficulty,
        estimated_minutes=p.estimated_minutes,
        status=p.status,
        author_username=p.author_username,
        cover_image=p.cover_image,
        tags=await tags_for(session, p),
        created_at=p.created_at,
        is_remix=getattr(p, "remixed_from_id", None) is not None,
        download_count=await _download_count(session, p.id),
        is_featured=bool(getattr(p, "is_featured", False)),
        featured_note=getattr(p, "featured_note", None),
        sections=await _sections(session, p),
        **extra,
    )


async def regenerate_slug(session: AsyncSession, project: Project, title: str) -> None:
    """Point the project's slug at ``title``, keeping the old one in
    ``project_slug_history`` so it 301s (issues #152/#190)."""
    previous_slug = project.slug
    new_slug = await unique_slug_for_author(
        session,
        author_username=project.author_username,
        base_slug=slugify_title(title),
        exclude_project_id=project.id,
    )
    if new_slug != previous_slug:
        # No-op history when ``previous_slug`` is None (legacy rows that
        # have never had a slug have nothing worth archiving).
        if previous_slug:
            session.add(ProjectSlugHistory(project_id=project.id, slug=previous_slug))
        project.slug = new_slug


@router.get("", response_model=list[ProjectListItem])
async def list_projects(
    difficulty: Optional[str] = Query(None),
    tag: Optional[str] = Query(None),
    search: Optional[str] = Query(None),
    author: Optional[str] = Query(
        None,
        description="Filter to projects authored by this username (issue #140 — public profiles).",
    ),
    boost_followed: bool = Query(
        False,
        description=(
            "Issue #140: when true and the caller is authenticated, projects "
            "authored by users the caller follows are sorted ahead of the "
            "general pool. Ignored for anonymous callers."
        ),
    ),
    limit: int = Query(20, ge=1, le=100),
    offset: int = Query(0, ge=0),
    user: Optional[str] = Depends(get_optional_user),
    session: AsyncSession = Depends(get_session),
) -> list[ProjectListItem]:
    # Epic #433: readers see the live version, so search and the
    # difficulty filter look at the live version's fields (the working
    # copy's for a project that has never been snapshotted).
    live = ProjectVersion

    def live_field(name: str):
        return case(
            (live.id.is_not(None), live.snapshot[name].as_string()),
            else_=getattr(Project, name),
        )

    live_title = live_field("title")
    live_desc = live_field("short_description")
    live_difficulty = live_field("difficulty")
    query = (
        select(Project)
        .outerjoin(live, live.id == Project.published_version_id)
        .where(
            Project.status != "archived",
            Project.is_blocked == False,  # noqa: E712
        )
    )
    # Issue #260: drafts are private. The only listing that includes one
    # is an author's own profile (``author`` == the caller).
    if not (author and user and author == user):
        query = query.where(not_draft())
    if difficulty:
        query = query.where(live_difficulty == difficulty)
    if author:
        query = query.where(Project.author_username == author)
    if search:
        query = query.where(
            live_title.ilike(f"%{search}%") | live_desc.ilike(f"%{search}%")
        )
    query = query.order_by(Project.created_at.desc()).limit(limit).offset(offset)

    projects = await live_views(session, list((await session.scalars(query)).all()))

    if tag:
        wanted_tag = tag.lower()
        kept = []
        for p in projects:
            if wanted_tag in await tags_for(session, p):
                kept.append(p)
        projects = kept

    # Issue #140: bump followed-authors when the caller asked for it.
    # Stable sort so within-bucket order (the SQL "created_at desc") is
    # preserved. Order: completed first → followed-authors → everyone else.
    if boost_followed and user:
        followed = set(
            (
                await session.execute(
                    select(UserFollow.followee_id).where(
                        UserFollow.follower_id == user
                    )
                )
            ).scalars().all()
        )
        if followed:
            def _bucket(p) -> tuple[int, int]:
                completed = 0 if p.status == "completed" else 1
                followed_author = 0 if p.author_username in followed else 1
                return (completed, followed_author)

            projects.sort(key=_bucket)

    return [await list_item(session, p) for p in projects]


@router.get("/{project_id}", response_model=ProjectResponse)
async def get_project(
    project_id: int,
    response: Response,
    version: Optional[str] = Query(
        None,
        description=(
            "Epic #433: ``live``, or (author only) ``working`` or a version "
            "number. Default: the working copy for the author, the live "
            "version for everyone else."
        ),
    ),
    user: Optional[str] = Depends(get_optional_user),
    session: AsyncSession = Depends(get_session),
) -> ProjectResponse:
    view = await resolve_view(session, project_id, user, version)
    if view.project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Project not found")
    response.headers.update(view.headers)
    return await _project_response(session, view.project, view)


@router.post("", response_model=ProjectResponse, status_code=201)
async def create_project(
    body: ProjectCreate,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> ProjectResponse:
    # Issue #152: auto-generate a slug from the title at create time. The
    # slug is unique within the author's portfolio; collisions get a
    # numeric suffix (-2, -3, …).
    base_slug = slugify_title(body.title)
    slug = await unique_slug_for_author(
        session,
        author_username=user,
        base_slug=base_slug,
    )

    project = Project(
        title=body.title,
        slug=slug,
        short_description=body.short_description,
        content_md=body.content_md,
        content_mode=body.content_mode or "markdown",
        difficulty=body.difficulty,
        estimated_minutes=body.estimated_minutes,
        code_repo_url=body.code_repo_url,
        status=body.status,
        author_username=user,
    )
    session.add(project)
    await session.flush()
    if body.tags:
        await _set_tags(session, project.id, body.tags)
    # Epic #433: the editor creates drafts and publishes with POST /publish.
    # A project an older API client creates public (``wip``, the old
    # default) has no version until it's published, and until then reads
    # as before #433 (``versions.resolve_view``).
    # Issue #111: log create as a profile-activity event. project_updated
    # would be wrong (no audience yet) — we use the same row to mean
    # "started a project"; the listener treats it the same as a publish
    # until the status flips to completed. Issue #260: a draft has no
    # audience, so it logs nothing until it's published.
    if not is_draft(project):
        await log_activity(
            session,
            user_id=user,
            kind="project_updated",
            subject_id=project.id,
            subject_title=project.title,
            subject_url=f"/projects/view.html?id={project.id}",
        )
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
    user: str = Depends(require_terms_accepted),
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
    # Issue #434/#437: refuse to overwrite a row that changed since the
    # caller last read it (another tab, or a restore).
    expected = update_data.pop("expected_updated_at", None)
    if expected is not None:
        if expected.tzinfo is not None:
            expected = expected.astimezone(timezone.utc).replace(tzinfo=None)
        if project.updated_at is not None and expected != project.updated_at:
            raise HTTPException(status.HTTP_409_CONFLICT, detail="stale")
    # Issue #152 / #190 / #435: a title change regenerates the slug only
    # while nothing is live. Once a version is live the URL follows the
    # *published* title, so it changes on publish, not on autosave.
    explicit_regen = bool(update_data.pop("regenerate_slug", False))
    new_status = update_data.pop("status", None)
    title_changed = "title" in update_data and update_data["title"] != project.title
    for field_name, value in update_data.items():
        setattr(project, field_name, value)
    project.updated_at = datetime.now(timezone.utc).replace(tzinfo=None)

    if (title_changed or explicit_regen) and project.published_version_id is None:
        await regenerate_slug(session, project, project.title)

    if tags is not None:
        await _set_tags(session, project.id, tags)
    await session.flush()

    # Epic #433: publishing is ``POST /publish`` now. Until every editor
    # uses it, ``status`` still works the old way: ``completed`` (or the
    # retired ``wip``) publishes the working copy, ``draft`` unpublishes,
    # ``archived`` archives (the live version stays live).
    if new_status is not None:
        if new_status in ("completed", "wip"):
            logger.warning(
                "PUT status=%s on project %s: deprecated, use POST /publish (epic #433)",
                new_status, project.id,
            )
            from .versions import publish_and_announce  # local: avoids an import cycle

            await publish_and_announce(session, project, user, note=None, quiet_if_unchanged=True)
        elif new_status == "draft":
            versions_mod.unpublish(project)
        elif new_status == "archived":
            project.status = "archived"

    await session.commit()
    await session.refresh(project)
    return await _project_response(session, project)


@router.delete("/{project_id}", status_code=204)
async def delete_project(
    project_id: int,
    user: str = Depends(require_terms_accepted),
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
    """The author's own projects, as their working copies, with which
    version is live and whether the working copy has changed since."""
    projects = (
        await session.scalars(
            select(Project)
            .where(Project.author_username == user)
            .order_by(Project.updated_at.desc())
        )
    ).all()
    items = []
    for p in projects:
        live = await versions_mod.live_version(session, p)
        items.append(
            await list_item(
                session,
                p,
                live_number=live.number if live is not None else None,
                has_unpublished_changes=(
                    await versions_mod.has_unpublished_changes(session, p, live)
                    if live is not None
                    else False
                ),
            )
        )
    return items
