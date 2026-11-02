"""Project versions: publish, list, view, restore, delete (epic #433).

* ``POST   /api/projects/{id}/publish``                  the working copy goes live
* ``POST   /api/projects/{id}/unpublish``                nothing live; versions kept
* ``POST   /api/projects/{id}/archive`` / ``unarchive``  off the listings / back
* ``GET    /api/projects/{id}/versions``                 the drop-down's rows
* ``GET    /api/projects/{id}/versions/{n}``             one version's snapshot
* ``POST   /api/projects/{id}/versions/{n}/restore``     copy it into the working copy
* ``DELETE /api/projects/{id}/versions/{n}``             not the live one

All of them are the project owner's alone. See ``projects_api/versions.py``
for the model.
"""

from __future__ import annotations

from typing import Optional

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy import func, select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import get_current_user, require_terms_accepted
from ..badges import evaluate_user
from ..db import get_session
from ..models import Project, ProjectVersion
from ..schemas import (
    ProjectResponse,
    PublishRequest,
    PublishResponse,
    RestoreResponse,
    VersionDetail,
    VersionList,
    VersionSummary,
)
from .. import versions as v
from .projects import _project_response, regenerate_slug
from .users import log_activity

router = APIRouter(prefix="/api/projects", tags=["versions"])


async def _owned(session: AsyncSession, project_id: int, user: str) -> Project:
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Project not found")
    if project.author_username != user:
        raise HTTPException(status.HTTP_403_FORBIDDEN, detail="Not the project owner")
    return project


async def _version_or_404(session: AsyncSession, project: Project, number: int) -> ProjectVersion:
    found = await v.get_version(session, project.id, number)
    if found is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Version not found")
    return found


def _summary(version: ProjectVersion, live_id: Optional[int]) -> VersionSummary:
    return VersionSummary(
        number=version.number,
        kind=version.kind,
        title=(version.snapshot or {}).get("title"),
        created_at=version.created_at,
        published_at=version.published_at,
        is_live=version.id == live_id,
        restored_from_number=version.restored_from_number,
        note=version.note,
    )


async def publish_and_announce(
    session: AsyncSession,
    project: Project,
    user: str,
    *,
    note: Optional[str] = None,
    quiet_if_unchanged: bool = False,
) -> Optional[ProjectVersion]:
    """Publish, then do what going live does: the slug follows the
    published title, and the profile feed hears about it once per version
    (``project_published`` for the first, ``project_updated`` after).
    Caller commits (and evaluates badges after). With
    ``quiet_if_unchanged`` an unchanged working copy is not an error: the
    project is simply made sure to be public (the old ``status`` route)."""
    try:
        version = await v.publish(session, project, note=note)
    except v.NoChanges:
        if not quiet_if_unchanged:
            raise HTTPException(status.HTTP_409_CONFLICT, detail="No changes to publish")
        if project.status != "archived":
            project.status = "completed"
        return None
    await regenerate_slug(session, project, (version.snapshot or {}).get("title") or project.title)
    published_before = await session.scalar(
        select(func.count(ProjectVersion.id)).where(
            ProjectVersion.project_id == project.id,
            ProjectVersion.kind == v.PUBLISHED,
            ProjectVersion.id != version.id,
        )
    )
    await log_activity(
        session,
        user_id=user,
        kind="project_updated" if published_before else "project_published",
        subject_id=project.id,
        subject_title=(version.snapshot or {}).get("title") or project.title,
        subject_url=f"/projects/view.html?id={project.id}",
    )
    return version


@router.post("/{project_id}/publish", response_model=PublishResponse)
async def publish_project(
    project_id: int,
    body: Optional[PublishRequest] = None,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> PublishResponse:
    """The working copy becomes the next version and goes live. 409 when
    it is the live version already (nothing to publish)."""
    project = await _owned(session, project_id, user)
    if project.is_blocked:
        raise HTTPException(status.HTTP_403_FORBIDDEN, detail="This project is blocked")
    version = await publish_and_announce(session, project, user, note=body.note if body else None)
    await session.commit()
    await session.refresh(project)
    newly = []
    try:
        newly = await evaluate_user(session, user)
    except Exception:  # noqa: BLE001 — never fail a publish over a badge
        pass
    resp = await _project_response(session, project)
    resp.newly_awarded_badges = newly
    return PublishResponse(version=_summary(version, project.published_version_id), project=resp)


@router.post("/{project_id}/unpublish", response_model=ProjectResponse)
async def unpublish_project(
    project_id: int,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> ProjectResponse:
    """Take the project off the site: nothing is live and it's a draft
    again. Every version is kept, so it can be published again."""
    project = await _owned(session, project_id, user)
    v.unpublish(project)
    await session.commit()
    await session.refresh(project)
    return await _project_response(session, project)


@router.post("/{project_id}/archive", response_model=ProjectResponse)
async def archive_project(
    project_id: int,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> ProjectResponse:
    """Off the listings, still reachable by link; the live version stays."""
    project = await _owned(session, project_id, user)
    project.status = "archived"
    await session.commit()
    await session.refresh(project)
    return await _project_response(session, project)


@router.post("/{project_id}/unarchive", response_model=ProjectResponse)
async def unarchive_project(
    project_id: int,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> ProjectResponse:
    """Back on the listings when a version is live, otherwise a draft."""
    project = await _owned(session, project_id, user)
    if project.status == "archived":
        project.status = "completed" if project.published_version_id else "draft"
    await session.commit()
    await session.refresh(project)
    return await _project_response(session, project)


@router.get("/{project_id}/versions", response_model=VersionList)
async def list_project_versions(
    project_id: int,
    user: str = Depends(get_current_user),
    session: AsyncSession = Depends(get_session),
) -> VersionList:
    project = await _owned(session, project_id, user)
    rows = await v.list_versions(session, project.id)
    live = next((r for r in rows if r.id == project.published_version_id), None)
    return VersionList(
        live_number=live.number if live is not None else None,
        has_unpublished_changes=(
            await v.has_unpublished_changes(session, project, live) if live is not None else False
        ),
        versions=[_summary(r, project.published_version_id) for r in rows],
    )


@router.get("/{project_id}/versions/{number}", response_model=VersionDetail)
async def get_project_version(
    project_id: int,
    number: int,
    user: str = Depends(get_current_user),
    session: AsyncSession = Depends(get_session),
) -> VersionDetail:
    project = await _owned(session, project_id, user)
    version = await _version_or_404(session, project, number)
    return VersionDetail(
        **_summary(version, project.published_version_id).model_dump(),
        snapshot=version.snapshot or {},
    )


@router.post("/{project_id}/versions/{number}/restore", response_model=RestoreResponse)
async def restore_project_version(
    project_id: int,
    number: int,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> RestoreResponse:
    """Issue #437: version ``number`` becomes the working copy. The
    working copy is saved first as an unpublished version, unless it is
    the same as the newest version already (so restoring twice in a row
    doesn't fill the list with copies). What's live doesn't change."""
    project = await _owned(session, project_id, user)
    target = await _version_or_404(session, project, number)
    current = await v.snapshot_project(session, project)
    # Already kept as some version (often the live one, or the copy the
    # last restore saved): nothing would be lost, so don't add another.
    already_kept = any(
        v.same_content(current, other.snapshot) for other in await v.list_versions(session, project.id)
    )
    saved_as = None
    if not already_kept:
        saved = await v.create_version(
            session,
            project,
            kind=v.UNPUBLISHED,
            snapshot=current,
            note=f"Saved before restoring v{target.number}",
        )
        saved_as = saved.number
    await v.apply_snapshot(session, project, target.snapshot or {})
    await session.commit()
    await session.refresh(project)
    return RestoreResponse(
        project=await _project_response(session, project),
        restored=target.number,
        saved_as=saved_as,
    )


@router.delete("/{project_id}/versions/{number}", status_code=204)
async def delete_project_version(
    project_id: int,
    number: int,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> None:
    """Delete a version that isn't live (issue #440). Its number is never
    given out again."""
    project = await _owned(session, project_id, user)
    version = await _version_or_404(session, project, number)
    if version.id == project.published_version_id:
        raise HTTPException(status.HTTP_409_CONFLICT, detail="The live version can't be deleted")
    await v.delete_version(session, project, version)
    await session.commit()
