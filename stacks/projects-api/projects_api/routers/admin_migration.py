"""Admin tools for moving existing projects to the new editor (issue #273).

The page at ``/admin/projects/migration/`` uses these routes to:

* list projects whose Bill of Materials is stored as rows but isn't yet a
  table in their Markdown, and append that table (the browser builds it
  with the editor's own ``BomSync.tableFromRows``, so the result is what
  the editor's "Insert BOM table" button writes);
* count the projects still using the old schematic editor;
* read every project's Markdown, so the page can check each one opens and
  saves in the new editor unchanged.

Drafts are included: these routes are admin-only.
"""

from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException, Query, status
from sqlalchemy import func, select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import require_admin
from ..db import get_session
from ..models import Project, ProjectBOMItem, ProjectSchematic, ProjectVideo, ProjectWiring
from ..schemas import (
    MigrationBomCandidate,
    MigrationBomTable,
    MigrationContent,
    MigrationSchematic,
)
from .bom import list_bom

router = APIRouter(prefix="/api/admin/migration", tags=["admin-migration"])

# Marks a table written by the bulk action, so it can be found later.
GENERATED_MARKER = "<!-- Bill of Materials table generated from the stored BOM rows (#273) -->"


@router.get("/bom", response_model=list[MigrationBomCandidate])
async def list_bom_candidates(
    _admin: str = Depends(require_admin),
    session: AsyncSession = Depends(get_session),
) -> list[MigrationBomCandidate]:
    """Every project with stored BOM rows, with its Markdown. The page
    keeps the ones whose text has no Bill of Materials table, using the
    same parser as the editor."""
    ids = (
        await session.scalars(
            select(ProjectBOMItem.project_id).distinct().order_by(ProjectBOMItem.project_id)
        )
    ).all()
    out = []
    for project_id in ids:
        project = await session.get(Project, project_id)
        if project is None:
            continue
        # Read as the owner, so a draft's rows come back too.
        rows = await list_bom(project_id, session, user=project.author_username)
        out.append(
            MigrationBomCandidate(
                project_id=project.id,
                title=project.title,
                author_username=project.author_username,
                status=project.status,
                content_mode=project.content_mode,
                content_md=project.content_md or "",
                rows=rows,
            )
        )
    return out


@router.put("/projects/{project_id}/bom-table", status_code=204)
async def append_bom_table(
    project_id: int,
    body: MigrationBomTable,
    _admin: str = Depends(require_admin),
    session: AsyncSession = Depends(get_session),
) -> None:
    """Append a generated Bill of Materials table to a project's text.

    Only ever appends: the new text must be the text the page read plus
    the marker and a table, and the stored text must not have changed
    since (409 if the owner saved in between). ``updated_at`` is left
    alone, so the project doesn't jump to the top of "recently updated".
    """
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Project not found")
    if (project.content_md or "") != body.expected_content_md:
        raise HTTPException(status.HTTP_409_CONFLICT, detail="The project's text changed; reload and try again")
    added = body.content_md[len(body.expected_content_md):]
    if not body.content_md.startswith(body.expected_content_md) or not added.lstrip().startswith(GENERATED_MARKER):
        raise HTTPException(
            status.HTTP_400_BAD_REQUEST,
            detail="Only a generated Bill of Materials table may be appended",
        )
    project.content_md = body.content_md
    await session.commit()


@router.get("/schematics", response_model=list[MigrationSchematic])
async def list_legacy_schematics(
    _admin: str = Depends(require_admin),
    session: AsyncSession = Depends(get_session),
) -> list[MigrationSchematic]:
    """Projects that still have a schematic from the old schematic editor
    and haven't redrawn it in the electronics workspace (#268)."""
    rows = (
        await session.execute(
            select(ProjectSchematic, Project)
            .join(Project, Project.id == ProjectSchematic.project_id)
            .outerjoin(ProjectWiring, ProjectWiring.project_id == Project.id)
            .where(ProjectWiring.id.is_(None))
            .order_by(ProjectSchematic.updated_at.desc())
        )
    ).all()
    return [
        MigrationSchematic(
            project_id=p.id,
            title=p.title,
            author_username=p.author_username,
            content_mode=p.content_mode,
            updated_at=s.updated_at,
        )
        for s, p in rows
    ]


@router.get("/contents", response_model=list[MigrationContent])
async def list_contents(
    offset: int = Query(0, ge=0),
    limit: int = Query(50, ge=1, le=200),
    _admin: str = Depends(require_admin),
    session: AsyncSession = Depends(get_session),
) -> list[MigrationContent]:
    """Every project's Markdown, a page at a time, for the round-trip check.

    With the two things the editor writes into the text on first open:
    the code repository link and the videos (each as a section).
    """
    projects = (
        await session.scalars(select(Project).order_by(Project.id).offset(offset).limit(limit))
    ).all()
    videos = dict(
        (
            await session.execute(
                select(ProjectVideo.project_id, func.count(ProjectVideo.id))
                .where(ProjectVideo.project_id.in_([p.id for p in projects]))
                .group_by(ProjectVideo.project_id)
            )
        ).all()
    )
    return [
        MigrationContent(
            project_id=p.id,
            title=p.title,
            content_mode=p.content_mode,
            content_md=p.content_md or "",
            code_repo_url=p.code_repo_url,
            video_count=videos.get(p.id, 0),
        )
        for p in projects
    ]


@router.get("/contents/count")
async def count_contents(
    _admin: str = Depends(require_admin),
    session: AsyncSession = Depends(get_session),
) -> dict:
    return {"count": await session.scalar(select(func.count(Project.id)))}
