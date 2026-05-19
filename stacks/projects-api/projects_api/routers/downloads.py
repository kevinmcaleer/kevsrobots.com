"""Download analytics and popularity endpoints.

Two endpoints live here:

* ``GET /api/projects/{project_id}/downloads/stats`` — owner-only project
  analytics (totals, 7d / 30d windows, per-file breakdown, contiguous
  30-day daily series).
* ``GET /api/projects/popular`` — public list of projects sorted by
  download count in the given window (7d / 30d / all-time).

The Download model is populated by the file-download endpoint in
``routers/files.py``; this module only reads from it.
"""

from __future__ import annotations

from datetime import datetime, timedelta, timezone
from typing import Optional

from fastapi import APIRouter, Depends, HTTPException, Query, status
from sqlalchemy import func, select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import get_current_user
from ..db import get_session
from ..models import Download, Project, ProjectFile, ProjectTag
from ..schemas import (
    DailyDownloadCount,
    FileDownloadStats,
    PopularProjectItem,
    ProjectDownloadStats,
)

router = APIRouter(tags=["downloads"])


def _utcnow_naive() -> datetime:
    return datetime.now(timezone.utc).replace(tzinfo=None)


@router.get(
    "/api/projects/{project_id}/downloads/stats",
    response_model=ProjectDownloadStats,
)
async def get_download_stats(
    project_id: int,
    user: str = Depends(get_current_user),
    session: AsyncSession = Depends(get_session),
) -> ProjectDownloadStats:
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Project not found")
    if project.author_username != user:
        raise HTTPException(status.HTTP_403_FORBIDDEN, detail="Not the project owner")

    now = _utcnow_naive()
    seven_days_ago = now - timedelta(days=7)
    thirty_days_ago = now - timedelta(days=30)

    # Totals
    total = (
        await session.execute(
            select(func.count(Download.id)).where(Download.project_id == project_id)
        )
    ).scalar_one() or 0
    last_7d = (
        await session.execute(
            select(func.count(Download.id)).where(
                Download.project_id == project_id,
                Download.downloaded_at >= seven_days_ago,
            )
        )
    ).scalar_one() or 0
    last_30d = (
        await session.execute(
            select(func.count(Download.id)).where(
                Download.project_id == project_id,
                Download.downloaded_at >= thirty_days_ago,
            )
        )
    ).scalar_one() or 0

    # Per-file breakdown
    files = (
        await session.scalars(
            select(ProjectFile).where(ProjectFile.project_id == project_id)
        )
    ).all()

    per_file: list[FileDownloadStats] = []
    for f in files:
        f_total = (
            await session.execute(
                select(func.count(Download.id)).where(Download.file_id == f.id)
            )
        ).scalar_one() or 0
        f_7d = (
            await session.execute(
                select(func.count(Download.id)).where(
                    Download.file_id == f.id,
                    Download.downloaded_at >= seven_days_ago,
                )
            )
        ).scalar_one() or 0
        f_30d = (
            await session.execute(
                select(func.count(Download.id)).where(
                    Download.file_id == f.id,
                    Download.downloaded_at >= thirty_days_ago,
                )
            )
        ).scalar_one() or 0
        per_file.append(
            FileDownloadStats(
                file_id=f.id,
                filename=f.filename,
                total=f_total,
                last_7d=f_7d,
                last_30d=f_30d,
            )
        )
    # Sort highest-first to make the table author-friendly
    per_file.sort(key=lambda x: x.total, reverse=True)

    # Daily series for the last 30 days, zero-filled.
    # Pull raw rows in the window and bucket them in Python — dialect-agnostic
    # and avoids portability issues between SQLite (tests) and Postgres (prod).
    raw_rows = (
        await session.execute(
            select(Download.downloaded_at).where(
                Download.project_id == project_id,
                Download.downloaded_at >= thirty_days_ago,
            )
        )
    ).all()
    bucket: dict[str, int] = {}
    for (ts,) in raw_rows:
        key = ts.date().isoformat()
        bucket[key] = bucket.get(key, 0) + 1

    daily: list[DailyDownloadCount] = []
    # Iterate 30 days inclusive, oldest -> today, so the chart x-axis is
    # contiguous even when there are no downloads on a given day.
    today = now.date()
    for offset in range(29, -1, -1):
        d = today - timedelta(days=offset)
        daily.append(
            DailyDownloadCount(date=d.isoformat(), count=bucket.get(d.isoformat(), 0))
        )

    return ProjectDownloadStats(
        project_id=project_id,
        total=total,
        last_7d=last_7d,
        last_30d=last_30d,
        per_file=per_file,
        daily=daily,
    )


@router.get("/api/projects/popular", response_model=list[PopularProjectItem])
async def popular_projects(
    window: str = Query("30d", pattern="^(7d|30d|all)$"),
    limit: int = Query(20, ge=1, le=100),
    offset: int = Query(0, ge=0),
    session: AsyncSession = Depends(get_session),
) -> list[PopularProjectItem]:
    """Public list of projects ranked by download count in the window.

    Archived and moderation-blocked projects are excluded. Projects with
    zero downloads in the window are excluded from the popular list —
    they're not "popular" by definition.
    """
    now = _utcnow_naive()
    if window == "7d":
        cutoff: Optional[datetime] = now - timedelta(days=7)
    elif window == "30d":
        cutoff = now - timedelta(days=30)
    else:
        cutoff = None  # all-time

    count_expr = func.count(Download.id).label("dl_count")
    query = (
        select(Project, count_expr)
        .join(Download, Download.project_id == Project.id)
        .where(
            Project.status != "archived",
            Project.is_blocked == False,  # noqa: E712
        )
        .group_by(Project.id)
        .order_by(count_expr.desc(), Project.created_at.desc())
        .limit(limit)
        .offset(offset)
    )
    if cutoff is not None:
        query = query.where(Download.downloaded_at >= cutoff)

    rows = (await session.execute(query)).all()

    items: list[PopularProjectItem] = []
    for project, dl_count in rows:
        tags = (
            await session.execute(
                select(ProjectTag.tag).where(ProjectTag.project_id == project.id)
            )
        ).scalars().all()
        items.append(
            PopularProjectItem(
                id=project.id,
                slug=getattr(project, "slug", None),
                title=project.title,
                short_description=project.short_description,
                difficulty=project.difficulty,
                estimated_minutes=project.estimated_minutes,
                status=project.status,
                author_username=project.author_username,
                cover_image=project.cover_image,
                tags=list(tags),
                created_at=project.created_at,
                download_count=int(dl_count or 0),
            )
        )
    return items
