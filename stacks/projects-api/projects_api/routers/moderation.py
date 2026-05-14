"""Moderation endpoints — reporting and admin tools."""

from __future__ import annotations

from datetime import datetime, timezone
from typing import Optional

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import get_current_user
from ..config import get_settings
from ..db import get_session
from ..models import Project, ProjectReport
from ..schemas import (
    BlockedProjectResponse,
    ModerationNoteUpdate,
    ReportCreate,
    ReportResponse,
    ReportUpdate,
)

router = APIRouter(tags=["moderation"])


def get_admin_user(
    user: str = Depends(get_current_user),
) -> str:
    """Dependency that ensures the current user is an admin."""
    settings = get_settings()
    if user not in settings.admin_usernames_list:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Admin access required",
        )
    return user


# ---- User-facing: report a project ----


@router.post(
    "/api/projects/{project_id}/report",
    response_model=ReportResponse,
    status_code=201,
)
async def report_project(
    project_id: int,
    body: ReportCreate,
    user: str = Depends(get_current_user),
    session: AsyncSession = Depends(get_session),
) -> ReportResponse:
    # Verify project exists
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Project not found")

    # Rate limit: max 1 report per user per project
    existing = (
        await session.execute(
            select(ProjectReport).where(
                ProjectReport.project_id == project_id,
                ProjectReport.reporter_username == user,
            )
        )
    ).scalar_one_or_none()
    if existing is not None:
        raise HTTPException(
            status.HTTP_409_CONFLICT,
            detail="You have already reported this project",
        )

    report = ProjectReport(
        project_id=project_id,
        reporter_username=user,
        reason=body.reason,
        status="pending",
    )
    session.add(report)
    await session.commit()
    await session.refresh(report)
    return ReportResponse(
        id=report.id,
        project_id=report.project_id,
        reporter_username=report.reporter_username,
        reason=report.reason,
        status=report.status,
        created_at=report.created_at,
        reviewed_at=report.reviewed_at,
        reviewed_by=report.reviewed_by,
    )


# ---- Admin endpoints ----


@router.get("/api/admin/reports", response_model=list[ReportResponse])
async def list_reports(
    report_status: Optional[str] = None,
    admin: str = Depends(get_admin_user),
    session: AsyncSession = Depends(get_session),
) -> list[ReportResponse]:
    query = select(ProjectReport)
    if report_status:
        query = query.where(ProjectReport.status == report_status)
    else:
        query = query.where(ProjectReport.status == "pending")
    query = query.order_by(ProjectReport.created_at.desc())

    reports = (await session.scalars(query)).all()
    return [
        ReportResponse(
            id=r.id,
            project_id=r.project_id,
            reporter_username=r.reporter_username,
            reason=r.reason,
            status=r.status,
            created_at=r.created_at,
            reviewed_at=r.reviewed_at,
            reviewed_by=r.reviewed_by,
        )
        for r in reports
    ]


@router.put("/api/admin/reports/{report_id}", response_model=ReportResponse)
async def update_report(
    report_id: int,
    body: ReportUpdate,
    admin: str = Depends(get_admin_user),
    session: AsyncSession = Depends(get_session),
) -> ReportResponse:
    report = await session.get(ProjectReport, report_id)
    if report is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Report not found")

    report.status = body.status
    report.reviewed_at = datetime.now(timezone.utc).replace(tzinfo=None)
    report.reviewed_by = admin
    await session.commit()
    await session.refresh(report)
    return ReportResponse(
        id=report.id,
        project_id=report.project_id,
        reporter_username=report.reporter_username,
        reason=report.reason,
        status=report.status,
        created_at=report.created_at,
        reviewed_at=report.reviewed_at,
        reviewed_by=report.reviewed_by,
    )


@router.put("/api/admin/projects/{project_id}/block")
async def block_project(
    project_id: int,
    admin: str = Depends(get_admin_user),
    session: AsyncSession = Depends(get_session),
) -> dict:
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Project not found")
    project.is_blocked = True
    await session.commit()
    return {"detail": "Project blocked"}


@router.put("/api/admin/projects/{project_id}/unblock")
async def unblock_project(
    project_id: int,
    admin: str = Depends(get_admin_user),
    session: AsyncSession = Depends(get_session),
) -> dict:
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Project not found")
    project.is_blocked = False
    await session.commit()
    return {"detail": "Project unblocked"}


@router.put("/api/admin/projects/{project_id}/moderate")
async def set_moderation_note(
    project_id: int,
    body: ModerationNoteUpdate,
    admin: str = Depends(get_admin_user),
    session: AsyncSession = Depends(get_session),
) -> dict:
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Project not found")
    project.moderation_note = body.moderation_note
    await session.commit()
    return {"detail": "Moderation note updated"}


@router.get("/api/admin/projects/blocked", response_model=list[BlockedProjectResponse])
async def list_blocked_projects(
    admin: str = Depends(get_admin_user),
    session: AsyncSession = Depends(get_session),
) -> list[BlockedProjectResponse]:
    projects = (
        await session.scalars(
            select(Project)
            .where(Project.is_blocked == True)  # noqa: E712
            .order_by(Project.created_at.desc())
        )
    ).all()
    return [
        BlockedProjectResponse(
            id=p.id,
            title=p.title,
            author_username=p.author_username,
            is_blocked=p.is_blocked,
            moderation_note=p.moderation_note,
            created_at=p.created_at,
        )
        for p in projects
    ]
