"""Project wiring: Snakie's electronics workspace (issue #268).

One wiring per project, like schematics (``routers/schematics.py``), which
stay readable for older projects. The workspace runs in the browser
(Snakie's embeddable bundle); it saves the RobotDefinition it edits and
the SVG it exports, and the project's Markdown shows that SVG as an
image, so the document stays portable and the diagram editable.

* ``GET    /api/projects/{id}/wiring``      public (drafts hidden), 404 if none
* ``PUT    /api/projects/{id}/wiring``      owner + T&Cs; creates or replaces
* ``DELETE /api/projects/{id}/wiring``      owner + T&Cs
* ``GET    /api/projects/{id}/wiring.svg``  the last render, as an image
"""

from __future__ import annotations

from typing import Optional

from fastapi import APIRouter, Depends, HTTPException, Response, status
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import get_optional_user, require_terms_accepted
from ..config import get_settings
from ..db import get_session
from ..models import Project, ProjectWiring
from ..schemas import ProjectWiringResponse, ProjectWiringUpdate
from ..storage import delete_file, read_file, save_file
from ..svg_safety import SVG_CSP, UnsafeSvg, clean_svg
from ..visibility import hide_drafts

router = APIRouter(prefix="/api/projects/{project_id}", tags=["wiring"])


async def _owned_project(session: AsyncSession, project_id: int, user: str) -> Project:
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Project not found")
    if project.author_username != user:
        raise HTTPException(status.HTTP_403_FORBIDDEN, detail="Not the project owner")
    return project


async def _wiring(session: AsyncSession, project_id: int) -> Optional[ProjectWiring]:
    return (
        await session.scalars(select(ProjectWiring).where(ProjectWiring.project_id == project_id))
    ).first()


def _svg_url(project_id: int) -> str:
    base = get_settings().public_base_url.rstrip("/")
    return f"{base}/api/projects/{project_id}/wiring.svg"


def _response(w: ProjectWiring) -> ProjectWiringResponse:
    return ProjectWiringResponse(
        project_id=w.project_id,
        data=w.data,
        svg_url=_svg_url(w.project_id) if w.svg_path else None,
        created_at=w.created_at,
        updated_at=w.updated_at,
    )


def _check_robot(data: dict) -> None:
    for key in ("parts", "connections"):
        if not isinstance(data.get(key, []), list):
            raise HTTPException(status.HTTP_422_UNPROCESSABLE_CONTENT, detail=f"{key} must be a list")


@router.get("/wiring", response_model=ProjectWiringResponse)
async def get_wiring(
    project_id: int,
    session: AsyncSession = Depends(get_session),
    user: Optional[str] = Depends(get_optional_user),
) -> ProjectWiringResponse:
    await hide_drafts(session, project_id, user)
    w = await _wiring(session, project_id)
    if w is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="No wiring for this project")
    return _response(w)


@router.put("/wiring", response_model=ProjectWiringResponse)
async def put_wiring(
    project_id: int,
    body: ProjectWiringUpdate,
    session: AsyncSession = Depends(get_session),
    user: str = Depends(require_terms_accepted),
) -> ProjectWiringResponse:
    project = await _owned_project(session, project_id, user)
    _check_robot(body.data)
    new_path = None
    if body.svg is not None:
        try:
            svg = clean_svg(body.svg)
        except UnsafeSvg as exc:
            raise HTTPException(status.HTTP_422_UNPROCESSABLE_CONTENT, detail=str(exc)) from exc
        new_path = save_file(svg.encode("utf-8"), "wiring.svg", project.id, "wiring")
        if new_path is None:
            raise HTTPException(status.HTTP_500_INTERNAL_SERVER_ERROR, detail="Failed to save the diagram")
    w = await _wiring(session, project_id)
    if w is None:
        w = ProjectWiring(project_id=project_id, data=body.data, svg_path=new_path)
        session.add(w)
    else:
        old = w.svg_path
        w.data = body.data
        if new_path:
            w.svg_path = new_path
            if old and old != new_path:
                delete_file(old)
    await session.commit()
    await session.refresh(w)
    return _response(w)


@router.delete("/wiring", status_code=204)
async def delete_wiring(
    project_id: int,
    session: AsyncSession = Depends(get_session),
    user: str = Depends(require_terms_accepted),
) -> Response:
    await _owned_project(session, project_id, user)
    w = await _wiring(session, project_id)
    if w is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="No wiring for this project")
    if w.svg_path:
        delete_file(w.svg_path)
    await session.delete(w)
    await session.commit()
    return Response(status_code=204)


@router.get("/wiring.svg")
async def get_wiring_svg(
    project_id: int,
    session: AsyncSession = Depends(get_session),
    user: Optional[str] = Depends(get_optional_user),
) -> Response:
    await hide_drafts(session, project_id, user)
    w = await _wiring(session, project_id)
    data = read_file(w.svg_path) if w and w.svg_path else None
    if data is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="No wiring diagram for this project")
    return Response(
        content=data,
        media_type="image/svg+xml",
        headers={
            "Content-Security-Policy": SVG_CSP,
            "X-Content-Type-Options": "nosniff",
            # Short: the editor adds ?v= to the link when it saves, but a
            # bare link should catch up quickly too.
            "Cache-Control": "public, max-age=60",
        },
    )
