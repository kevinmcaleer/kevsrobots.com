"""Project diagrams: the freeform diagram tool (issue #270).

Many per project. The browser edits a Fabric canvas; it saves the canvas
JSON (the source, never flattened) and its export, an SVG and a PNG. The
project's Markdown shows the SVG as an image; the PNG is for places that
can't show SVG (social cards).

* ``GET    /api/projects/{id}/diagrams``             list (public, drafts hidden)
* ``POST   /api/projects/{id}/diagrams``             owner + T&Cs
* ``GET    /api/projects/{id}/diagrams/{did}``       with its canvas JSON
* ``PUT    /api/projects/{id}/diagrams/{did}``       owner + T&Cs, partial
* ``DELETE /api/projects/{id}/diagrams/{did}``       owner + T&Cs
* ``GET    /api/projects/{id}/diagrams/{did}.svg``   the last export
* ``GET    /api/projects/{id}/diagrams/{did}.png``
"""

from __future__ import annotations

import base64
import binascii
from typing import Optional

from fastapi import APIRouter, Depends, HTTPException, Response, status
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import get_optional_user, require_terms_accepted
from ..config import get_settings
from ..db import get_session
from ..models import Project, ProjectDiagram
from ..schemas import (
    ProjectDiagramCreate,
    ProjectDiagramResponse,
    ProjectDiagramSummary,
    ProjectDiagramUpdate,
)
from ..storage import delete_file, read_file, save_file
from ..svg_safety import SVG_CSP, UnsafeSvg, clean_svg
from ..visibility import NO_STORE, hide_drafts

router = APIRouter(prefix="/api/projects/{project_id}/diagrams", tags=["diagrams"])

PNG_MAGIC = b"\x89PNG\r\n\x1a\n"
MAX_PNG = 15 * 1024 * 1024
EMPTY_CANVAS = {"version": "6.4.0", "objects": []}


async def _owned_project(session: AsyncSession, project_id: int, user: str) -> Project:
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Project not found")
    if project.author_username != user:
        raise HTTPException(status.HTTP_403_FORBIDDEN, detail="Not the project owner")
    return project


async def _diagram(
    session: AsyncSession, project_id: int, diagram_id: int, no_store: bool = False
) -> ProjectDiagram:
    d = await session.get(ProjectDiagram, diagram_id)
    if d is None or d.project_id != project_id:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Diagram not found", headers=NO_STORE if no_store else None)
    return d


def _url(d: ProjectDiagram, ext: str) -> str:
    base = get_settings().public_base_url.rstrip("/")
    return f"{base}/api/projects/{d.project_id}/diagrams/{d.id}.{ext}"


def _summary(d: ProjectDiagram) -> ProjectDiagramSummary:
    return ProjectDiagramSummary(
        id=d.id, project_id=d.project_id, title=d.title,
        svg_url=_url(d, "svg") if d.svg_path else None,
        png_url=_url(d, "png") if d.png_path else None,
        updated_at=d.updated_at,
    )


def _full(d: ProjectDiagram) -> ProjectDiagramResponse:
    return ProjectDiagramResponse(**_summary(d).model_dump(), data=d.data, created_at=d.created_at)


def _check_canvas(data: dict) -> None:
    if not isinstance(data.get("objects", []), list):
        raise HTTPException(status.HTTP_422_UNPROCESSABLE_CONTENT, detail="objects must be a list")


def _png_bytes(data_url: str) -> bytes:
    head, _, payload = data_url.partition(",")
    if head != "data:image/png;base64":
        raise HTTPException(status.HTTP_422_UNPROCESSABLE_CONTENT, detail="png must be a data:image/png;base64 URL")
    try:
        raw = base64.b64decode(payload, validate=True)
    except (binascii.Error, ValueError) as exc:
        raise HTTPException(status.HTTP_422_UNPROCESSABLE_CONTENT, detail="png isn't valid base64") from exc
    if not raw.startswith(PNG_MAGIC):
        raise HTTPException(status.HTTP_422_UNPROCESSABLE_CONTENT, detail="png isn't a PNG")
    if len(raw) > MAX_PNG:
        raise HTTPException(status.HTTP_422_UNPROCESSABLE_CONTENT, detail="The PNG is too large")
    return raw


@router.get("", response_model=list[ProjectDiagramSummary])
async def list_diagrams(
    project_id: int,
    session: AsyncSession = Depends(get_session),
    user: Optional[str] = Depends(get_optional_user),
) -> list[ProjectDiagramSummary]:
    await hide_drafts(session, project_id, user)
    rows = (
        await session.scalars(
            select(ProjectDiagram).where(ProjectDiagram.project_id == project_id).order_by(ProjectDiagram.id)
        )
    ).all()
    return [_summary(d) for d in rows]


@router.post("", response_model=ProjectDiagramResponse, status_code=201)
async def create_diagram(
    project_id: int,
    body: ProjectDiagramCreate,
    session: AsyncSession = Depends(get_session),
    user: str = Depends(require_terms_accepted),
) -> ProjectDiagramResponse:
    await _owned_project(session, project_id, user)
    data = body.data if body.data is not None else dict(EMPTY_CANVAS)
    _check_canvas(data)
    d = ProjectDiagram(project_id=project_id, title=body.title.strip() or "Diagram", data=data)
    session.add(d)
    await session.commit()
    await session.refresh(d)
    return _full(d)


def _file_response(path: Optional[str], media_type: str, draft: bool, extra: Optional[dict] = None) -> Response:
    data = read_file(path) if path else None
    if data is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="This diagram hasn't been exported yet", headers=NO_STORE)
    cache = NO_STORE["Cache-Control"] if draft else "public, max-age=60"
    headers = {"X-Content-Type-Options": "nosniff", "Cache-Control": cache}
    headers.update(extra or {})
    return Response(content=data, media_type=media_type, headers=headers)


# The export routes come before /{diagram_id} so "3.svg" isn't read as an id.
@router.get("/{diagram_id}.svg")
async def get_diagram_svg(
    project_id: int,
    diagram_id: int,
    session: AsyncSession = Depends(get_session),
    user: Optional[str] = Depends(get_optional_user),
) -> Response:
    draft = await hide_drafts(session, project_id, user, no_store=True)
    d = await _diagram(session, project_id, diagram_id, no_store=True)
    return _file_response(d.svg_path, "image/svg+xml", draft, {"Content-Security-Policy": SVG_CSP})


@router.get("/{diagram_id}.png")
async def get_diagram_png(
    project_id: int,
    diagram_id: int,
    session: AsyncSession = Depends(get_session),
    user: Optional[str] = Depends(get_optional_user),
) -> Response:
    draft = await hide_drafts(session, project_id, user, no_store=True)
    d = await _diagram(session, project_id, diagram_id, no_store=True)
    return _file_response(d.png_path, "image/png", draft)


@router.get("/{diagram_id}", response_model=ProjectDiagramResponse)
async def get_diagram(
    project_id: int,
    diagram_id: int,
    session: AsyncSession = Depends(get_session),
    user: Optional[str] = Depends(get_optional_user),
) -> ProjectDiagramResponse:
    await hide_drafts(session, project_id, user)
    return _full(await _diagram(session, project_id, diagram_id))


@router.put("/{diagram_id}", response_model=ProjectDiagramResponse)
async def update_diagram(
    project_id: int,
    diagram_id: int,
    body: ProjectDiagramUpdate,
    session: AsyncSession = Depends(get_session),
    user: str = Depends(require_terms_accepted),
) -> ProjectDiagramResponse:
    await _owned_project(session, project_id, user)
    d = await _diagram(session, project_id, diagram_id)
    if body.data is not None:
        _check_canvas(body.data)
        d.data = body.data
    if body.title is not None:
        d.title = body.title.strip() or d.title
    if body.svg is not None:
        try:
            svg = clean_svg(body.svg)
        except UnsafeSvg as exc:
            raise HTTPException(status.HTTP_422_UNPROCESSABLE_CONTENT, detail=str(exc)) from exc
        path = save_file(svg.encode("utf-8"), f"diagram-{d.id}.svg", project_id, "diagrams")
        if path is None:
            raise HTTPException(status.HTTP_500_INTERNAL_SERVER_ERROR, detail="Failed to save the diagram")
        if d.svg_path and d.svg_path != path:
            delete_file(d.svg_path)
        d.svg_path = path
    if body.png is not None:
        path = save_file(_png_bytes(body.png), f"diagram-{d.id}.png", project_id, "diagrams")
        if path is None:
            raise HTTPException(status.HTTP_500_INTERNAL_SERVER_ERROR, detail="Failed to save the diagram")
        if d.png_path and d.png_path != path:
            delete_file(d.png_path)
        d.png_path = path
    await session.commit()
    await session.refresh(d)
    return _full(d)


@router.delete("/{diagram_id}", status_code=204)
async def delete_diagram(
    project_id: int,
    diagram_id: int,
    session: AsyncSession = Depends(get_session),
    user: str = Depends(require_terms_accepted),
) -> Response:
    await _owned_project(session, project_id, user)
    d = await _diagram(session, project_id, diagram_id)
    for path in (d.svg_path, d.png_path):
        if path:
            delete_file(path)
    await session.delete(d)
    await session.commit()
    return Response(status_code=204)
