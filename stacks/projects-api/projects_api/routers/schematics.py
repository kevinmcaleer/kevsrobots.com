"""Project schematic endpoints (issue #178, Phase E2).

Each project has at most ONE schematic today — enforced by the unique
constraint on ``project_schematics.project_id`` and the idempotent POST
below. Mirrors the shape of ``routers/instructions.py`` (singular noun
in the URL, idempotent POST, partial PUT, owner + T&Cs gating) so the
two CRUD shapes stay learnable as a pair.

URL surface:

* ``GET    /api/projects/{id}/schematic`` — public, 404 if absent.
* ``POST   /api/projects/{id}/schematic`` — owner + T&Cs.
  Idempotent: if a row already exists, returns it with 200 (NOT 201).
* ``PUT    /api/projects/{id}/schematic`` — owner + T&Cs, partial update.
* ``DELETE /api/projects/{id}/schematic`` — owner + T&Cs.

The schematic graph itself is an opaque JSON document in
``schematic_data`` — the server doesn't parse it. The instruction-step
linkage (``instruction_steps.schematic_id``) is set by the frontend via
the existing instructions PUT route; no new endpoint is needed because
for v1 (one schematic per project) the frontend just writes the
project's single schematic id onto the step.
"""

from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import require_terms_accepted
from ..db import get_session
from ..models import Project, ProjectSchematic
from ..schemas import (
    ProjectSchematicCreate,
    ProjectSchematicResponse,
    ProjectSchematicUpdate,
)

router = APIRouter(
    prefix="/api/projects/{project_id}/schematic", tags=["schematics"]
)


async def _check_owner(session: AsyncSession, project_id: int, user: str) -> Project:
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Project not found")
    if project.author_username != user:
        raise HTTPException(status.HTTP_403_FORBIDDEN, detail="Not the project owner")
    return project


async def _get_schematic(
    session: AsyncSession, project_id: int
) -> ProjectSchematic | None:
    """Load the single schematic for a project (or None)."""
    return (
        await session.scalars(
            select(ProjectSchematic).where(
                ProjectSchematic.project_id == project_id
            )
        )
    ).first()


def _to_response(schematic: ProjectSchematic) -> ProjectSchematicResponse:
    return ProjectSchematicResponse(
        id=schematic.id,
        project_id=schematic.project_id,
        name=schematic.name,
        description=schematic.description,
        schematic_data=schematic.schematic_data,
        created_at=schematic.created_at,
        updated_at=schematic.updated_at,
    )


@router.get("", response_model=ProjectSchematicResponse)
async def get_schematic(
    project_id: int,
    session: AsyncSession = Depends(get_session),
) -> ProjectSchematicResponse:
    """Public read — surfaces user-authored content."""
    schematic = await _get_schematic(session, project_id)
    if schematic is None:
        raise HTTPException(
            status.HTTP_404_NOT_FOUND,
            detail="No schematic for this project",
        )
    return _to_response(schematic)


@router.post("", response_model=ProjectSchematicResponse)
async def create_schematic(
    project_id: int,
    body: ProjectSchematicCreate,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> ProjectSchematicResponse:
    """Owner-only, idempotent.

    If a schematic already exists for this project, the existing row is
    returned with 200 (NOT 201). The frontend wires this up so that
    flipping a step to ``step_type=schematic`` can call POST blindly
    without first checking — the second-and-later POSTs are safe.
    """
    await _check_owner(session, project_id, user)
    existing = await _get_schematic(session, project_id)
    if existing is not None:
        return _to_response(existing)
    schematic = ProjectSchematic(
        project_id=project_id,
        name=body.name,
        description=body.description,
        schematic_data=body.schematic_data,
    )
    session.add(schematic)
    await session.commit()
    await session.refresh(schematic)
    return _to_response(schematic)


@router.put("", response_model=ProjectSchematicResponse)
async def update_schematic(
    project_id: int,
    body: ProjectSchematicUpdate,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> ProjectSchematicResponse:
    """Partial update of name / description / schematic_data.

    Used by the schematic editor's debounced autosave (every ~800ms of
    edit activity). The endpoint accepts arbitrary text in
    ``schematic_data`` — payload size is effectively bounded by HTTP
    body limits (the practical upper bound for v1 schematics is well
    under 2 MB; see ``test_schematics`` for a 1 MB round-trip case)."""
    await _check_owner(session, project_id, user)
    schematic = await _get_schematic(session, project_id)
    if schematic is None:
        raise HTTPException(
            status.HTTP_404_NOT_FOUND,
            detail="No schematic for this project",
        )
    payload = body.model_dump(exclude_unset=True)
    for field, value in payload.items():
        setattr(schematic, field, value)
    await session.commit()
    await session.refresh(schematic)
    return _to_response(schematic)


@router.delete("", status_code=204)
async def delete_schematic(
    project_id: int,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> None:
    await _check_owner(session, project_id, user)
    schematic = await _get_schematic(session, project_id)
    if schematic is None:
        raise HTTPException(
            status.HTTP_404_NOT_FOUND,
            detail="No schematic for this project",
        )
    await session.delete(schematic)
    await session.commit()
