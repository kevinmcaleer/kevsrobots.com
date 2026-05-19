"""Build-instructions endpoints (issue #178, Phase 0).

Each project has at most ONE instruction set today — enforced by the
unique constraint on ``instructions.project_id`` and the idempotent POST
below. A future phase may relax this to multiple instructions per
project (e.g. assembly vs. wiring) by removing the unique constraint;
the URL scheme reserves only the singular ``/instruction`` path so the
plural form is available for that expansion.

URL surface (mirrors ``routers/videos.py`` for shape + ownership rules):

* ``GET    /api/projects/{id}/instruction``           — public, 404 if absent.
* ``POST   /api/projects/{id}/instruction``           — owner + T&Cs.
  Idempotent: if a row already exists, returns it with 200 (NOT 201).
* ``PUT    /api/projects/{id}/instruction``           — owner + T&Cs.
* ``DELETE /api/projects/{id}/instruction``           — owner + T&Cs.
* ``GET    /api/projects/{id}/instruction/steps``     — public.
* ``POST   /api/projects/{id}/instruction/steps``     — owner + T&Cs.
* ``PUT    /api/projects/{id}/instruction/steps/{id}`` — owner + T&Cs.
* ``DELETE /api/projects/{id}/instruction/steps/{id}`` — owner + T&Cs.

Reorder semantics: a step PUT carrying ``step_number`` triggers a
single-transaction renumber so the resulting sequence is gap-free
(1..len(steps)). A step DELETE intentionally leaves gaps; survivors
keep their stable numbers so any UI handles the frontend has cached
remain valid.
"""

from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy import func, select
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.orm import selectinload

from ..auth import require_terms_accepted
from ..db import get_session
from ..models import Instruction, InstructionStep, Project
from ..schemas import (
    InstructionCreate,
    InstructionResponse,
    InstructionStepCreate,
    InstructionStepResponse,
    InstructionStepUpdate,
    InstructionUpdate,
)

router = APIRouter(prefix="/api/projects/{project_id}/instruction", tags=["instructions"])


async def _check_owner(session: AsyncSession, project_id: int, user: str) -> Project:
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Project not found")
    if project.author_username != user:
        raise HTTPException(status.HTTP_403_FORBIDDEN, detail="Not the project owner")
    return project


async def _get_instruction(
    session: AsyncSession, project_id: int
) -> Instruction | None:
    """Load the single instruction for a project (or None)."""
    return (
        await session.scalars(
            select(Instruction)
            .where(Instruction.project_id == project_id)
            .options(selectinload(Instruction.steps))
        )
    ).first()


async def _load_steps(
    session: AsyncSession, instruction_id: int
) -> list[InstructionStep]:
    return list(
        (
            await session.scalars(
                select(InstructionStep)
                .where(InstructionStep.instruction_id == instruction_id)
                .order_by(InstructionStep.step_number, InstructionStep.id)
            )
        ).all()
    )


def _instruction_to_response(instruction: Instruction) -> InstructionResponse:
    """Build the response dict. ``selectinload`` populates ``steps`` already
    in ``step_number`` order (per the relationship's ``order_by``)."""
    return InstructionResponse(
        id=instruction.id,
        project_id=instruction.project_id,
        title=instruction.title,
        description=instruction.description,
        created_at=instruction.created_at,
        updated_at=instruction.updated_at,
        steps=[
            InstructionStepResponse.model_validate(s, from_attributes=True)
            for s in instruction.steps
        ],
    )


# --- Instruction (singular) ---------------------------------------------


@router.get("", response_model=InstructionResponse)
async def get_instruction(
    project_id: int,
    session: AsyncSession = Depends(get_session),
) -> InstructionResponse:
    """Public read — surfaces user-authored content like the other GETs."""
    instruction = await _get_instruction(session, project_id)
    if instruction is None:
        raise HTTPException(
            status.HTTP_404_NOT_FOUND,
            detail="No instructions for this project",
        )
    return _instruction_to_response(instruction)


@router.post("", response_model=InstructionResponse)
async def create_instruction(
    project_id: int,
    body: InstructionCreate,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> InstructionResponse:
    """Owner-only, idempotent.

    If an instruction already exists for this project, the existing row
    is returned with 200 (NOT 201) — the frontend "Add step" button POSTs
    here without checking first, so we need to be safe under double-click
    and stale-state races.
    """
    await _check_owner(session, project_id, user)
    existing = await _get_instruction(session, project_id)
    if existing is not None:
        # Existing-row return is a 200 by virtue of the default status code
        # on @router.post (FastAPI default is 201 only when the route's
        # `status_code` argument is set; we leave it unset on purpose so
        # the idempotent re-create lands as 200 OK).
        return _instruction_to_response(existing)
    instruction = Instruction(
        project_id=project_id,
        title=body.title,
        description=body.description,
    )
    session.add(instruction)
    await session.commit()
    await session.refresh(instruction)
    # Re-load with steps eager-loaded so the response shape is consistent.
    fresh = await _get_instruction(session, project_id)
    assert fresh is not None  # just inserted, so the row must exist
    return _instruction_to_response(fresh)


@router.put("", response_model=InstructionResponse)
async def update_instruction(
    project_id: int,
    body: InstructionUpdate,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> InstructionResponse:
    """Partial update of title / description. Steps are untouched here —
    callers manage them through the ``/steps`` sub-collection."""
    await _check_owner(session, project_id, user)
    instruction = await _get_instruction(session, project_id)
    if instruction is None:
        raise HTTPException(
            status.HTTP_404_NOT_FOUND,
            detail="No instructions for this project",
        )
    payload = body.model_dump(exclude_unset=True)
    for field, value in payload.items():
        setattr(instruction, field, value)
    await session.commit()
    await session.refresh(instruction)
    fresh = await _get_instruction(session, project_id)
    assert fresh is not None
    return _instruction_to_response(fresh)


@router.delete("", status_code=204)
async def delete_instruction(
    project_id: int,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> None:
    await _check_owner(session, project_id, user)
    instruction = await _get_instruction(session, project_id)
    if instruction is None:
        raise HTTPException(
            status.HTTP_404_NOT_FOUND,
            detail="No instructions for this project",
        )
    # ``cascade="all, delete-orphan"`` on the relationship plus
    # ``ondelete="CASCADE"`` on the FK takes care of steps.
    await session.delete(instruction)
    await session.commit()


# --- Instruction steps (nested collection) ------------------------------


@router.get("/steps", response_model=list[InstructionStepResponse])
async def list_steps(
    project_id: int,
    session: AsyncSession = Depends(get_session),
) -> list[InstructionStepResponse]:
    """Public read. Steps come back in ``step_number`` ASC order; ties
    (which shouldn't happen post-renumber but might during a transient
    DB state) fall back to ``id`` ASC."""
    instruction = await _get_instruction(session, project_id)
    if instruction is None:
        # Empty list rather than 404 — the steps collection is
        # conceptually empty when the parent instruction doesn't exist,
        # and the frontend can call the parent GET if it needs to
        # distinguish "no instruction" from "instruction with no steps".
        return []
    steps = await _load_steps(session, instruction.id)
    return [InstructionStepResponse.model_validate(s, from_attributes=True) for s in steps]


@router.post("/steps", response_model=InstructionStepResponse, status_code=201)
async def add_step(
    project_id: int,
    body: InstructionStepCreate,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> InstructionStepResponse:
    """Owner-only. Server assigns ``step_number = max(existing) + 1``
    (or 1 on the first step)."""
    await _check_owner(session, project_id, user)
    instruction = await _get_instruction(session, project_id)
    if instruction is None:
        raise HTTPException(
            status.HTTP_404_NOT_FOUND,
            detail="No instructions for this project",
        )
    max_step = await session.scalar(
        select(func.max(InstructionStep.step_number)).where(
            InstructionStep.instruction_id == instruction.id
        )
    )
    next_number = 1 if max_step is None else int(max_step) + 1
    step = InstructionStep(
        instruction_id=instruction.id,
        step_number=next_number,
        title=body.title,
        description=body.description,
        canvas_json=body.canvas_json,
    )
    session.add(step)
    await session.commit()
    await session.refresh(step)
    return InstructionStepResponse.model_validate(step, from_attributes=True)


@router.put("/steps/{step_id}", response_model=InstructionStepResponse)
async def update_step(
    project_id: int,
    step_id: int,
    body: InstructionStepUpdate,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> InstructionStepResponse:
    """Partial update.

    If ``step_number`` is in the payload, the router renumbers the entire
    sequence so the target lands at the requested 1-based position and
    survivors fill 1..N gap-free. Otherwise it just applies the field
    update without disturbing ordering. ``step_number=0`` is rejected
    at the Pydantic layer (``ge=1``); values beyond ``len(steps)`` clamp
    to "last position" rather than 422 — that keeps the frontend's
    move-down button trivial.
    """
    await _check_owner(session, project_id, user)
    instruction = await _get_instruction(session, project_id)
    if instruction is None:
        raise HTTPException(
            status.HTTP_404_NOT_FOUND,
            detail="No instructions for this project",
        )
    step = await session.get(InstructionStep, step_id)
    if step is None or step.instruction_id != instruction.id:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Step not found")

    payload = body.model_dump(exclude_unset=True)
    new_step_number = payload.pop("step_number", None)

    for field, value in payload.items():
        setattr(step, field, value)

    if new_step_number is not None:
        # Pydantic's ge=1 enforces this — defence-in-depth.
        if new_step_number < 1:
            raise HTTPException(
                status.HTTP_422_UNPROCESSABLE_ENTITY,
                detail="step_number must be >= 1",
            )
        # Load the current ordering. ``step`` is in this list with its
        # pre-update step_number — we'll move it out, re-insert at the
        # requested position, and renumber 1..N.
        steps = await _load_steps(session, instruction.id)
        # ``_load_steps`` returns a fresh ORM read; locate the target by
        # id rather than identity (which won't match the freshly queried
        # objects under all session configurations).
        survivors = [s for s in steps if s.id != step.id]
        target_index = min(new_step_number, len(survivors) + 1) - 1
        target_index = max(target_index, 0)
        survivors.insert(target_index, step)
        # Renumber: only assign when the new position differs from the
        # current value to keep the UPDATE noise down (and avoid spurious
        # ``updated_at`` bumps on untouched rows).
        for idx, s in enumerate(survivors, start=1):
            if s.step_number != idx:
                s.step_number = idx

    await session.commit()
    await session.refresh(step)
    return InstructionStepResponse.model_validate(step, from_attributes=True)


@router.delete("/steps/{step_id}", status_code=204)
async def delete_step(
    project_id: int,
    step_id: int,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> None:
    """Delete. Survivors keep their existing ``step_number`` — gaps are
    allowed so any UI handles the frontend has cached remain valid.
    The frontend can re-PUT explicit step_numbers if it wants to close
    the gap visually."""
    await _check_owner(session, project_id, user)
    instruction = await _get_instruction(session, project_id)
    if instruction is None:
        raise HTTPException(
            status.HTTP_404_NOT_FOUND,
            detail="No instructions for this project",
        )
    step = await session.get(InstructionStep, step_id)
    if step is None or step.instruction_id != instruction.id:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Step not found")
    await session.delete(step)
    await session.commit()
