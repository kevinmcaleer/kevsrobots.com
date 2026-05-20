"""Project symbols (Symbol Designer) CRUD endpoints.

Per-project user-designed schematic symbols that augment the hard-coded
stub library in the schematic editor (E2). Mirrors ``routers/schematics``
in surface conventions (owner + T&Cs gating on writes, public reads,
partial-update PUT) but the noun is *plural* because a project can hold
many symbols — one per electronic component the user wants to design.

URL surface:

* ``GET    /api/projects/{id}/symbols``         — public, list.
* ``POST   /api/projects/{id}/symbols``         — owner + T&Cs, returns 201.
* ``GET    /api/projects/{id}/symbols/{sid}``   — public, single.
* ``PUT    /api/projects/{id}/symbols/{sid}``   — owner + T&Cs, partial.
* ``DELETE /api/projects/{id}/symbols/{sid}``   — owner + T&Cs.
* ``POST   /api/projects/{id}/symbols/{sid}/duplicate``
                                                — owner + T&Cs, clones the row.

The symbol body + pin layout itself is an opaque JSON document in
``symbol_data`` — the server doesn't parse it. ``bom_item_id`` is
validated (404 if it doesn't exist or belongs to a different project) so
a stale link can't slip through; the same rule applies on PUT.
"""

from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import require_terms_accepted
from ..db import get_session
from ..models import Project, ProjectBOMItem, ProjectSymbol
from ..schemas import (
    ProjectSymbolCreate,
    ProjectSymbolResponse,
    ProjectSymbolUpdate,
)

router = APIRouter(
    prefix="/api/projects/{project_id}/symbols", tags=["symbols"]
)


async def _check_owner(session: AsyncSession, project_id: int, user: str) -> Project:
    project = await session.get(Project, project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Project not found")
    if project.author_username != user:
        raise HTTPException(status.HTTP_403_FORBIDDEN, detail="Not the project owner")
    return project


async def _get_symbol(
    session: AsyncSession, project_id: int, symbol_id: int
) -> ProjectSymbol | None:
    """Load a symbol, scoped to the URL's project for safety.

    Returning None on either a missing row or a project-id mismatch
    means the caller can issue a single 404 without leaking which case
    fired."""
    sym = await session.get(ProjectSymbol, symbol_id)
    if sym is None or sym.project_id != project_id:
        return None
    return sym


async def _validate_bom_item(
    session: AsyncSession, project_id: int, bom_item_id: int | None
) -> None:
    """Reject ``bom_item_id`` values that don't correspond to a BOM row
    in this project. Pass ``None`` through unchanged (it means "no link").
    Raises 422 on a bad id — chosen over 400 because the value is a
    request-body field and 422 is FastAPI's convention for body
    validation failures."""
    if bom_item_id is None:
        return
    item = await session.get(ProjectBOMItem, bom_item_id)
    if item is None or item.project_id != project_id:
        raise HTTPException(
            status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail="bom_item_id does not correspond to a BOM row on this project",
        )


def _to_response(symbol: ProjectSymbol) -> ProjectSymbolResponse:
    return ProjectSymbolResponse(
        id=symbol.id,
        project_id=symbol.project_id,
        name=symbol.name,
        ref_des_prefix=symbol.ref_des_prefix or "U",
        description=symbol.description,
        bom_item_id=symbol.bom_item_id,
        symbol_data=symbol.symbol_data,
        created_at=symbol.created_at,
        updated_at=symbol.updated_at,
    )


@router.get("", response_model=list[ProjectSymbolResponse])
async def list_symbols(
    project_id: int,
    session: AsyncSession = Depends(get_session),
) -> list[ProjectSymbolResponse]:
    """Public read — surfaces user-authored content for project viewers
    (a remixer can see the library that ships with the project before
    deciding to fork)."""
    rows = (
        await session.scalars(
            select(ProjectSymbol)
            .where(ProjectSymbol.project_id == project_id)
            .order_by(ProjectSymbol.id.asc())
        )
    ).all()
    return [_to_response(r) for r in rows]


@router.post("", response_model=ProjectSymbolResponse, status_code=201)
async def create_symbol(
    project_id: int,
    body: ProjectSymbolCreate,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> ProjectSymbolResponse:
    await _check_owner(session, project_id, user)
    await _validate_bom_item(session, project_id, body.bom_item_id)
    symbol = ProjectSymbol(
        project_id=project_id,
        name=body.name,
        ref_des_prefix=body.ref_des_prefix or "U",
        description=body.description,
        bom_item_id=body.bom_item_id,
        symbol_data=body.symbol_data,
    )
    session.add(symbol)
    await session.commit()
    await session.refresh(symbol)
    return _to_response(symbol)


@router.get("/{symbol_id}", response_model=ProjectSymbolResponse)
async def get_symbol(
    project_id: int,
    symbol_id: int,
    session: AsyncSession = Depends(get_session),
) -> ProjectSymbolResponse:
    symbol = await _get_symbol(session, project_id, symbol_id)
    if symbol is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Symbol not found")
    return _to_response(symbol)


@router.put("/{symbol_id}", response_model=ProjectSymbolResponse)
async def update_symbol(
    project_id: int,
    symbol_id: int,
    body: ProjectSymbolUpdate,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> ProjectSymbolResponse:
    """Partial update — fields omitted from the payload are untouched.

    Used by the symbol designer's debounced autosave (every ~500ms of
    edit activity). Body shape is opaque on the server; see the
    docstring of ``models.ProjectSymbol`` for the schema."""
    await _check_owner(session, project_id, user)
    symbol = await _get_symbol(session, project_id, symbol_id)
    if symbol is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Symbol not found")
    payload = body.model_dump(exclude_unset=True)
    if "bom_item_id" in payload:
        await _validate_bom_item(session, project_id, payload["bom_item_id"])
    for field, value in payload.items():
        setattr(symbol, field, value)
    await session.commit()
    await session.refresh(symbol)
    return _to_response(symbol)


@router.delete("/{symbol_id}", status_code=204)
async def delete_symbol(
    project_id: int,
    symbol_id: int,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> None:
    await _check_owner(session, project_id, user)
    symbol = await _get_symbol(session, project_id, symbol_id)
    if symbol is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Symbol not found")
    await session.delete(symbol)
    await session.commit()


@router.post(
    "/{symbol_id}/duplicate",
    response_model=ProjectSymbolResponse,
    status_code=201,
)
async def duplicate_symbol(
    project_id: int,
    symbol_id: int,
    user: str = Depends(require_terms_accepted),
    session: AsyncSession = Depends(get_session),
) -> ProjectSymbolResponse:
    """Convenience clone — fields copied 1:1 with ``(copy)`` appended to
    the name. The new row gets a fresh id + created_at; the BOM link is
    copied through (a duplicate likely wants the same association — the
    user can clear it from the designer if not)."""
    await _check_owner(session, project_id, user)
    source = await _get_symbol(session, project_id, symbol_id)
    if source is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Symbol not found")
    copy = ProjectSymbol(
        project_id=project_id,
        name=(source.name or "Untitled") + " (copy)",
        ref_des_prefix=source.ref_des_prefix or "U",
        description=source.description,
        bom_item_id=source.bom_item_id,
        symbol_data=source.symbol_data,
    )
    session.add(copy)
    await session.commit()
    await session.refresh(copy)
    return _to_response(copy)
