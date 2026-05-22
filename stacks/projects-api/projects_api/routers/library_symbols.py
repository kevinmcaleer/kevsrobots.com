"""Library symbols (admin-curated, global) CRUD endpoints.

Sibling to :mod:`routers.symbols` (per-project). Writes are gated by
:func:`auth.require_admin` (env-driven ``ADMIN_USERNAMES`` allow-list,
defaulting to ``kev``). Public ``GET`` so non-owner visitors of a
project's schematic still see the full library in the editor.

URL surface:

* ``GET    /api/library/symbols``                     — public, list (optionally filtered by ?category=).
* ``POST   /api/library/symbols``                     — admin, create.
* ``GET    /api/library/symbols/{id}``                — public, single.
* ``PUT    /api/library/symbols/{id}``                — admin, partial.
* ``DELETE /api/library/symbols/{id}``                — admin.
* ``POST   /api/library/symbols/promote-from-project``
                                                       — admin, copies a ProjectSymbol into the library.

The ``symbol_data`` JSON document is opaque to the server — same shape
as ProjectSymbol.symbol_data (``{ bodyShapes, pins }``).
"""

from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException, Query, status
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import require_admin
from ..db import get_session
from ..models import LIBRARY_SYMBOL_CATEGORIES, LibrarySymbol, ProjectSymbol
from ..schemas import (
    LibrarySymbolCreate,
    LibrarySymbolPromoteRequest,
    LibrarySymbolResponse,
    LibrarySymbolUpdate,
)

router = APIRouter(prefix="/api/library/symbols", tags=["library-symbols"])


def _to_response(sym: LibrarySymbol) -> LibrarySymbolResponse:
    return LibrarySymbolResponse(
        id=sym.id,
        name=sym.name,
        category=sym.category or "Custom",
        ref_des_prefix=sym.ref_des_prefix or "U",
        description=sym.description,
        symbol_data=sym.symbol_data,
        created_by_username=sym.created_by_username,
        updated_by_username=sym.updated_by_username,
        created_at=sym.created_at,
        updated_at=sym.updated_at,
    )


@router.get("", response_model=list[LibrarySymbolResponse])
async def list_library_symbols(
    category: str | None = Query(default=None, max_length=32),
    session: AsyncSession = Depends(get_session),
) -> list[LibrarySymbolResponse]:
    """Public list. ``?category=Sensor`` filters; omitted = all.

    Ordered by category then name so the frontend can render a stable
    grouping without an extra sort pass."""
    stmt = select(LibrarySymbol)
    if category:
        stmt = stmt.where(LibrarySymbol.category == category)
    stmt = stmt.order_by(LibrarySymbol.category.asc(), LibrarySymbol.name.asc())
    rows = (await session.scalars(stmt)).all()
    return [_to_response(r) for r in rows]


@router.get("/categories", response_model=list[str])
async def list_categories() -> list[str]:
    """Surface the server-defined category list so the frontend
    dropdown stays in sync. Returns the tuple as a list for JSON."""
    return list(LIBRARY_SYMBOL_CATEGORIES)


@router.get("/{symbol_id}", response_model=LibrarySymbolResponse)
async def get_library_symbol(
    symbol_id: int,
    session: AsyncSession = Depends(get_session),
) -> LibrarySymbolResponse:
    sym = await session.get(LibrarySymbol, symbol_id)
    if sym is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Library symbol not found")
    return _to_response(sym)


@router.post("", response_model=LibrarySymbolResponse, status_code=201)
async def create_library_symbol(
    body: LibrarySymbolCreate,
    user: str = Depends(require_admin),
    session: AsyncSession = Depends(get_session),
) -> LibrarySymbolResponse:
    sym = LibrarySymbol(
        name=body.name,
        category=body.category or "Custom",
        ref_des_prefix=(body.ref_des_prefix or "U"),
        description=body.description,
        symbol_data=body.symbol_data,
        created_by_username=user,
        updated_by_username=user,
    )
    session.add(sym)
    await session.commit()
    await session.refresh(sym)
    return _to_response(sym)


@router.put("/{symbol_id}", response_model=LibrarySymbolResponse)
async def update_library_symbol(
    symbol_id: int,
    body: LibrarySymbolUpdate,
    user: str = Depends(require_admin),
    session: AsyncSession = Depends(get_session),
) -> LibrarySymbolResponse:
    sym = await session.get(LibrarySymbol, symbol_id)
    if sym is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Library symbol not found")
    # exclude_unset so an omitted field stays untouched (matches the
    # project-symbol PUT semantics).
    data = body.model_dump(exclude_unset=True)
    for field, value in data.items():
        setattr(sym, field, value)
    sym.updated_by_username = user
    await session.commit()
    await session.refresh(sym)
    return _to_response(sym)


@router.delete("/{symbol_id}", status_code=204)
async def delete_library_symbol(
    symbol_id: int,
    user: str = Depends(require_admin),
    session: AsyncSession = Depends(get_session),
) -> None:
    sym = await session.get(LibrarySymbol, symbol_id)
    if sym is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Library symbol not found")
    await session.delete(sym)
    await session.commit()


@router.post(
    "/promote-from-project",
    response_model=LibrarySymbolResponse,
    status_code=201,
)
async def promote_from_project(
    body: LibrarySymbolPromoteRequest,
    user: str = Depends(require_admin),
    session: AsyncSession = Depends(get_session),
) -> LibrarySymbolResponse:
    """Copy a ProjectSymbol into the library (the project row stays).

    The curator can override ``name`` / ``category`` / ``ref_des_prefix``
    / ``description`` at promotion time — useful for tidying up an
    author's working name (e.g. "BME280 v2 (tested)") into something
    clean for the global library ("BME280")."""
    src = await session.get(ProjectSymbol, body.project_symbol_id)
    if src is None:
        raise HTTPException(
            status.HTTP_404_NOT_FOUND,
            detail="Source project symbol not found",
        )
    sym = LibrarySymbol(
        name=(body.name or src.name),
        category=(body.category or "Custom"),
        ref_des_prefix=(body.ref_des_prefix or src.ref_des_prefix or "U"),
        description=(body.description if body.description is not None else src.description),
        symbol_data=src.symbol_data,
        created_by_username=user,
        updated_by_username=user,
    )
    session.add(sym)
    await session.commit()
    await session.refresh(sym)
    return _to_response(sym)
