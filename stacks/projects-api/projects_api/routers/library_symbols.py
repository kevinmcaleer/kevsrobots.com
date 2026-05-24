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

import json

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


# Canonical geometry for the schematic editor's built-in symbol set.
# This is the source of truth for the symbols that ship out of the box
# (Pico, generic IC, R/C/LED, GND, V+). The schematic editor has a
# hardcoded list with the same data for backwards compat, but once an
# admin runs the seed endpoint below they land in library_symbols and
# the editor de-dupes by name so the editable library versions win.
#
# symbol_data shape matches what the Symbol Designer expects:
#   { bodyShapes: [{kind: 'rect', x, y, w, h}], pins: [{x, y, side, name, number}] }
# Coords are centred (0,0 = body centre).
def _builtin_rect_body(w: int, h: int) -> dict:
    return {"kind": "rect", "x": -w // 2, "y": -h // 2, "w": w, "h": h}


def _pin_from_side(side: str, offset: int, w: int, h: int,
                   name: str, number: str) -> dict:
    """Convert an (edge, offset-from-top-or-left) pin into the centred
    (x, y) coords the Symbol Designer authors with."""
    if side == "left":
        x, y = -w // 2, -h // 2 + offset
    elif side == "right":
        x, y = w // 2, -h // 2 + offset
    elif side == "top":
        x, y = -w // 2 + offset, -h // 2
    else:  # bottom
        x, y = -w // 2 + offset, h // 2
    return {"x": x, "y": y, "side": side, "name": name, "number": number}


_BUILTIN_PICO_PINS = [
    ("1",  "GP0",    "left",  32),  ("2",  "GP1",    "left",  64),
    ("3",  "GND",    "left",  96),  ("4",  "GP2",    "left",  128),
    ("5",  "GP3",    "left",  160), ("6",  "GP4",    "left",  192),
    ("7",  "GP5",    "left",  224), ("8",  "GND",    "left",  256),
    ("9",  "GP6",    "left",  288),
    ("36", "3V3",    "right", 32),  ("37", "3V3_EN", "right", 64),
    ("38", "GND",    "right", 96),  ("39", "VSYS",   "right", 128),
    ("40", "VBUS",   "right", 160), ("34", "GP28",   "right", 192),
    ("33", "GND",    "right", 224), ("32", "GP27",   "right", 256),
    ("31", "GP26",   "right", 288),
]

_BUILTIN_DEFINITIONS = [
    # name, category, ref_des_prefix, w, h, pins[(number, name, side, offset)]
    ("Raspberry Pi Pico", "Module",    "U",   160, 320, _BUILTIN_PICO_PINS),
    ("Generic 14-pin IC", "Active",    "U",   128, 224, [
        ("1", "1", "left",  32),  ("2", "2", "left",  64),
        ("3", "3", "left",  96),  ("4", "4", "left",  128),
        ("5", "5", "left",  160), ("6", "6", "left",  192),
        ("7", "GND", "left", 192),
        ("14", "VCC", "right", 32),  ("13", "13", "right", 64),
        ("12", "12", "right", 96),   ("11", "11", "right", 128),
        ("10", "10", "right", 160),  ("9",  "9",  "right", 192),
        ("8",  "8",  "right", 192),
    ]),
    ("Resistor",          "Passive",   "R",   64,  32,  [
        ("1", "1", "left",  16), ("2", "2", "right", 16),
    ]),
    ("Capacitor",         "Passive",   "C",   64,  32,  [
        ("1", "+", "left",  16), ("2", "-", "right", 16),
    ]),
    ("LED",               "Active",    "D",   64,  48,  [
        ("1", "A", "left",  24), ("2", "K", "right", 24),
    ]),
    ("GND",               "Power",     "GND", 48,  32,  [
        ("1", "GND", "top", 24),
    ]),
    ("V+ rail",           "Power",     "V",   48,  32,  [
        ("1", "V+", "bottom", 24),
    ]),
]


def _builtin_symbol_data_json(w: int, h: int, pins_spec) -> str:
    return json.dumps({
        "bodyShapes": [_builtin_rect_body(w, h)],
        "pins": [
            _pin_from_side(side, off, w, h, name, num)
            for (num, name, side, off) in pins_spec
        ],
    })

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
    "/seed-builtins",
    response_model=list[LibrarySymbolResponse],
    status_code=201,
)
async def seed_builtin_symbols(
    user: str = Depends(require_admin),
    session: AsyncSession = Depends(get_session),
) -> list[LibrarySymbolResponse]:
    """Insert the canonical built-in symbol set into library_symbols.

    Idempotent — runs an existence check by name before insert, so
    calling it twice doesn't duplicate. Once seeded, the schematic
    editor de-dupes by name so the editable library versions shadow
    the hardcoded built-in list. Admins can then edit each via
    /admin/symbols/ → Edit design like any other library symbol.

    Returns the full list of seeded symbols (whether newly created
    or already present), so the admin UI can refresh in one shot.
    """
    existing = (await session.scalars(
        select(LibrarySymbol).where(
            LibrarySymbol.name.in_([d[0] for d in _BUILTIN_DEFINITIONS])
        )
    )).all()
    existing_names = {s.name for s in existing}

    created: list[LibrarySymbol] = []
    for (name, category, refdes, w, h, pins_spec) in _BUILTIN_DEFINITIONS:
        if name in existing_names:
            continue
        sym = LibrarySymbol(
            name=name,
            category=category,
            ref_des_prefix=refdes,
            description=f"Built-in {name} symbol",
            symbol_data=_builtin_symbol_data_json(w, h, pins_spec),
            created_by_username=user,
            updated_by_username=user,
        )
        session.add(sym)
        created.append(sym)
    if created:
        await session.commit()
        for sym in created:
            await session.refresh(sym)

    # Return ALL canonical built-ins (newly created + already existing)
    # so the UI sees the full set in one response.
    all_builtins = (await session.scalars(
        select(LibrarySymbol)
        .where(LibrarySymbol.name.in_([d[0] for d in _BUILTIN_DEFINITIONS]))
        .order_by(LibrarySymbol.name.asc())
    )).all()
    return [_to_response(s) for s in all_builtins]


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
