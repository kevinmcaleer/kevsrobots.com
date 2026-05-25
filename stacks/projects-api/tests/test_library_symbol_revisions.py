"""Versioning Phase 1 — library-symbol revision backfill.

Confirms that ``backfill_library_symbol_revisions`` snapshots every
library symbol lacking a current revision as revision 1, points
``current_revision_id`` at it, and is idempotent on re-run.

Each test uses fresh sessions from ``sessionmaker_`` for write +
read-back so there's no stale identity-map cache to trip over (the
backfill commits via its own session).
"""

import pytest
from sqlalchemy import func, select

from projects_api import db as db_module
from projects_api.models import LibrarySymbol, LibrarySymbolRevision


async def _add_symbol(sessionmaker_, **kw):
    async with sessionmaker_() as s:
        sym = LibrarySymbol(**kw)
        s.add(sym)
        await s.commit()
        return sym.id


@pytest.mark.asyncio
async def test_backfill_creates_revision_one(sessionmaker_, monkeypatch):
    monkeypatch.setattr(db_module, "get_sessionmaker", lambda: sessionmaker_)
    sid = await _add_symbol(
        sessionmaker_, name="Resistor", category="Passive", ref_des_prefix="R",
        symbol_data='{"pins":[],"bodyShapes":[]}', created_by_username="kev",
    )

    await db_module.backfill_library_symbol_revisions()

    async with sessionmaker_() as s:
        sym = await s.get(LibrarySymbol, sid)
        assert sym.current_revision_id is not None
        rev = await s.get(LibrarySymbolRevision, sym.current_revision_id)
        assert rev.symbol_id == sid
        assert rev.name == "Resistor"
        assert rev.category == "Passive"
        assert rev.ref_des_prefix == "R"
        assert rev.symbol_data == '{"pins":[],"bodyShapes":[]}'
        assert rev.author == "kev"
        assert "backfill" in rev.change_summary.lower()


@pytest.mark.asyncio
async def test_backfill_is_idempotent(sessionmaker_, monkeypatch):
    monkeypatch.setattr(db_module, "get_sessionmaker", lambda: sessionmaker_)
    sid = await _add_symbol(
        sessionmaker_, name="Cap", category="Passive", ref_des_prefix="C",
        symbol_data="{}", created_by_username="kev",
    )

    await db_module.backfill_library_symbol_revisions()
    await db_module.backfill_library_symbol_revisions()  # 2nd run = no-op

    async with sessionmaker_() as s:
        count = (
            await s.execute(
                select(func.count()).select_from(LibrarySymbolRevision)
                .where(LibrarySymbolRevision.symbol_id == sid)
            )
        ).scalar()
    assert count == 1


@pytest.mark.asyncio
async def test_backfill_noop_when_already_revisioned(sessionmaker_, monkeypatch):
    monkeypatch.setattr(db_module, "get_sessionmaker", lambda: sessionmaker_)
    sid = await _add_symbol(
        sessionmaker_, name="LED", category="Active", ref_des_prefix="D",
        symbol_data="{}", created_by_username="kev",
    )
    # Pre-seed a revision + pointer as if already versioned.
    async with sessionmaker_() as s:
        rev = LibrarySymbolRevision(
            symbol_id=sid, author="kev", change_summary="hand-made",
            name="LED", category="Active", ref_des_prefix="D", symbol_data="{}",
        )
        s.add(rev)
        await s.flush()
        sym = await s.get(LibrarySymbol, sid)
        sym.current_revision_id = rev.id
        await s.commit()

    await db_module.backfill_library_symbol_revisions()

    async with sessionmaker_() as s:
        count = (
            await s.execute(
                select(func.count()).select_from(LibrarySymbolRevision)
                .where(LibrarySymbolRevision.symbol_id == sid)
            )
        ).scalar()
    assert count == 1  # backfill skipped it — no extra revision
