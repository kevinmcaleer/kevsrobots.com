"""Async SQLAlchemy engine, session factory, and declarative Base."""

from __future__ import annotations

import logging
from typing import AsyncIterator

from sqlalchemy import text
from sqlalchemy.ext.asyncio import (
    AsyncEngine,
    AsyncSession,
    async_sessionmaker,
    create_async_engine,
)
from sqlalchemy.orm import DeclarativeBase

from .config import get_settings

logger = logging.getLogger(__name__)


class Base(DeclarativeBase):
    pass


_engine: AsyncEngine | None = None
_sessionmaker: async_sessionmaker[AsyncSession] | None = None


def get_engine() -> AsyncEngine:
    global _engine
    if _engine is None:
        settings = get_settings()
        _engine = create_async_engine(settings.database_url, future=True)
    return _engine


def get_sessionmaker() -> async_sessionmaker[AsyncSession]:
    global _sessionmaker
    if _sessionmaker is None:
        _sessionmaker = async_sessionmaker(
            bind=get_engine(),
            expire_on_commit=False,
            class_=AsyncSession,
        )
    return _sessionmaker


async def get_session() -> AsyncIterator[AsyncSession]:
    sessionmaker = get_sessionmaker()
    async with sessionmaker() as session:
        yield session


async def create_all() -> None:
    from . import models  # noqa: F401

    engine = get_engine()
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)


async def repair_stale_fks() -> None:
    """Detect and rewire FK constraints that still reference legacy tables.

    Some `project_*` tables were created during an early migration with
    FKs pointing at the now-deleted ``projects_old`` table. Any INSERT
    into those tables fails with a ForeignKeyViolationError even though
    the row in ``projects`` exists. This walks ``information_schema`` for
    FKs that reference ``projects_old`` (or any other ``_old`` shadow
    table) and rebuilds them to point at the live table, preserving the
    original ON DELETE rule.

    Safe to run on every startup — no-op when there's nothing to fix.
    Postgres-only; silently skips on other dialects (e.g. SQLite tests).
    """
    engine = get_engine()
    if engine.dialect.name != "postgresql":
        return
    async with engine.begin() as conn:
        rows = await conn.execute(text(
            """
            SELECT
              tc.table_name        AS owning_table,
              tc.constraint_name   AS fk_name,
              kcu.column_name      AS fk_column,
              ccu.table_name       AS legacy_target,
              rc.delete_rule       AS delete_rule
            FROM information_schema.table_constraints tc
            JOIN information_schema.referential_constraints rc
              ON rc.constraint_name = tc.constraint_name
              AND rc.constraint_schema = tc.constraint_schema
            JOIN information_schema.key_column_usage kcu
              ON kcu.constraint_name = tc.constraint_name
              AND kcu.constraint_schema = tc.constraint_schema
            JOIN information_schema.constraint_column_usage ccu
              ON ccu.constraint_name = tc.constraint_name
              AND ccu.constraint_schema = tc.constraint_schema
            WHERE tc.constraint_type = 'FOREIGN KEY'
              AND ccu.table_name LIKE '%\\_old' ESCAPE '\\'
            """
        ))
        bad_fks = rows.fetchall()
        if not bad_fks:
            return

        for owning_table, fk_name, fk_column, legacy_target, delete_rule in bad_fks:
            # Strip the trailing _old to find the live table name
            live_target = legacy_target[:-4]
            check = await conn.execute(text(
                "SELECT 1 FROM information_schema.tables "
                "WHERE table_schema = current_schema() AND table_name = :t"
            ), {"t": live_target})
            if check.first() is None:
                logger.warning(
                    "Skipping repair of %s.%s: live target %s does not exist",
                    owning_table, fk_name, live_target,
                )
                continue

            cascade = ""
            if delete_rule and delete_rule.upper() not in {"NO ACTION", "RESTRICT"}:
                cascade = f" ON DELETE {delete_rule}"

            logger.warning(
                "Rewiring stale FK %s.%s: %s -> %s%s",
                owning_table, fk_name, legacy_target, live_target, cascade or " (no delete rule)",
            )
            await conn.execute(text(
                f'ALTER TABLE "{owning_table}" DROP CONSTRAINT "{fk_name}"'
            ))
            await conn.execute(text(
                f'ALTER TABLE "{owning_table}" ADD CONSTRAINT "{fk_name}" '
                f'FOREIGN KEY ("{fk_column}") REFERENCES "{live_target}" (id){cascade}'
            ))
