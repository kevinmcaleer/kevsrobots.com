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


async def add_remix_columns_if_missing() -> None:
    """Additive migration for issue #108 (project remixes).

    Adds ``remixed_from_id`` (FK -> projects.id ON DELETE SET NULL) and
    ``remix_description`` (TEXT) to the existing ``projects`` table when
    they do not already exist. SQLAlchemy's ``create_all`` does not ALTER
    existing tables, so this lightweight idempotent helper keeps existing
    Postgres deployments in sync. Safe to run on every startup; no-op when
    the columns are already present. Postgres-only — SQLite test runs use
    ``create_all`` against a fresh in-memory DB which already includes the
    new columns.
    """
    engine = get_engine()
    if engine.dialect.name != "postgresql":
        return
    async with engine.begin() as conn:
        # Check existence of the projects table first; if the DB is brand
        # new, create_all will already have built it with these columns.
        table_check = await conn.execute(text(
            "SELECT 1 FROM information_schema.tables "
            "WHERE table_schema = current_schema() AND table_name = 'projects'"
        ))
        if table_check.first() is None:
            return

        existing = await conn.execute(text(
            "SELECT column_name FROM information_schema.columns "
            "WHERE table_schema = current_schema() AND table_name = 'projects'"
        ))
        cols = {row[0] for row in existing.fetchall()}

        if "remixed_from_id" not in cols:
            logger.warning("Adding projects.remixed_from_id column (issue #108)")
            await conn.execute(text(
                'ALTER TABLE "projects" ADD COLUMN "remixed_from_id" INTEGER '
                'REFERENCES "projects" (id) ON DELETE SET NULL'
            ))
            await conn.execute(text(
                'CREATE INDEX IF NOT EXISTS "ix_projects_remixed_from_id" '
                'ON "projects" ("remixed_from_id")'
            ))
        if "remix_description" not in cols:
            logger.warning("Adding projects.remix_description column (issue #108)")
            await conn.execute(text(
                'ALTER TABLE "projects" ADD COLUMN "remix_description" TEXT'
            ))


async def add_bom_part_id_if_missing() -> None:
    """Additive migration for issue #121 (parts catalog).

    Adds ``part_id`` (FK -> parts.id ON DELETE SET NULL) to the existing
    ``project_bom_items`` table when it does not already exist. Without
    this, every SELECT / INSERT on the BOM table 500s after a redeploy
    that ships the new ORM model. Idempotent — runs on every startup.
    Postgres-only; SQLite tests use a fresh in-memory DB.
    """
    engine = get_engine()
    if engine.dialect.name != "postgresql":
        return
    async with engine.begin() as conn:
        table_check = await conn.execute(text(
            "SELECT 1 FROM information_schema.tables "
            "WHERE table_schema = current_schema() AND table_name = 'project_bom_items'"
        ))
        if table_check.first() is None:
            return

        existing = await conn.execute(text(
            "SELECT column_name FROM information_schema.columns "
            "WHERE table_schema = current_schema() AND table_name = 'project_bom_items'"
        ))
        cols = {row[0] for row in existing.fetchall()}

        if "part_id" not in cols:
            logger.warning("Adding project_bom_items.part_id column (issue #121)")
            # Add column first without FK so the migration succeeds even if
            # the parts table hasn't been created yet (it will be by
            # create_all on the same startup pass).
            await conn.execute(text(
                'ALTER TABLE "project_bom_items" ADD COLUMN "part_id" INTEGER'
            ))
            # Check whether parts table exists before adding the FK.
            parts_check = await conn.execute(text(
                "SELECT 1 FROM information_schema.tables "
                "WHERE table_schema = current_schema() AND table_name = 'parts'"
            ))
            if parts_check.first() is not None:
                await conn.execute(text(
                    'ALTER TABLE "project_bom_items" ADD CONSTRAINT '
                    '"fk_project_bom_items_part_id" FOREIGN KEY ("part_id") '
                    'REFERENCES "parts" (id) ON DELETE SET NULL'
                ))
            await conn.execute(text(
                'CREATE INDEX IF NOT EXISTS "ix_project_bom_items_part_id" '
                'ON "project_bom_items" ("part_id")'
            ))
