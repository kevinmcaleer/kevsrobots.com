"""Async SQLAlchemy engine, session factory, and declarative Base."""

from __future__ import annotations

import logging
from typing import AsyncIterator

from sqlalchemy import select, text
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


async def add_part_category_family_if_missing() -> None:
    """Additive migration for issue #135 (related parts + category + family).

    Adds ``category`` and ``family`` columns to the existing ``parts``
    table, and the matching snapshot columns to ``part_revisions``, when
    they do not already exist. Also indexes the two new ``parts`` columns
    so filtering by family / category is cheap. Idempotent — safe to run
    on every startup. Postgres-only; SQLite tests use a fresh in-memory
    DB and ``create_all`` already includes the new columns.

    The new ``part_relations`` table is handled by ``create_all`` and
    therefore does NOT need a helper here.
    """
    engine = get_engine()
    if engine.dialect.name != "postgresql":
        return
    async with engine.begin() as conn:
        for table_name, cols_to_add in (
            (
                "parts",
                [
                    ("category", 'VARCHAR(60)'),
                    ("family", 'VARCHAR(80)'),
                ],
            ),
            (
                "part_revisions",
                [
                    ("category", 'VARCHAR(60)'),
                    ("family", 'VARCHAR(80)'),
                ],
            ),
        ):
            table_check = await conn.execute(text(
                "SELECT 1 FROM information_schema.tables "
                "WHERE table_schema = current_schema() AND table_name = :t"
            ), {"t": table_name})
            if table_check.first() is None:
                continue

            existing = await conn.execute(text(
                "SELECT column_name FROM information_schema.columns "
                "WHERE table_schema = current_schema() AND table_name = :t"
            ), {"t": table_name})
            cols = {row[0] for row in existing.fetchall()}

            for col_name, col_type in cols_to_add:
                if col_name in cols:
                    continue
                logger.warning(
                    "Adding %s.%s column (issue #135)", table_name, col_name
                )
                await conn.execute(text(
                    f'ALTER TABLE "{table_name}" ADD COLUMN "{col_name}" {col_type}'
                ))
                if table_name == "parts":
                    await conn.execute(text(
                        f'CREATE INDEX IF NOT EXISTS "ix_parts_{col_name}" '
                        f'ON "parts" ("{col_name}")'
                    ))


async def add_project_featured_columns_if_missing() -> None:
    """Additive migration for issue #115 (featured projects).

    Adds ``is_featured`` (BOOLEAN, default false), ``featured_at``
    (TIMESTAMP NULL), ``featured_by`` (VARCHAR(100) NULL), and
    ``featured_note`` (VARCHAR(200) NULL) to the existing ``projects``
    table when they do not already exist. Also creates the
    ``ix_projects_is_featured`` index so ``GET /api/projects/featured``
    stays cheap. SQLAlchemy's ``create_all`` does not ALTER existing
    tables, so this lightweight idempotent helper keeps existing Postgres
    deployments in sync. Safe to run on every startup; no-op when the
    columns are already present. Postgres-only — SQLite test runs use
    ``create_all`` against a fresh in-memory DB which already includes
    the new columns.
    """
    engine = get_engine()
    if engine.dialect.name != "postgresql":
        return
    async with engine.begin() as conn:
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

        if "is_featured" not in cols:
            logger.warning("Adding projects.is_featured column (issue #115)")
            await conn.execute(text(
                'ALTER TABLE "projects" ADD COLUMN "is_featured" BOOLEAN '
                "NOT NULL DEFAULT FALSE"
            ))
        if "featured_at" not in cols:
            logger.warning("Adding projects.featured_at column (issue #115)")
            await conn.execute(text(
                'ALTER TABLE "projects" ADD COLUMN "featured_at" TIMESTAMP'
            ))
        if "featured_by" not in cols:
            logger.warning("Adding projects.featured_by column (issue #115)")
            await conn.execute(text(
                'ALTER TABLE "projects" ADD COLUMN "featured_by" VARCHAR(100)'
            ))
        if "featured_note" not in cols:
            logger.warning("Adding projects.featured_note column (issue #115)")
            await conn.execute(text(
                'ALTER TABLE "projects" ADD COLUMN "featured_note" VARCHAR(200)'
            ))
        # Always (re-)assert the index — IF NOT EXISTS makes it cheap.
        await conn.execute(text(
            'CREATE INDEX IF NOT EXISTS "ix_projects_is_featured" '
            'ON "projects" ("is_featured")'
        ))


async def add_user_disabled_columns_if_missing() -> None:
    """Additive migration for issue #136 (mass-deletion auto-disable).

    The ``users`` table is brand-new in this PR — ``create_all`` will build
    it from scratch on fresh databases with ``is_disabled`` and
    ``disabled_reason`` already in place. This helper handles the
    (theoretical) case where an older deployment created the table without
    these columns, e.g. via a partial schema rollout. Idempotent — runs on
    every startup and is a no-op when everything is already in sync.
    Postgres-only; SQLite tests rely on ``create_all`` against a fresh DB.
    """
    engine = get_engine()
    if engine.dialect.name != "postgresql":
        return
    async with engine.begin() as conn:
        table_check = await conn.execute(text(
            "SELECT 1 FROM information_schema.tables "
            "WHERE table_schema = current_schema() AND table_name = 'users'"
        ))
        if table_check.first() is None:
            # Table doesn't exist yet — create_all on the same startup pass
            # will build it with the new columns. Nothing to ALTER.
            return

        existing = await conn.execute(text(
            "SELECT column_name FROM information_schema.columns "
            "WHERE table_schema = current_schema() AND table_name = 'users'"
        ))
        cols = {row[0] for row in existing.fetchall()}

        if "is_disabled" not in cols:
            logger.warning("Adding users.is_disabled column (issue #136)")
            await conn.execute(text(
                'ALTER TABLE "users" ADD COLUMN "is_disabled" BOOLEAN '
                'NOT NULL DEFAULT FALSE'
            ))
        if "disabled_reason" not in cols:
            logger.warning("Adding users.disabled_reason column (issue #136)")
            await conn.execute(text(
                'ALTER TABLE "users" ADD COLUMN "disabled_reason" TEXT'
            ))
        if "disabled_at" not in cols:
            logger.warning("Adding users.disabled_at column (issue #136)")
            await conn.execute(text(
                'ALTER TABLE "users" ADD COLUMN "disabled_at" TIMESTAMP'
            ))


async def add_user_terms_acceptance_if_missing() -> None:
    """Additive migration for the T&Cs acceptance gate (terms-gate).

    Adds ``terms_accepted_at`` (TIMESTAMP NULL) and
    ``terms_accepted_version`` (VARCHAR(20) NULL) to the existing
    ``users`` table when they do not already exist. On fresh deploys
    ``create_all`` builds the columns from the ORM definition, so this
    helper is a no-op there. Postgres-only; SQLite tests rely on
    ``create_all`` against a fresh in-memory DB.

    Safe to run on every startup — runs an idempotent
    ``information_schema`` probe before each ALTER.
    """
    engine = get_engine()
    if engine.dialect.name != "postgresql":
        return
    async with engine.begin() as conn:
        table_check = await conn.execute(text(
            "SELECT 1 FROM information_schema.tables "
            "WHERE table_schema = current_schema() AND table_name = 'users'"
        ))
        if table_check.first() is None:
            # Brand-new deployment — create_all on the same startup pass
            # will build the table with every column already present.
            return

        existing = await conn.execute(text(
            "SELECT column_name FROM information_schema.columns "
            "WHERE table_schema = current_schema() AND table_name = 'users'"
        ))
        cols = {row[0] for row in existing.fetchall()}

        if "terms_accepted_at" not in cols:
            logger.warning("Adding users.terms_accepted_at column (terms-gate)")
            await conn.execute(text(
                'ALTER TABLE "users" ADD COLUMN "terms_accepted_at" TIMESTAMP'
            ))
        if "terms_accepted_version" not in cols:
            logger.warning(
                "Adding users.terms_accepted_version column (terms-gate)"
            )
            await conn.execute(text(
                'ALTER TABLE "users" ADD COLUMN "terms_accepted_version" '
                'VARCHAR(20)'
            ))


async def add_supplier_health_columns_if_missing() -> None:
    """Additive migration for issue #122 Phase 2 (supplier link health).

    Adds ``last_status_code`` (INTEGER), ``is_broken`` (BOOLEAN NOT NULL
    DEFAULT FALSE), and ``consecutive_failures`` (INTEGER NOT NULL DEFAULT
    0) to the existing ``part_suppliers`` table when they do not already
    exist. The pre-existing ``last_checked_at`` and ``last_status`` columns
    are kept untouched. Idempotent — runs on every startup. Postgres-only;
    SQLite tests use a fresh in-memory DB that already includes the new
    columns via ``create_all``.
    """
    engine = get_engine()
    if engine.dialect.name != "postgresql":
        return
    async with engine.begin() as conn:
        table_check = await conn.execute(text(
            "SELECT 1 FROM information_schema.tables "
            "WHERE table_schema = current_schema() AND table_name = 'part_suppliers'"
        ))
        if table_check.first() is None:
            # Brand-new deployment — create_all builds the table with every
            # column already present.
            return

        existing = await conn.execute(text(
            "SELECT column_name FROM information_schema.columns "
            "WHERE table_schema = current_schema() AND table_name = 'part_suppliers'"
        ))
        cols = {row[0] for row in existing.fetchall()}

        if "last_status_code" not in cols:
            logger.warning("Adding part_suppliers.last_status_code column (issue #122)")
            await conn.execute(text(
                'ALTER TABLE "part_suppliers" ADD COLUMN "last_status_code" INTEGER'
            ))
        if "is_broken" not in cols:
            logger.warning("Adding part_suppliers.is_broken column (issue #122)")
            await conn.execute(text(
                'ALTER TABLE "part_suppliers" ADD COLUMN "is_broken" BOOLEAN '
                'NOT NULL DEFAULT FALSE'
            ))
        if "consecutive_failures" not in cols:
            logger.warning("Adding part_suppliers.consecutive_failures column (issue #122)")
            await conn.execute(text(
                'ALTER TABLE "part_suppliers" ADD COLUMN "consecutive_failures" '
                'INTEGER NOT NULL DEFAULT 0'
            ))


async def add_part_status_columns_if_missing() -> None:
    """Additive migration for issue #122 Phase 2 (part lifecycle).

    ``parts.status`` already exists from Phase 1 (issue #121), but
    ``verified_at`` and ``verified_signals`` are new. Also (re-)asserts the
    ``ix_parts_status`` index because Phase 1 created the column without
    one. Idempotent — runs on every startup. Postgres-only; SQLite tests
    use ``create_all`` against a fresh in-memory DB.
    """
    engine = get_engine()
    if engine.dialect.name != "postgresql":
        return
    async with engine.begin() as conn:
        table_check = await conn.execute(text(
            "SELECT 1 FROM information_schema.tables "
            "WHERE table_schema = current_schema() AND table_name = 'parts'"
        ))
        if table_check.first() is None:
            return

        existing = await conn.execute(text(
            "SELECT column_name FROM information_schema.columns "
            "WHERE table_schema = current_schema() AND table_name = 'parts'"
        ))
        cols = {row[0] for row in existing.fetchall()}

        if "verified_at" not in cols:
            logger.warning("Adding parts.verified_at column (issue #122)")
            await conn.execute(text(
                'ALTER TABLE "parts" ADD COLUMN "verified_at" TIMESTAMP'
            ))
        if "verified_signals" not in cols:
            logger.warning("Adding parts.verified_signals column (issue #122)")
            await conn.execute(text(
                'ALTER TABLE "parts" ADD COLUMN "verified_signals" INTEGER '
                'NOT NULL DEFAULT 0'
            ))
        # Always (re-)assert the supporting index. Phase 1 created the
        # column without one; ``IF NOT EXISTS`` keeps this cheap.
        await conn.execute(text(
            'CREATE INDEX IF NOT EXISTS "ix_parts_status" ON "parts" ("status")'
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


async def add_supplier_country_if_missing() -> None:
    """Additive migration for issue #149 (supplier country tags).

    Adds ``country_code`` (CHAR(2), nullable) to the existing
    ``part_suppliers`` table when it does not already exist. SQLAlchemy's
    ``create_all`` does not ALTER existing tables, so this lightweight
    idempotent helper keeps existing Postgres deployments in sync. Safe
    to run on every startup; no-op when the column is already present.
    Postgres-only — SQLite tests use a fresh in-memory DB which already
    includes the new column.
    """
    engine = get_engine()
    if engine.dialect.name != "postgresql":
        return
    async with engine.begin() as conn:
        table_check = await conn.execute(text(
            "SELECT 1 FROM information_schema.tables "
            "WHERE table_schema = current_schema() AND table_name = 'part_suppliers'"
        ))
        if table_check.first() is None:
            # Table doesn't exist yet — create_all on the same startup pass
            # will build it with the new column.
            return

        existing = await conn.execute(text(
            "SELECT column_name FROM information_schema.columns "
            "WHERE table_schema = current_schema() AND table_name = 'part_suppliers'"
        ))
        cols = {row[0] for row in existing.fetchall()}

        if "country_code" not in cols:
            logger.warning("Adding part_suppliers.country_code column (issue #149)")
            await conn.execute(text(
                'ALTER TABLE "part_suppliers" ADD COLUMN "country_code" CHAR(2)'
            ))


async def add_supplier_pricing_if_missing() -> None:
    """Additive migration for the supplier-pricing feature.

    Adds ``unit_cost`` (Float, nullable) + ``currency_code`` (CHAR(3),
    nullable) to the existing ``part_suppliers`` table when they do not
    already exist. SQLAlchemy's ``create_all`` does not ALTER existing
    tables, so this lightweight idempotent helper keeps existing Postgres
    deployments in sync. Safe to run on every startup; no-op when the
    columns are already present. Postgres-only — SQLite tests use a
    fresh in-memory DB which already includes the new columns.

    Mirrors ``add_supplier_country_if_missing`` (issue #149) — both touch
    part_suppliers and follow the same probe-then-ALTER pattern.
    """
    engine = get_engine()
    if engine.dialect.name != "postgresql":
        return
    async with engine.begin() as conn:
        table_check = await conn.execute(text(
            "SELECT 1 FROM information_schema.tables "
            "WHERE table_schema = current_schema() AND table_name = 'part_suppliers'"
        ))
        if table_check.first() is None:
            # Brand-new deployment — create_all builds the table with every
            # column already present.
            return

        existing = await conn.execute(text(
            "SELECT column_name FROM information_schema.columns "
            "WHERE table_schema = current_schema() AND table_name = 'part_suppliers'"
        ))
        cols = {row[0] for row in existing.fetchall()}

        if "unit_cost" not in cols:
            logger.warning(
                "Adding part_suppliers.unit_cost column (supplier-pricing-feature)"
            )
            await conn.execute(text(
                'ALTER TABLE "part_suppliers" ADD COLUMN "unit_cost" '
                'DOUBLE PRECISION'
            ))
        if "currency_code" not in cols:
            logger.warning(
                "Adding part_suppliers.currency_code column (supplier-pricing-feature)"
            )
            await conn.execute(text(
                'ALTER TABLE "part_suppliers" ADD COLUMN "currency_code" CHAR(3)'
            ))


async def add_bom_supplier_id_if_missing() -> None:
    """Additive migration for the supplier-pricing feature.

    Adds ``supplier_id`` (FK -> part_suppliers.id ON DELETE SET NULL) to
    the existing ``project_bom_items`` table when it does not already
    exist. Without this, every SELECT / INSERT on the BOM table 500s
    after a redeploy that ships the new ORM model.

    Two-step pattern mirrors ``add_bom_part_id_if_missing`` (issue #121):
    add the bare column first so the migration succeeds even if
    ``part_suppliers`` doesn't exist yet (legacy DBs where the parts
    catalog landed in a later migration pass), then conditionally add
    the FK constraint only if the target table is present. Idempotent —
    safe to run on every startup. Postgres-only; SQLite tests use a
    fresh in-memory DB built by ``create_all``.
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

        if "supplier_id" not in cols:
            logger.warning(
                "Adding project_bom_items.supplier_id column "
                "(supplier-pricing-feature)"
            )
            # Step 1: bare column, no FK — succeeds even if part_suppliers
            # hasn't been created yet on this legacy DB.
            await conn.execute(text(
                'ALTER TABLE "project_bom_items" ADD COLUMN "supplier_id" INTEGER'
            ))
            # Step 2: add the FK constraint only if the target table
            # exists. On the same startup pass ``create_all`` will have
            # built ``part_suppliers`` for fresh deployments, but on the
            # legacy-without-parts case we leave the constraint off and
            # rely on the resolver to silently treat any stale id as NULL.
            suppliers_check = await conn.execute(text(
                "SELECT 1 FROM information_schema.tables "
                "WHERE table_schema = current_schema() AND table_name = 'part_suppliers'"
            ))
            if suppliers_check.first() is not None:
                await conn.execute(text(
                    'ALTER TABLE "project_bom_items" ADD CONSTRAINT '
                    '"fk_project_bom_items_supplier_id" FOREIGN KEY '
                    '("supplier_id") REFERENCES "part_suppliers" (id) '
                    'ON DELETE SET NULL'
                ))
            await conn.execute(text(
                'CREATE INDEX IF NOT EXISTS "ix_project_bom_items_supplier_id" '
                'ON "project_bom_items" ("supplier_id")'
            ))


async def add_bom_currency_if_missing() -> None:
    """Additive migration for issue #149 (BOM currency codes).

    Adds ``currency_code`` (CHAR(3), nullable) to the existing
    ``project_bom_items`` table when it does not already exist.
    Idempotent — runs on every startup; no-op when the column is already
    present. Postgres-only; SQLite tests use a fresh in-memory DB which
    already includes the new column.
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

        if "currency_code" not in cols:
            logger.warning("Adding project_bom_items.currency_code column (issue #149)")
            await conn.execute(text(
                'ALTER TABLE "project_bom_items" ADD COLUMN "currency_code" CHAR(3)'
            ))


async def add_project_slug_if_missing() -> None:
    """Additive migration for issue #152 (project owner/slug URLs).

    Adds ``slug`` (VARCHAR(80), nullable) to the existing ``projects``
    table when it does not already exist, creates the supporting index
    and the ``(author_username, slug)`` unique constraint, and backfills
    legacy rows whose slug column is NULL by deriving one from the title.
    Idempotent — safe to run on every startup; no-op when nothing is
    missing. Postgres-only — SQLite test runs use ``create_all`` against
    a fresh in-memory DB which already includes the new column.

    Concurrency: the backfill loop generates per-author unique slugs by
    consulting the live ``projects`` table inside the same transaction;
    two replicas racing to backfill the same rows is fine — the unique
    constraint guarantees one of them rolls back on conflict.
    """
    engine = get_engine()
    if engine.dialect.name != "postgresql":
        return
    async with engine.begin() as conn:
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

        if "slug" not in cols:
            logger.warning("Adding projects.slug column (issue #152)")
            await conn.execute(text(
                'ALTER TABLE "projects" ADD COLUMN "slug" VARCHAR(80)'
            ))

        # Always (re-)assert the supporting index — IF NOT EXISTS makes it
        # cheap on subsequent restarts.
        await conn.execute(text(
            'CREATE INDEX IF NOT EXISTS "ix_projects_slug" '
            'ON "projects" ("slug")'
        ))

        # And the (author_username, slug) unique constraint. ``ADD
        # CONSTRAINT IF NOT EXISTS`` isn't a thing on older Postgres, so we
        # probe information_schema first.
        constraint_check = await conn.execute(text(
            "SELECT 1 FROM information_schema.table_constraints "
            "WHERE table_schema = current_schema() "
            "  AND table_name = 'projects' "
            "  AND constraint_name = 'uq_projects_author_slug'"
        ))
        if constraint_check.first() is None:
            try:
                await conn.execute(text(
                    'ALTER TABLE "projects" ADD CONSTRAINT '
                    '"uq_projects_author_slug" UNIQUE ("author_username", "slug")'
                ))
            except Exception as exc:  # noqa: BLE001 — log + continue
                # If duplicate (author, slug) tuples already exist on
                # legacy data, the constraint creation will fail. The
                # backfill below dedupes via the suffix-loop and will
                # succeed on the next restart.
                logger.warning(
                    "Could not add uq_projects_author_slug yet (will retry next boot): %s",
                    exc,
                )

    # Backfill empty slugs from titles. Done in a separate transaction so
    # the schema changes above commit even if the backfill is interrupted.
    await _backfill_project_slugs()


async def _backfill_project_slugs() -> None:
    """Populate ``Project.slug`` for legacy rows where it's NULL.

    Loads (id, author_username, title) for NULL-slug rows in a single
    query, then writes a unique-per-author slug for each row using the
    same algorithm as the create-path. Idempotent — re-running after a
    crash skips already-populated rows because ``slug IS NULL`` filters
    them out.
    """
    from .models import Project
    from .slugs import slugify_title, unique_slug_for_author

    sessionmaker = get_sessionmaker()
    async with sessionmaker() as session:
        rows = (
            await session.execute(
                select(Project.id, Project.author_username, Project.title)
                .where(Project.slug.is_(None))
            )
        ).all()
        if not rows:
            return
        logger.warning(
            "Backfilling slugs for %d legacy projects (issue #152)", len(rows)
        )
        for pid, author, title in rows:
            base = slugify_title(title or "")
            try:
                slug = await unique_slug_for_author(
                    session,
                    author_username=author,
                    base_slug=base,
                )
                project = await session.get(Project, pid)
                if project is None:
                    continue
                project.slug = slug
                await session.flush()
            except Exception as exc:  # noqa: BLE001
                logger.warning(
                    "Failed to backfill slug for project %d: %s — skipping",
                    pid, exc,
                )
                await session.rollback()
        await session.commit()


async def add_user_profile_columns_if_missing() -> None:
    """Additive migration for issue #111 (public user profiles).

    The ``user_profiles`` table is brand-new — on fresh deployments
    ``create_all`` builds it with every column present. This helper is
    defensive: if a deployment is upgraded from a preview branch that
    shipped only a subset of the columns, the missing ones are added
    here. Postgres-only; SQLite tests use a fresh in-memory DB so the
    table always has the full set already.

    Safe to run on every startup — no-op when nothing is missing.
    """
    engine = get_engine()
    if engine.dialect.name != "postgresql":
        return
    async with engine.begin() as conn:
        table_check = await conn.execute(text(
            "SELECT 1 FROM information_schema.tables "
            "WHERE table_schema = current_schema() AND table_name = 'user_profiles'"
        ))
        if table_check.first() is None:
            # Table doesn't exist yet — create_all in the same lifespan
            # pass builds it with every column.
            return

        existing = await conn.execute(text(
            "SELECT column_name FROM information_schema.columns "
            "WHERE table_schema = current_schema() AND table_name = 'user_profiles'"
        ))
        cols = {row[0] for row in existing.fetchall()}

        # Column definitions: name -> SQL type. All NULLable so adding
        # to a populated table never fails. created_at / updated_at use
        # NOW() as a sane default for legacy rows.
        expected: list[tuple[str, str]] = [
            ("bio", "VARCHAR(500)"),
            ("location", "VARCHAR(120)"),
            ("website_url", "VARCHAR(200)"),
            ("social_links", "JSONB"),
            ("featured_badge_slugs", "JSONB"),
            ("created_at", "TIMESTAMP NOT NULL DEFAULT NOW()"),
            ("updated_at", "TIMESTAMP NOT NULL DEFAULT NOW()"),
        ]
        for col_name, col_type in expected:
            if col_name not in cols:
                logger.warning(
                    "Adding user_profiles.%s column (issue #111)", col_name
                )
                await conn.execute(text(
                    f'ALTER TABLE "user_profiles" ADD COLUMN "{col_name}" {col_type}'
                ))


async def add_user_currency_preference_if_missing() -> None:
    """Additive migration for issue #150 (preferred display currency).

    Adds ``user_profiles.preferred_currency`` as a nullable CHAR(3) on
    legacy Postgres deployments where the table already exists. On fresh
    deploys ``create_all`` builds the column from the ORM definition, so
    this helper is a no-op.

    Independent of #149 — does not touch ``part_suppliers.country_code``
    or ``project_bom_items.currency_code``. Those columns are added by
    their own helpers when #149 lands. #150 can ship before, after, or
    alongside #149.

    Postgres-only; SQLite tests use a fresh in-memory DB so the column
    is always present.

    Safe to run on every startup — no-op when nothing is missing.
    """
    engine = get_engine()
    if engine.dialect.name != "postgresql":
        return
    async with engine.begin() as conn:
        table_check = await conn.execute(text(
            "SELECT 1 FROM information_schema.tables "
            "WHERE table_schema = current_schema() AND table_name = 'user_profiles'"
        ))
        if table_check.first() is None:
            # Table doesn't exist yet — create_all in the same lifespan
            # pass builds it with every column.
            return

        existing = await conn.execute(text(
            "SELECT column_name FROM information_schema.columns "
            "WHERE table_schema = current_schema() AND table_name = 'user_profiles' "
            "AND column_name = 'preferred_currency'"
        ))
        if existing.first() is None:
            logger.warning(
                "Adding user_profiles.preferred_currency column (issue #150)"
            )
            await conn.execute(text(
                'ALTER TABLE "user_profiles" '
                'ADD COLUMN "preferred_currency" CHAR(3)'
            ))


async def add_instruction_step_type_if_missing() -> None:
    """Additive migration for B3 (instruction step types).

    Adds ``step_type`` / ``body`` / ``video_url`` / ``schematic_id`` columns
    to ``instruction_steps``. Postgres only; no-op on SQLite test DB which
    is recreated from Base.metadata.create_all per test.

    Each ALTER is independently IF NOT EXISTS so re-runs are safe.
    The step_type default is ``'photo'`` so existing rows backfill
    correctly under both NOT NULL and the default clause.

    Idempotent: probing ``information_schema.columns`` before each ALTER
    means calling this helper a second time is a cheap series of SELECTs
    with no writes — see ``test_step_type_migration_helper_is_idempotent``
    in ``tests/test_instructions.py``.
    """
    engine = get_engine()
    if engine.dialect.name != "postgresql":
        return
    async with engine.begin() as conn:
        table_check = await conn.execute(text(
            "SELECT 1 FROM information_schema.tables "
            "WHERE table_schema = current_schema() AND table_name = 'instruction_steps'"
        ))
        if table_check.first() is None:
            # Brand-new deployment — create_all on the same startup pass
            # will build the table with every column already present.
            return

        existing = await conn.execute(text(
            "SELECT column_name FROM information_schema.columns "
            "WHERE table_schema = current_schema() AND table_name = 'instruction_steps'"
        ))
        cols = {row[0] for row in existing.fetchall()}

        # Each ALTER in its own try block so a failure on one doesn't
        # abort the others. The NOT NULL + default on step_type means
        # legacy rows without the column backfill to 'photo' atomically
        # as part of the ALTER.
        if "step_type" not in cols:
            try:
                logger.warning("Adding instruction_steps.step_type column (B3)")
                await conn.execute(text(
                    'ALTER TABLE "instruction_steps" ADD COLUMN IF NOT EXISTS '
                    '"step_type" VARCHAR(20) NOT NULL DEFAULT \'photo\''
                ))
            except Exception as exc:  # noqa: BLE001 — log + continue
                logger.warning("Failed to add instruction_steps.step_type: %s", exc)
        if "body" not in cols:
            try:
                logger.warning("Adding instruction_steps.body column (B3)")
                await conn.execute(text(
                    'ALTER TABLE "instruction_steps" ADD COLUMN IF NOT EXISTS '
                    '"body" TEXT'
                ))
            except Exception as exc:  # noqa: BLE001
                logger.warning("Failed to add instruction_steps.body: %s", exc)
        if "video_url" not in cols:
            try:
                logger.warning("Adding instruction_steps.video_url column (B3)")
                await conn.execute(text(
                    'ALTER TABLE "instruction_steps" ADD COLUMN IF NOT EXISTS '
                    '"video_url" VARCHAR(500)'
                ))
            except Exception as exc:  # noqa: BLE001
                logger.warning("Failed to add instruction_steps.video_url: %s", exc)
        if "schematic_id" not in cols:
            try:
                logger.warning("Adding instruction_steps.schematic_id column (B3)")
                # No FK constraint here — the ``schematics`` table doesn't
                # exist yet (lands in E2). The column lives so the API can
                # round-trip values; the FK will be added by E2's helper.
                await conn.execute(text(
                    'ALTER TABLE "instruction_steps" ADD COLUMN IF NOT EXISTS '
                    '"schematic_id" INTEGER'
                ))
            except Exception as exc:  # noqa: BLE001
                logger.warning("Failed to add instruction_steps.schematic_id: %s", exc)


async def add_project_file_description_if_missing() -> None:
    """Additive migration for issue #187 (per-file descriptions).

    Adds ``description`` (TEXT, nullable) to the existing ``project_files``
    table when it does not already exist. SQLAlchemy's ``create_all`` does
    not ALTER existing tables, so this lightweight idempotent helper keeps
    existing Postgres deployments in sync. Safe to run on every startup;
    no-op when the column is already present. Postgres-only — SQLite test
    runs use a fresh in-memory DB which already includes the new column
    via ``create_all``.

    Mirrors ``add_bom_part_id_if_missing`` / ``add_remix_columns_if_missing``
    — same probe-then-ALTER pattern.
    """
    engine = get_engine()
    if engine.dialect.name != "postgresql":
        return
    async with engine.begin() as conn:
        table_check = await conn.execute(text(
            "SELECT 1 FROM information_schema.tables "
            "WHERE table_schema = current_schema() AND table_name = 'project_files'"
        ))
        if table_check.first() is None:
            # Brand-new deployment — create_all on the same startup pass
            # will build the table with the new column already present.
            return

        existing = await conn.execute(text(
            "SELECT column_name FROM information_schema.columns "
            "WHERE table_schema = current_schema() AND table_name = 'project_files' "
            "AND column_name = 'description'"
        ))
        if existing.first() is None:
            logger.warning(
                "Adding project_files.description column (issue #187)"
            )
            await conn.execute(text(
                'ALTER TABLE "project_files" ADD COLUMN "description" TEXT'
            ))
