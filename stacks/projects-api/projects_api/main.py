"""FastAPI application entrypoint for the Projects API."""

from __future__ import annotations

import logging
from contextlib import asynccontextmanager
from typing import AsyncIterator

from apscheduler.schedulers.asyncio import AsyncIOScheduler
from apscheduler.triggers.interval import IntervalTrigger
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from .badges import seed_badge_definitions
from .config import get_settings
from .db import (
    add_bom_currency_if_missing,
    add_bom_part_id_if_missing,
    add_bom_part_revision_id_if_missing,
    add_bom_supplier_id_if_missing,
    add_instruction_step_type_if_missing,
    add_library_symbol_current_revision_if_missing,
    add_library_symbol_fork_columns_if_missing,
    add_library_symbol_power_port_if_missing,
    add_part_category_family_if_missing,
    add_part_status_columns_if_missing,
    add_part_symbol_id_if_missing,
    add_project_content_mode_if_missing,
    add_project_featured_columns_if_missing,
    add_project_file_description_if_missing,
    add_project_slug_if_missing,
    add_remix_columns_if_missing,
    add_supplier_country_if_missing,
    add_supplier_health_columns_if_missing,
    add_supplier_pricing_if_missing,
    add_user_currency_preference_if_missing,
    add_user_disabled_columns_if_missing,
    add_user_profile_columns_if_missing,
    add_user_terms_acceptance_if_missing,
    backfill_bom_part_revisions,
    backfill_library_symbol_revisions,
    backfill_part_image_url_to_photos,
    backfill_part_revisions_if_missing,
    create_all,
    migrate_image_files_to_images,
    recanonicalize_part_categories,
    get_sessionmaker,
    repair_stale_fks,
)
from .routers import (
    admin_feedback,
    admin_parts,
    auth,
    badges,
    bom,
    by_slug,
    downloads,
    featured,
    feedback,
    files,
    follows,
    fx,
    health,
    images,
    instructions,
    journal,
    links,
    makes,
    moderation,
    parts,
    parts_moderation,
    parts_photos,
    parts_talk,
    projects,
    remixes,
    library_symbols,
    schematics,
    social_og,
    staff_picks,
    symbols,
    users,
    videos,
)
from .supplier_health import background_jobs_disabled, run_full_health_check

logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncIterator[None]:
    await create_all()
    await repair_stale_fks()
    # Issue #108: ensure remix columns exist on legacy Postgres deployments.
    await add_remix_columns_if_missing()
    # Issue #121: ensure project_bom_items.part_id column exists.
    await add_bom_part_id_if_missing()
    # Issue #135: ensure parts.category / parts.family and the matching
    # part_revisions snapshot columns exist on legacy Postgres deployments.
    await add_part_category_family_if_missing()
    # Issue #115: ensure projects.is_featured + sibling columns exist.
    await add_project_featured_columns_if_missing()
    # Issue #111: defensive ALTER for the new user_profiles table —
    # no-op on fresh deploys where create_all built every column.
    await add_user_profile_columns_if_missing()
    # Issue #150: defensive ALTER for user_profiles.preferred_currency.
    # No-op on fresh deploys; only runs ALTER on legacy Postgres.
    await add_user_currency_preference_if_missing()
    # Issue #136: ensure users.is_disabled / disabled_reason columns exist.
    await add_user_disabled_columns_if_missing()
    # T&Cs acceptance gate (terms-gate): ensure terms_accepted_at +
    # terms_accepted_version columns exist on legacy Postgres deployments.
    await add_user_terms_acceptance_if_missing()
    # Issue #152: add projects.slug + (author_username, slug) unique
    # constraint + backfill legacy rows. Must run after create_all (which
    # builds the table on fresh deploys) and before any router handles a
    # request (the backfill happens in a separate session).
    await add_project_slug_if_missing()
    # Issue #149: supplier country_code + BOM currency_code columns on
    # existing tables (part_suppliers / project_bom_items).
    await add_supplier_country_if_missing()
    await add_bom_currency_if_missing()
    # Supplier-pricing feature: per-supplier unit_cost + currency_code on
    # part_suppliers, and supplier_id FK on project_bom_items. Must run
    # after ``add_supplier_country_if_missing`` so both column-additions
    # land in the same lifespan pass; ordering between them doesn't
    # matter (both helpers are independent ALTERs against different
    # columns on the same table).
    await add_supplier_pricing_if_missing()
    await add_bom_supplier_id_if_missing()
    # Issue #122 Phase 2: supplier link-health columns + lifecycle columns.
    await add_supplier_health_columns_if_missing()
    await add_part_status_columns_if_missing()
    # Issue #187: ensure project_files.description column exists on legacy
    # Postgres deployments. No-op on fresh deploys / SQLite tests.
    await add_project_file_description_if_missing()
    # B3 (issue #178): ensure instruction_steps.step_type / body /
    # video_url / schematic_id columns exist on legacy Postgres
    # deployments. No-op on fresh deploys / SQLite tests.
    await add_instruction_step_type_if_missing()
    # Editor content-surface toggle: ensure projects.content_mode column
    # exists on legacy Postgres deployments (defaults to 'markdown').
    await add_project_content_mode_if_missing()
    # Versioning migrations. CRITICAL ORDERING: every column-adding ALTER
    # must run BEFORE any backfill, because the backfills issue full-model
    # ORM SELECTs (e.g. ``select(Part)``) that reference EVERY mapped column
    # — including newly-added ones like ``parts.symbol_id``. If a backfill
    # runs before its column's ALTER, Postgres raises UndefinedColumnError
    # and startup aborts (SQLite tests don't catch this — create_all builds
    # all columns up front). So: ALTERs first, backfills second.
    #
    # --- ALTERs (idempotent column adds) ---
    await add_library_symbol_current_revision_if_missing()
    await add_library_symbol_fork_columns_if_missing()
    await add_library_symbol_power_port_if_missing()
    # Part↔symbol link: parts.symbol_id + part_revisions.symbol_id. After
    # create_all (which builds library_symbols) so the FK target exists.
    await add_part_symbol_id_if_missing()
    # Phase 3: project_bom_items.part_revision_id.
    await add_bom_part_revision_id_if_missing()
    #
    # --- Backfills (issue full-model SELECTs — must follow the ALTERs) ---
    await backfill_library_symbol_revisions()
    await backfill_part_revisions_if_missing()
    # Re-case legacy slug categories so the self-organising category
    # vocabulary doesn't fragment (selects Part → needs symbol_id present).
    await recanonicalize_part_categories()
    # Phase 3: pin existing BOM rows to their part's current revision. Runs
    # after backfill_part_revisions_if_missing so every part has a current
    # revision to pin to.
    await backfill_bom_part_revisions()
    # Migrate the legacy parts.image_url cover into part_photos (first photo)
    # so uploaded photos are the single image source. Postgres-only,
    # idempotent; the column stays dormant after the code stops using it.
    await backfill_part_image_url_to_photos()
    # Relocate images that were uploaded through the Files & Downloads section
    # before the upload handler learned to route images to the gallery. Runs
    # after add_project_file_description_if_missing (above) because it issues a
    # full-model select(ProjectFile).
    await migrate_image_files_to_images()
    # Issue #106: seed the badge catalog (idempotent upsert by slug).
    sessionmaker = get_sessionmaker()
    async with sessionmaker() as session:
        await seed_badge_definitions(session)

    # Issue #122 Phase 2: daily APScheduler job that walks every supplier
    # row and updates link-health columns. Skipped under pytest (the
    # PYTEST_CURRENT_TEST guard in ``background_jobs_disabled``) so the
    # test suite is hermetic and doesn't make outbound HTTP.
    scheduler: AsyncIOScheduler | None = None
    if not background_jobs_disabled():
        scheduler = AsyncIOScheduler()
        scheduler.add_job(
            run_full_health_check,
            trigger=IntervalTrigger(hours=24),
            id="projects-supplier-health",
            name="Daily supplier link health check",
            max_instances=1,
            coalesce=True,
            replace_existing=True,
        )
        scheduler.start()
        logger.info("scheduled: supplier link health check every 24 hours")
    try:
        yield
    finally:
        if scheduler is not None:
            scheduler.shutdown(wait=False)
            logger.info("supplier health scheduler stopped")


def create_app() -> FastAPI:
    app = FastAPI(
        title="Projects API",
        description="Project writeup service for kevsrobots.com",
        version="0.1.0",
        lifespan=lifespan,
    )
    settings = get_settings()
    app.add_middleware(
        CORSMiddleware,
        allow_origins=settings.cors_origins_list,
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )
    app.include_router(health.router)
    # Issue #139: /api/auth/me — login-state introspection for the frontend.
    app.include_router(auth.router)
    # Downloads router declares /api/projects/popular and must be registered
    # BEFORE projects.router so the more-specific path wins over the
    # catch-all /api/projects/{project_id}.
    app.include_router(downloads.router)
    # Same constraint applies to the featured router — it owns
    # /api/projects/featured and that path must beat /api/projects/{id}.
    app.include_router(featured.router)
    app.include_router(projects.router)
    # Issue #152: slug-based read endpoints. Mounted after projects.router
    # because none of the by-slug paths overlap with the id-based ones
    # (different segment counts) — ordering is for readability only.
    app.include_router(by_slug.router)
    # Social-card OG endpoint (GET /og/<owner>/<slug>). HTML output for
    # crawlers; sits outside the /api/ prefix because it's not JSON.
    app.include_router(social_og.router)
    app.include_router(bom.router)
    app.include_router(files.router)
    app.include_router(images.router)
    app.include_router(links.router)
    # Issue #171: YouTube videos as a first-class table, embedded above
    # the description on the public view page.
    app.include_router(videos.router)
    # Issue #178 Phase 0: build instructions (step-by-step outlines).
    # Phase 1 will layer a Fabric.js canvas on top of these rows.
    app.include_router(instructions.router)
    # Issue #178 Phase E2: per-project schematic (one schematic per
    # project for v1; opaque JSON blob stored in project_schematics).
    app.include_router(schematics.router)
    # Symbol Designer: per-project user-designed schematic symbols.
    # Augments the hard-coded library in the schematic editor (E2) at
    # runtime; one symbol can be linked to a BOM row via ``bom_item_id``
    # so it surfaces as a ``⌗`` chip in C1's asset drawer.
    app.include_router(symbols.router)
    # Admin-curated global symbol library — promoted from project
    # symbols so every project's schematic editor can use them.
    # Writes gated by ``require_admin`` (ADMIN_USERNAMES env var).
    app.include_router(library_symbols.router)
    app.include_router(journal.router)
    app.include_router(moderation.router)
    # Issue #123: parts catalog moderation (reports + community merges).
    # MUST be mounted BEFORE parts.router because that router's
    # ``GET /api/parts/{slug}`` would otherwise eat
    # ``/api/parts/merge-proposals`` (it'd resolve {slug}="merge-proposals"
    # and 404). Mounting the moderation routes first lets FastAPI match
    # the more-specific literal paths before the catch-all slug capture.
    app.include_router(parts_moderation.router)
    app.include_router(parts.router)
    app.include_router(parts_photos.router)
    # Issue #122 Phase 2: parts talk pages + admin recheck.
    app.include_router(parts_talk.router)
    app.include_router(admin_parts.router)
    app.include_router(makes.router)
    # Issue #108: project remixes (fork & attribution).
    app.include_router(remixes.router)
    # Issue #115: editorial staff-picks collections. The featured router
    # is already mounted above (before projects.router) so that
    # /api/projects/featured outranks the /api/projects/{id} catch-all.
    app.include_router(staff_picks.router)
    # Issue #140: user follows + badges + profile-page support.
    app.include_router(follows.router)
    # Issue #111: public user profiles + activity feed + follower lists.
    # Mounted after follows so this router's wider /api/users/... surface
    # coexists with the follow toggle endpoints.
    app.include_router(users.router)
    # Issue #106: badges & achievements.
    app.include_router(badges.router)
    # Issue #138: user feedback widget + admin inbox.
    app.include_router(feedback.router)
    app.include_router(admin_feedback.router)
    # Issue #150: public FX conversion endpoint.
    app.include_router(fx.router)
    return app


app = create_app()
