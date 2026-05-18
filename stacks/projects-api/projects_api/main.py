"""FastAPI application entrypoint for the Projects API."""

from __future__ import annotations

from contextlib import asynccontextmanager
from typing import AsyncIterator

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from .badges import seed_badge_definitions
from .config import get_settings
from .db import (
    add_bom_part_id_if_missing,
    add_remix_columns_if_missing,
    add_user_profile_columns_if_missing,
    create_all,
    get_sessionmaker,
    repair_stale_fks,
)
from .routers import (
    auth,
    badges,
    bom,
    downloads,
    files,
    follows,
    health,
    images,
    journal,
    links,
    makes,
    moderation,
    parts,
    projects,
    remixes,
    users,
)


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncIterator[None]:
    await create_all()
    await repair_stale_fks()
    # Issue #108: ensure remix columns exist on legacy Postgres deployments.
    await add_remix_columns_if_missing()
    # Issue #121: ensure project_bom_items.part_id column exists.
    await add_bom_part_id_if_missing()
    # Issue #111: defensive ALTER for the new user_profiles table —
    # no-op on fresh deploys where create_all built every column.
    await add_user_profile_columns_if_missing()
    # Issue #106: seed the badge catalog (idempotent upsert by slug).
    sessionmaker = get_sessionmaker()
    async with sessionmaker() as session:
        await seed_badge_definitions(session)
    yield


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
    app.include_router(projects.router)
    app.include_router(bom.router)
    app.include_router(files.router)
    app.include_router(images.router)
    app.include_router(links.router)
    app.include_router(journal.router)
    app.include_router(moderation.router)
    app.include_router(parts.router)
    app.include_router(makes.router)
    # Issue #108: project remixes (fork & attribution).
    app.include_router(remixes.router)
    # Issue #140: user follows + badges + profile-page support.
    app.include_router(follows.router)
    # Issue #111: public user profiles + activity feed + follower lists.
    # Mounted after follows so this router's wider /api/users/... surface
    # coexists with the follow toggle endpoints.
    app.include_router(users.router)
    # Issue #106: badges & achievements.
    app.include_router(badges.router)
    return app


app = create_app()
