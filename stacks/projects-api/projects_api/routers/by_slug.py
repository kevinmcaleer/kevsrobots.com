"""Slug-based read endpoints (issue #152).

Adds a parallel API surface that addresses projects by their
``(author_username, slug)`` pair instead of their integer ``id``. The
goal is twofold:

* **SEO-friendly URLs** — ``/projects/<owner>/<slug>`` reads naturally
  and shares cleanly.
* **Harder to enumerate** — sequential id scraping by abuse bots is no
  longer enough to walk the catalogue.

We deliberately keep the id-based endpoints (``GET /api/projects/{id}``,
``/api/projects/{id}/bom`` etc.) in place for backward compatibility —
the editor and a few internal admin pages know the id once a project
loads and POST/PUT/DELETE through them. New read paths in this module
all funnel through :func:`resolve_project_id` so the resolution rules
stay in one place.

**Write surface** (POST / PUT / DELETE on a project's nested resources)
stays id-based intentionally: the editor knows the id from the moment
it loads a project, the by-slug URL only matters for the public view.
Mounting writes on a slug URL would also force a rename to invalidate
in-flight requests; sticking with the id avoids the foot-gun.
"""

from __future__ import annotations

from typing import Optional

from fastapi import APIRouter, Depends
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import get_optional_user
from ..db import get_session
from ..schemas import (
    BOMItemResponse,
    FileResponse,
    ImageResponse,
    JournalEntryResponse,
    LinkResponse,
    MakeResponse,
    ProjectResponse,
)
from . import bom as bom_router_module
from . import files as files_router_module
from . import images as images_router_module
from . import journal as journal_router_module
from . import links as links_router_module
from . import makes as makes_router_module
from . import projects as projects_router_module
from .projects import resolve_project_id

router = APIRouter(prefix="/api/projects", tags=["projects-by-slug"])


# ---- Canonical lookup ----------------------------------------------------


@router.get("/by-slug/{owner}/{slug}", response_model=ProjectResponse)
async def get_project_by_slug(
    owner: str,
    slug: str,
    user: Optional[str] = Depends(get_optional_user),
    session: AsyncSession = Depends(get_session),
) -> ProjectResponse:
    """Look up a project by ``(owner, slug)``.

    Returned shape matches the id-based ``GET /api/projects/{id}`` so
    the frontend can call either endpoint and converge on the same
    rendering code.
    """
    project_id = await resolve_project_id(session, owner, slug)
    return await projects_router_module.get_project(
        project_id=project_id, user=user, session=session
    )


# ---- Nested read resources ----------------------------------------------
#
# Each handler resolves the (owner, slug) to a numeric id and forwards
# to the existing id-based handler so the response shape and behaviour
# stay 1:1.


@router.get("/{owner}/{slug}/bom", response_model=list[BOMItemResponse])
async def list_bom_by_slug(
    owner: str,
    slug: str,
    session: AsyncSession = Depends(get_session),
) -> list[BOMItemResponse]:
    project_id = await resolve_project_id(session, owner, slug)
    return await bom_router_module.list_bom(
        project_id=project_id, session=session
    )


@router.get("/{owner}/{slug}/images", response_model=list[ImageResponse])
async def list_images_by_slug(
    owner: str,
    slug: str,
    session: AsyncSession = Depends(get_session),
) -> list[ImageResponse]:
    project_id = await resolve_project_id(session, owner, slug)
    return await images_router_module.list_images(
        project_id=project_id, session=session
    )


@router.get("/{owner}/{slug}/files", response_model=list[FileResponse])
async def list_files_by_slug(
    owner: str,
    slug: str,
    session: AsyncSession = Depends(get_session),
) -> list[FileResponse]:
    project_id = await resolve_project_id(session, owner, slug)
    return await files_router_module.list_files(
        project_id=project_id, session=session
    )


@router.get("/{owner}/{slug}/links", response_model=list[LinkResponse])
async def list_links_by_slug(
    owner: str,
    slug: str,
    session: AsyncSession = Depends(get_session),
) -> list[LinkResponse]:
    project_id = await resolve_project_id(session, owner, slug)
    return await links_router_module.list_links(
        project_id=project_id, session=session
    )


@router.get(
    "/{owner}/{slug}/journal",
    response_model=list[JournalEntryResponse],
)
async def list_journal_by_slug(
    owner: str,
    slug: str,
    session: AsyncSession = Depends(get_session),
) -> list[JournalEntryResponse]:
    project_id = await resolve_project_id(session, owner, slug)
    return await journal_router_module.list_entries(
        project_id=project_id, session=session
    )


@router.get("/{owner}/{slug}/makes", response_model=list[MakeResponse])
async def list_makes_by_slug(
    owner: str,
    slug: str,
    session: AsyncSession = Depends(get_session),
) -> list[MakeResponse]:
    project_id = await resolve_project_id(session, owner, slug)
    return await makes_router_module.list_makes_for_project(
        project_id=project_id, session=session
    )
