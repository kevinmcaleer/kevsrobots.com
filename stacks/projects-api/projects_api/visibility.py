"""Draft visibility (issue #260).

A project whose ``status`` is ``draft`` is private to its author: other
users (and anonymous callers) get a 404 for the project and its
read-only sub-resources, and it never appears in the hub, search,
profiles, featured lists or activity feeds. ``wip`` and ``completed``
are both public; ``archived`` keeps its existing meaning (hidden from
listings, still reachable by direct link).
"""

from __future__ import annotations

from typing import Optional

from fastapi import HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession

from .models import Project

DRAFT = "draft"


def is_draft(project: Project) -> bool:
    return project.status == DRAFT


def can_view(project: Project, user: Optional[str]) -> bool:
    """True unless ``project`` is a draft and ``user`` isn't its author."""
    return not is_draft(project) or (user is not None and user == project.author_username)


def not_draft():
    """SQL filter that leaves drafts out of public listings."""
    return Project.status != DRAFT


# For a picture that a draft can hide: nothing may keep it, neither the
# browser nor Cloudflare. Cloudflare caches .svg/.png URLs and gives a
# response without Cache-Control a 4 hour browser TTL, so a stranger's 404
# for a draft's diagram (or an image tag sent without the dev token) stuck
# for hours, even after the project went public; and a public 200 could
# hand a draft's picture to the next caller.
NO_STORE = {"Cache-Control": "private, no-store"}


async def hide_drafts(
    session: AsyncSession, project_id: int, user: Optional[str], *, no_store: bool = False
) -> bool:
    """404 when ``project_id`` is someone else's draft.

    A missing project is left to the caller, so the sub-resource routes
    keep whatever they already did for unknown ids. Returns whether the
    project is a draft (which its author is allowed to see). With
    ``no_store`` the 404 tells caches not to keep it.
    """
    project = await session.get(Project, project_id)
    if project is not None and not can_view(project, user):
        raise HTTPException(
            status.HTTP_404_NOT_FOUND, detail="Project not found", headers=NO_STORE if no_store else None
        )
    return project is not None and is_draft(project)
