"""Project slug helpers (issue #152).

A project slug is a URL-friendly form of the project title used in the
canonical ``/projects/<owner>/<slug>`` URL scheme. Slugs are:

* lowercase ASCII only
* spaces / underscores replaced by ``-``
* anything outside ``[a-z0-9-]`` stripped
* repeated dashes collapsed to one
* trimmed to at most ``MAX_LEN`` chars, with no leading / trailing dash
* unique **per author** (``UniqueConstraint(author_username, slug)``)

When a generated slug collides with another row owned by the same author,
:func:`unique_slug_for_author` appends a numeric suffix (``-2``, ``-3``, …)
until it finds a free slot.

This module is the single source of truth for slug generation so create,
update, and backfill paths can't drift apart.
"""

from __future__ import annotations

import re
import unicodedata

from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

# 80 chars matches the column width on Project.slug. We trim a little
# shorter than that so suffix variants like "-23" still fit.
MAX_LEN = 80
_SUFFIX_HEADROOM = 6  # leave room for "-9999"

_INVALID = re.compile(r"[^a-z0-9-]+")
_REPEATED_DASH = re.compile(r"-{2,}")


def slugify_title(title: str) -> str:
    """Convert a title to a URL-safe slug.

    Always returns a non-empty string — when the input collapses to the
    empty string (e.g. only emoji / CJK characters that ASCII-fold away),
    we fall back to ``"project"`` so the caller can still build a URL.
    """
    if not title:
        return "project"
    # NFKD-fold + drop combining marks gives us reasonable ASCII for most
    # Latin-derived titles without pulling in a transliteration library.
    normalised = unicodedata.normalize("NFKD", title)
    ascii_bytes = normalised.encode("ascii", "ignore")
    lowered = ascii_bytes.decode("ascii").lower()
    # Replace whitespace / underscores with dashes before stripping.
    lowered = re.sub(r"[\s_]+", "-", lowered)
    cleaned = _INVALID.sub("", lowered)
    cleaned = _REPEATED_DASH.sub("-", cleaned).strip("-")
    if not cleaned:
        return "project"
    return cleaned[: MAX_LEN - _SUFFIX_HEADROOM] or "project"


async def unique_slug_for_author(
    session: AsyncSession,
    *,
    author_username: str,
    base_slug: str,
    exclude_project_id: int | None = None,
) -> str:
    """Return a slug that doesn't collide with any existing project owned
    by ``author_username``.

    The conflict-resolution algorithm is:

    1. Try ``base_slug`` itself.
    2. On collision, append ``-2``, ``-3``, … (numeric, starting at 2)
       and probe each candidate until one is free.

    The probe is a serial query loop rather than a single ``LIKE`` scan so
    the caller doesn't have to reason about gaps left by deleted rows.
    Two concurrent writers can still race here — the
    :class:`UniqueConstraint` on ``(author_username, slug)`` is the
    authoritative tiebreaker (the loser sees an IntegrityError and the
    caller is expected to retry with a fresh probe).

    ``exclude_project_id`` lets callers update an existing project without
    matching themselves.
    """
    # Local import — defer to avoid models <-> slugs import cycle.
    from .models import Project

    base = (base_slug or "").strip("-") or "project"
    # Defensive cap; the column is 80 chars and we want headroom for suffixes.
    base = base[: MAX_LEN - _SUFFIX_HEADROOM]

    candidate = base
    suffix = 1
    while True:
        query = select(Project.id).where(
            Project.author_username == author_username,
            Project.slug == candidate,
        )
        if exclude_project_id is not None:
            query = query.where(Project.id != exclude_project_id)
        existing = await session.scalar(query)
        if existing is None:
            return candidate
        suffix += 1
        candidate = f"{base}-{suffix}"
        # Final defensive cap — never emit a slug longer than the column.
        if len(candidate) > MAX_LEN:
            # Trim base further to keep total length within bounds.
            head = base[: MAX_LEN - (len(candidate) - len(base))]
            candidate = f"{head}-{suffix}"
