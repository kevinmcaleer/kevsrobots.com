"""Open Graph / Twitter Card endpoint for project view pages.

Project pages on the public site live at ``/projects/<owner>/<slug>`` and
are fully rendered client-side: a static ``view.html`` boots up, fetches
project JSON from this API, and stitches the DOM together. That works
fine for humans but not for social-card crawlers (Twitterbot, Slackbot,
facebookexternalhit, LinkedInBot, Discordbot, WhatsApp, TelegramBot,
Mastodon, Iframely, Embedly, …) — none of them execute JavaScript, so
the link unfurl falls back to whatever static ``<head>`` the SPA shell
ships with (currently the 404 redirector's "Page Not Found" preview).

This module ships a tiny HTML-only endpoint that returns just the
relevant OG / Twitter meta tags for a single project. A Cloudflare
Worker sits in front of the main site, sniffs the User-Agent on
``/projects/<owner>/<slug>``, and serves the response from here when
the requester is a known crawler. Humans never see it.

Notes:

* The route lives at ``/og/<owner>/<slug>`` — deliberately **not** under
  ``/api/`` because the response is ``text/html``, not JSON.
* No auth: the OG card surface is fully public.
* No schema migration is needed; this endpoint only reads existing
  columns on the ``projects`` table.
* Cache headers are short (300s browser, 600s edge) so a project edit
  shows up in reshared previews within ~10 minutes.
"""

from __future__ import annotations

import html
from typing import Optional

from fastapi import APIRouter, Depends, HTTPException
from fastapi.responses import Response
from sqlalchemy.ext.asyncio import AsyncSession

from ..db import get_session
from ..models import Project
from .projects import _get_tags, resolve_project_id

router = APIRouter(tags=["social-og"])

# Cover-image fallback used when a project has no ``cover_image`` value.
# The file itself can be added to the static site later — the URL just
# needs to be stable so crawlers always resolve to something.
DEFAULT_OG_IMAGE = (
    "https://www.kevsrobots.com/assets/img/projects/projects-hub-og.jpg"
)

# Host that serves user-uploaded images. Relative ``cover_image`` paths
# (e.g. ``/uploads/abc.jpg``) are absolutised against this host so the
# resulting OG URL is valid in any social-card preview.
UPLOAD_HOST = "https://projects.kevsrobots.com"

# Short cache: 5 min browser, 10 min at the edge — long enough that the
# crawler never hammers the API, short enough that an edited project's
# preview refreshes within a single Cloudflare TTL.
CACHE_HEADERS = {"Cache-Control": "public, max-age=300, s-maxage=600"}


def _absolutise_cover(cover_image: Optional[str]) -> str:
    """Return an absolute URL for the OG image."""
    if not cover_image:
        return DEFAULT_OG_IMAGE
    if cover_image.startswith("http://") or cover_image.startswith("https://"):
        return cover_image
    if cover_image.startswith("/"):
        return f"{UPLOAD_HOST}{cover_image}"
    # Bare filename — treat as an uploads-host relative path.
    return f"{UPLOAD_HOST}/{cover_image}"


def _esc(value: Optional[str]) -> str:
    """HTML-escape including quotes (safe for attribute values)."""
    return html.escape(value or "", quote=True)


def render_og_html(project: Project, tags: list[str]) -> str:
    """Build the OG / Twitter card HTML for one project."""
    page_url = (
        "https://www.kevsrobots.com/projects/"
        f"{project.author_username}/{project.slug}"
    )
    title = project.title or "Untitled Project"
    description = project.short_description or (
        f"A Projects Hub build by @{project.author_username} on KevsRobots."
    )
    image = _absolutise_cover(project.cover_image)
    created_iso = (
        project.created_at.isoformat() if project.created_at is not None else ""
    )

    title_esc = _esc(title)
    description_esc = _esc(description)
    image_esc = _esc(image)
    page_url_esc = _esc(page_url)
    author_esc = _esc(project.author_username)
    created_esc = _esc(created_iso)

    tag_meta_lines = "\n  ".join(
        f'<meta property="article:tag" content="{_esc(tag)}">'
        for tag in tags
    )
    if tag_meta_lines:
        tag_meta_lines = "  " + tag_meta_lines + "\n"

    return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>{title_esc} — KevsRobots Projects</title>
  <meta name="description" content="{description_esc}">

  <meta property="og:type" content="article">
  <meta property="og:site_name" content="KevsRobots">
  <meta property="og:title" content="{title_esc}">
  <meta property="og:description" content="{description_esc}">
  <meta property="og:image" content="{image_esc}">
  <meta property="og:url" content="{page_url_esc}">
  <meta property="article:author" content="{author_esc}">
  <meta property="article:published_time" content="{created_esc}">
{tag_meta_lines}
  <meta name="twitter:card" content="summary_large_image">
  <meta name="twitter:site" content="@kevsmac">
  <meta name="twitter:title" content="{title_esc}">
  <meta name="twitter:description" content="{description_esc}">
  <meta name="twitter:image" content="{image_esc}">

  <link rel="canonical" href="{page_url_esc}">
  <meta http-equiv="refresh" content="0; url={page_url_esc}">
</head>
<body>
  <p>Redirecting to <a href="{page_url_esc}">{title_esc}</a>…</p>
</body>
</html>
"""


def _render_not_found_html() -> str:
    """Generic OG card returned when the requested project doesn't exist.

    Still HTML (not JSON) so a crawler that follows a stale share-link
    gets a usable unfurl pointing at the Projects Hub instead of an
    error preview.
    """
    hub_url = "https://www.kevsrobots.com/projects/hub"
    title = "KevsRobots Projects Hub"
    description = (
        "Browse community-built robotics, electronics and maker projects "
        "on KevsRobots."
    )
    image = DEFAULT_OG_IMAGE
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>{title} — Project not found</title>
  <meta name="description" content="{description}">

  <meta property="og:type" content="website">
  <meta property="og:site_name" content="KevsRobots">
  <meta property="og:title" content="{title}">
  <meta property="og:description" content="{description}">
  <meta property="og:image" content="{image}">
  <meta property="og:url" content="{hub_url}">

  <meta name="twitter:card" content="summary_large_image">
  <meta name="twitter:site" content="@kevsmac">
  <meta name="twitter:title" content="{title}">
  <meta name="twitter:description" content="{description}">
  <meta name="twitter:image" content="{image}">

  <link rel="canonical" href="{hub_url}">
  <meta http-equiv="refresh" content="0; url={hub_url}">
</head>
<body>
  <p>Project not found. <a href="{hub_url}">Browse the Projects Hub</a>.</p>
</body>
</html>
"""


@router.get("/og/{owner}/{slug}")
async def project_og_card(
    owner: str,
    slug: str,
    session: AsyncSession = Depends(get_session),
) -> Response:
    """Return crawler-friendly OG / Twitter card HTML for one project.

    Always responds with ``text/html``. Returns 404 (with a generic hub
    card body) when the project can't be resolved so a stale share-link
    still produces a non-broken preview.
    """
    try:
        project_id = await resolve_project_id(session, owner, slug)
    except HTTPException as exc:
        if exc.status_code == 404:
            return Response(
                content=_render_not_found_html(),
                status_code=404,
                media_type="text/html; charset=utf-8",
                headers=CACHE_HEADERS,
            )
        raise

    project = await session.get(Project, project_id)
    if project is None:  # pragma: no cover - resolve_project_id just found it
        return Response(
            content=_render_not_found_html(),
            status_code=404,
            media_type="text/html; charset=utf-8",
            headers=CACHE_HEADERS,
        )

    tags = await _get_tags(session, project_id)
    html_body = render_og_html(project, tags)
    return Response(
        content=html_body,
        status_code=200,
        media_type="text/html; charset=utf-8",
        headers=CACHE_HEADERS,
    )
