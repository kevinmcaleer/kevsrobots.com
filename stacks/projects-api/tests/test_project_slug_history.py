"""Tests for the issue #190 rename-syncs-slug behaviour.

When a project owner changes the title:

* The slug regenerates from the new title.
* The OLD slug is preserved in ``project_slug_history`` so inbound links
  keep resolving.
* The by-slug GET endpoint returns a 301 redirect from the historical
  slug to the canonical current slug.

These tests live in a sibling file to ``test_project_slugs.py`` (which
covers the issue #152 create-time + structural slug behaviour) so the
two feature areas can evolve independently.
"""

from __future__ import annotations

import pytest
from sqlalchemy import select

from projects_api.models import ProjectSlugHistory

from .conftest import make_auth_header


# ---- Spaces-in-title → dash separators (issue #190 explicit requirement) -


@pytest.mark.asyncio
async def test_spaces_in_title_become_dashes_in_slug(client) -> None:
    """Locks the "replace spaces with `-`" requirement from issue #190."""
    response = await client.post(
        "/api/projects",
        json={"title": "Hello World"},
        headers=make_auth_header(),
    )
    assert response.status_code == 201
    assert response.json()["slug"] == "hello-world"


# ---- Title change auto-regenerates slug + writes history ----------------


@pytest.mark.asyncio
async def test_title_change_writes_history_and_updates_slug(
    client, session
) -> None:
    headers = make_auth_header()
    created = await client.post(
        "/api/projects",
        json={"title": "BurgerBot 2"},
        headers=headers,
    )
    pid = created.json()["id"]
    assert created.json()["slug"] == "burgerbot-2"

    response = await client.put(
        f"/api/projects/{pid}",
        json={"title": "BurgerBot 3000"},
        headers=headers,
    )
    assert response.status_code == 200
    assert response.json()["slug"] == "burgerbot-3000"

    rows = (
        await session.execute(
            select(ProjectSlugHistory).where(
                ProjectSlugHistory.project_id == pid
            )
        )
    ).scalars().all()
    assert len(rows) == 1
    assert rows[0].slug == "burgerbot-2"


# ---- No-op when the computed slug doesn't change ------------------------


@pytest.mark.asyncio
async def test_title_edit_that_yields_same_slug_writes_no_history(
    client, session
) -> None:
    """Whitespace-only or case-only title edits don't pollute history."""
    headers = make_auth_header()
    created = await client.post(
        "/api/projects",
        json={"title": "Hello World"},
        headers=headers,
    )
    pid = created.json()["id"]

    # Trailing whitespace + a different case → same slug.
    response = await client.put(
        f"/api/projects/{pid}",
        json={"title": "  HELLO   World  "},
        headers=headers,
    )
    assert response.status_code == 200
    assert response.json()["slug"] == "hello-world"

    rows = (
        await session.execute(
            select(ProjectSlugHistory).where(
                ProjectSlugHistory.project_id == pid
            )
        )
    ).scalars().all()
    assert rows == []


# ---- Rename to a colliding slug must NOT count the project itself -------


@pytest.mark.asyncio
async def test_rename_does_not_collide_with_own_current_slug(client) -> None:
    """If the rename's new slug equals the project's own current slug, it
    must NOT be bumped to `-2`. (Regression guard for the
    ``exclude_project_id`` plumbing through ``unique_slug_for_author``.)
    """
    headers = make_auth_header()
    created = await client.post(
        "/api/projects",
        json={"title": "My Project"},
        headers=headers,
    )
    pid = created.json()["id"]
    assert created.json()["slug"] == "my-project"

    # Same title typed again (no actual change) — the in-place rename path
    # must not bump itself.
    response = await client.put(
        f"/api/projects/{pid}",
        json={"title": "My Project"},
        headers=headers,
    )
    assert response.status_code == 200
    assert response.json()["slug"] == "my-project"


@pytest.mark.asyncio
async def test_rename_collides_with_other_project_gets_suffix(client) -> None:
    """Renaming to match an existing sibling's title bumps to ``-2``."""
    headers = make_auth_header()
    other = await client.post(
        "/api/projects",
        json={"title": "Taken Title"},
        headers=headers,
    )
    assert other.json()["slug"] == "taken-title"

    me = await client.post(
        "/api/projects",
        json={"title": "Original"},
        headers=headers,
    )
    me_id = me.json()["id"]

    response = await client.put(
        f"/api/projects/{me_id}",
        json={"title": "Taken Title"},
        headers=headers,
    )
    assert response.status_code == 200
    assert response.json()["slug"] == "taken-title-2"


# ---- Old slug → 301 to canonical URL ------------------------------------


@pytest.mark.asyncio
async def test_old_slug_redirects_to_canonical(client) -> None:
    headers = make_auth_header()  # testuser
    created = await client.post(
        "/api/projects",
        json={"title": "Initial Name"},
        headers=headers,
    )
    pid = created.json()["id"]

    await client.put(
        f"/api/projects/{pid}",
        json={"title": "Renamed Project"},
        headers=headers,
    )

    # GET on the historical slug returns 301 to the canonical one.
    response = await client.get(
        "/api/projects/by-slug/testuser/initial-name",
        follow_redirects=False,
    )
    assert response.status_code == 301
    assert (
        response.headers["location"]
        == "/api/projects/by-slug/testuser/renamed-project"
    )


@pytest.mark.asyncio
async def test_current_slug_does_not_redirect(client) -> None:
    """Sanity check: the new canonical URL serves a 200, not a redirect
    loop."""
    headers = make_auth_header()
    created = await client.post(
        "/api/projects",
        json={"title": "Stable Name"},
        headers=headers,
    )
    await client.put(
        f"/api/projects/{created.json()['id']}",
        json={"title": "Now Different"},
        headers=headers,
    )

    response = await client.get(
        "/api/projects/by-slug/testuser/now-different",
        follow_redirects=False,
    )
    assert response.status_code == 200
    assert response.json()["slug"] == "now-different"


# ---- Renaming back to a previous title ----------------------------------


@pytest.mark.asyncio
async def test_rename_back_to_previous_title_writes_new_history_row(
    client, session
) -> None:
    """A → B → A leaves two history rows (one per retirement) and the
    project ends up back on slug A. (Titles padded to ProjectCreate's
    5-char minimum.)"""
    headers = make_auth_header()
    created = await client.post(
        "/api/projects",
        json={"title": "AlphaBot"},
        headers=headers,
    )
    pid = created.json()["id"]
    assert created.json()["slug"] == "alphabot"

    await client.put(
        f"/api/projects/{pid}",
        json={"title": "BetaBot"},
        headers=headers,
    )
    final = await client.put(
        f"/api/projects/{pid}",
        json={"title": "AlphaBot"},
        headers=headers,
    )
    assert final.json()["slug"] == "alphabot"

    rows = (
        await session.execute(
            select(ProjectSlugHistory)
            .where(ProjectSlugHistory.project_id == pid)
            .order_by(ProjectSlugHistory.id)
        )
    ).scalars().all()
    retired = [r.slug for r in rows]
    # Both transitions were captured: alphabot→betabot retired
    # "alphabot", and betabot→alphabot retired "betabot". Order of
    # insertion is preserved.
    assert retired == ["alphabot", "betabot"]


# ---- Non-owner can't trigger a rename -----------------------------------


@pytest.mark.asyncio
async def test_non_owner_rename_403_does_not_change_slug(client, session) -> None:
    owner = make_auth_header("alice")
    intruder = make_auth_header("bob")

    created = await client.post(
        "/api/projects",
        json={"title": "Owned By Alice"},
        headers=owner,
    )
    pid = created.json()["id"]
    original_slug = created.json()["slug"]

    response = await client.put(
        f"/api/projects/{pid}",
        json={"title": "Hijacked"},
        headers=intruder,
    )
    assert response.status_code == 403

    fetched = await client.get(f"/api/projects/{pid}")
    assert fetched.json()["slug"] == original_slug

    rows = (
        await session.execute(
            select(ProjectSlugHistory).where(
                ProjectSlugHistory.project_id == pid
            )
        )
    ).scalars().all()
    assert rows == []


# ---- Nested by-slug endpoints still resolve via history -----------------


@pytest.mark.asyncio
async def test_nested_endpoints_resolve_via_historical_slug(client) -> None:
    """The machine-callers (BOM etc.) don't redirect, but they still
    resolve the project so the existing /projects/<owner>/<slug>/<...>
    endpoints don't die the moment the owner renames."""
    headers = make_auth_header()  # testuser
    created = await client.post(
        "/api/projects",
        json={"title": "Has BOM"},
        headers=headers,
    )
    pid = created.json()["id"]
    await client.post(
        f"/api/projects/{pid}/bom",
        json={"name": "Servo", "quantity": 2},
        headers=headers,
    )

    await client.put(
        f"/api/projects/{pid}",
        json={"title": "Renamed BOM"},
        headers=headers,
    )

    # Old-slug nested fetch silently serves the project (no redirect).
    response = await client.get("/api/projects/testuser/has-bom/bom")
    assert response.status_code == 200
    assert response.json()[0]["name"] == "Servo"
