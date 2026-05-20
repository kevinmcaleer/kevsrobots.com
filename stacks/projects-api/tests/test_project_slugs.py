"""Tests for project slugs / by-slug URL routing (issue #152)."""

from __future__ import annotations

import pytest

from projects_api.slugs import slugify_title

from .conftest import make_auth_header


# ---- Pure-function slugify behaviour -----------------------------------


def test_slugify_basic_title() -> None:
    assert slugify_title("My Test Robot") == "my-test-robot"


def test_slugify_collapses_repeats_and_strips_punctuation() -> None:
    assert slugify_title("  Hello !! World!  ") == "hello-world"


def test_slugify_handles_unicode_fallback() -> None:
    # Emoji-only / non-ASCII titles fall back to "project" so a URL can
    # still be built without a 500.
    assert slugify_title("🚀🚀🚀") == "project"
    assert slugify_title("") == "project"


def test_slugify_caps_length() -> None:
    long = "abc" * 100
    out = slugify_title(long)
    # MAX_LEN is 80 with 6-char suffix headroom, so the base never exceeds 74.
    assert len(out) <= 74
    assert out.startswith("abc")


# ---- Create / list / by-slug fetch -------------------------------------


@pytest.mark.asyncio
async def test_create_project_auto_generates_slug(client) -> None:
    response = await client.post(
        "/api/projects",
        json={"title": "My Test Robot", "short_description": "Cool"},
        headers=make_auth_header(),
    )
    assert response.status_code == 201
    body = response.json()
    assert body["slug"] == "my-test-robot"


@pytest.mark.asyncio
async def test_list_projects_includes_slug(client) -> None:
    headers = make_auth_header()
    await client.post(
        "/api/projects",
        json={"title": "Slug Test Project"},
        headers=headers,
    )
    response = await client.get("/api/projects")
    assert response.status_code == 200
    items = response.json()
    assert any(p.get("slug") == "slug-test-project" for p in items)


@pytest.mark.asyncio
async def test_duplicate_title_gets_suffix_per_author(client) -> None:
    headers = make_auth_header()  # testuser
    first = await client.post(
        "/api/projects",
        json={"title": "Same Title"},
        headers=headers,
    )
    second = await client.post(
        "/api/projects",
        json={"title": "Same Title"},
        headers=headers,
    )
    third = await client.post(
        "/api/projects",
        json={"title": "Same Title"},
        headers=headers,
    )
    assert first.json()["slug"] == "same-title"
    assert second.json()["slug"] == "same-title-2"
    assert third.json()["slug"] == "same-title-3"


@pytest.mark.asyncio
async def test_same_slug_allowed_across_authors(client) -> None:
    # Two different users can both have a "same-title" slug — uniqueness
    # is per-author, not global.
    a = await client.post(
        "/api/projects",
        json={"title": "Shared Title"},
        headers=make_auth_header("alice"),
    )
    b = await client.post(
        "/api/projects",
        json={"title": "Shared Title"},
        headers=make_auth_header("bob"),
    )
    assert a.json()["slug"] == "shared-title"
    assert b.json()["slug"] == "shared-title"


@pytest.mark.asyncio
async def test_get_by_slug_returns_project(client) -> None:
    headers = make_auth_header()  # testuser
    created = await client.post(
        "/api/projects",
        json={"title": "Lookup Me", "short_description": "hi"},
        headers=headers,
    )
    pid = created.json()["id"]
    response = await client.get("/api/projects/by-slug/testuser/lookup-me")
    assert response.status_code == 200
    body = response.json()
    assert body["id"] == pid
    assert body["slug"] == "lookup-me"


@pytest.mark.asyncio
async def test_get_by_slug_404_for_unknown(client) -> None:
    response = await client.get("/api/projects/by-slug/nobody/nope")
    assert response.status_code == 404


# ---- Update / slug regeneration ----------------------------------------


@pytest.mark.asyncio
async def test_update_title_auto_regenerates_slug(client) -> None:
    """Issue #190: a title change automatically refreshes the slug.

    Inbound links to the old slug still work via the
    ``project_slug_history`` fallback + 301 redirect — see
    ``test_old_slug_redirects_to_canonical`` below.
    """
    headers = make_auth_header()
    created = await client.post(
        "/api/projects",
        json={"title": "Old Title"},
        headers=headers,
    )
    pid = created.json()["id"]
    old_slug = created.json()["slug"]
    assert old_slug == "old-title"

    response = await client.put(
        f"/api/projects/{pid}",
        json={"title": "Brand New Title"},
        headers=headers,
    )
    assert response.status_code == 200
    assert response.json()["slug"] == "brand-new-title"


@pytest.mark.asyncio
async def test_update_with_regenerate_slug_updates(client) -> None:
    headers = make_auth_header()
    created = await client.post(
        "/api/projects",
        json={"title": "First Title"},
        headers=headers,
    )
    pid = created.json()["id"]

    response = await client.put(
        f"/api/projects/{pid}",
        json={"title": "Refreshed Title", "regenerate_slug": True},
        headers=headers,
    )
    assert response.status_code == 200
    assert response.json()["slug"] == "refreshed-title"


# ---- Nested by-slug read endpoints -------------------------------------


@pytest.mark.asyncio
async def test_by_slug_nested_endpoints_match_id_endpoints(client) -> None:
    """The slug-based BOM / images / files / journal endpoints return the
    same shape as the id-based equivalents."""
    headers = make_auth_header()  # testuser
    created = await client.post(
        "/api/projects",
        json={"title": "Nested Slug"},
        headers=headers,
    )
    pid = created.json()["id"]
    slug = created.json()["slug"]

    # Add one of each nested resource via the id-based endpoints.
    await client.post(
        f"/api/projects/{pid}/bom",
        json={"name": "Servo", "quantity": 2},
        headers=headers,
    )
    await client.post(
        f"/api/projects/{pid}/links",
        json={"title": "Datasheet", "url": "https://example.com/ds.pdf"},
        headers=headers,
    )
    await client.post(
        f"/api/projects/{pid}/journal",
        json={"title": "Day 1", "status": "in_progress"},
        headers=headers,
    )

    # by-slug fetches return the same data.
    bom = await client.get(f"/api/projects/testuser/{slug}/bom")
    links = await client.get(f"/api/projects/testuser/{slug}/links")
    journal = await client.get(f"/api/projects/testuser/{slug}/journal")
    assert bom.status_code == 200
    assert links.status_code == 200
    assert journal.status_code == 200
    assert len(bom.json()) == 1
    assert bom.json()[0]["name"] == "Servo"
    assert len(links.json()) == 1
    assert links.json()[0]["title"] == "Datasheet"
    assert len(journal.json()) == 1
    assert journal.json()[0]["title"] == "Day 1"


@pytest.mark.asyncio
async def test_by_slug_nested_404_for_unknown_project(client) -> None:
    response = await client.get("/api/projects/nobody/missing/bom")
    assert response.status_code == 404


# ---- Backfill idempotency ----------------------------------------------


@pytest.mark.asyncio
async def test_backfill_is_idempotent_for_legacy_null_slugs(session) -> None:
    """``_backfill_project_slugs`` writes a slug only when one is missing;
    re-running it leaves populated rows alone."""
    from projects_api import db as db_module
    from projects_api.models import Project

    # Insert a row with slug=None (simulating a legacy row pre-#152).
    legacy = Project(
        title="Legacy One",
        slug=None,
        author_username="alice",
        status="wip",
    )
    session.add(legacy)
    await session.commit()
    await session.refresh(legacy)
    assert legacy.slug is None

    # Patch the sessionmaker so the backfill uses our test session's bind.
    bind = session.bind
    from sqlalchemy.ext.asyncio import async_sessionmaker, AsyncSession
    test_sm = async_sessionmaker(bind=bind, expire_on_commit=False, class_=AsyncSession)
    original = db_module.get_sessionmaker
    db_module.get_sessionmaker = lambda: test_sm
    try:
        await db_module._backfill_project_slugs()
        await db_module._backfill_project_slugs()  # second run = no-op
    finally:
        db_module.get_sessionmaker = original

    # Reload from the underlying DB so we don't read the stale session cache.
    async with test_sm() as fresh:
        reloaded = await fresh.get(Project, legacy.id)
        assert reloaded is not None
        assert reloaded.slug == "legacy-one"
