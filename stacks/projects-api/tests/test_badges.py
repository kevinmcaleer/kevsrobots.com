"""Tests for badges & achievements (issue #106).

Covers:
* Catalog seeding is idempotent and re-runs without duplicating rows.
* Badge awarded on threshold crossing (first project, prolific maker).
* Badge NOT awarded below threshold.
* Tiered progression — crossing silver also awards bronze.
* Re-evaluation is idempotent (no double rows even if you call it twice).
* Public ``GET /api/badges`` lists the catalog.
* Public ``GET /api/users/{username}/badges`` returns earned rows.
* ``POST /api/badges/evaluate/{username}`` enforces self-or-admin.
* ``newly_awarded_badges`` is attached to the project-create response.
"""

from __future__ import annotations

import datetime as _dt

import pytest
from sqlalchemy import select

from projects_api.badges import (
    BADGE_CATALOG,
    evaluate_user,
    list_user_badges,
    seed_badge_definitions,
)
from projects_api.models import BadgeDefinition, Project, UserBadge

from .conftest import make_auth_header


# --- Seeding -------------------------------------------------------------


@pytest.mark.asyncio
async def test_seed_is_idempotent(session) -> None:
    n1 = await seed_badge_definitions(session)
    assert n1 == len(BADGE_CATALOG)  # First run inserts everything.

    # Second run should touch nothing — no copy drift.
    n2 = await seed_badge_definitions(session)
    assert n2 == 0

    rows = (await session.scalars(select(BadgeDefinition))).all()
    assert len(rows) == len(BADGE_CATALOG)
    # No duplicate slugs.
    assert len({r.slug for r in rows}) == len(rows)


@pytest.mark.asyncio
async def test_seed_updates_copy_drift(session) -> None:
    await seed_badge_definitions(session)
    row = (
        await session.scalars(
            select(BadgeDefinition).where(BadgeDefinition.slug == "first_project")
        )
    ).one()
    row.name = "Old Name From A Past Deploy"
    await session.commit()

    touched = await seed_badge_definitions(session)
    assert touched >= 1
    refreshed = (
        await session.scalars(
            select(BadgeDefinition).where(BadgeDefinition.slug == "first_project")
        )
    ).one()
    assert refreshed.name == "First Project"


# --- Threshold awarding --------------------------------------------------


@pytest.mark.asyncio
async def test_below_threshold_awards_nothing(session) -> None:
    await seed_badge_definitions(session)
    # User exists in the system but has done literally nothing — no rows.
    awarded = await evaluate_user(session, "ghost")
    assert awarded == []


@pytest.mark.asyncio
async def test_first_project_awards_on_create(client) -> None:
    headers = make_auth_header("alice")
    resp = await client.post(
        "/api/projects",
        json={"title": "Alice's First Bot"},
        headers=headers,
    )
    assert resp.status_code == 201
    body = resp.json()
    slugs = {b["slug"] for b in body.get("newly_awarded_badges", [])}
    assert "first_project" in slugs


@pytest.mark.asyncio
async def test_prolific_maker_tiered_progression(session) -> None:
    """Crossing the silver threshold awards bronze too (if not yet held).

    Builds 10 projects directly in the DB (faster than 10 API calls), then
    runs the evaluator once and expects both first_project, prolific_maker_bronze
    and prolific_maker_silver — but NOT _gold.
    """
    await seed_badge_definitions(session)

    for i in range(10):
        session.add(
            Project(
                title=f"Bot {i}",
                short_description="x",
                status="wip",
                author_username="alice",
            )
        )
    await session.commit()

    awarded = await evaluate_user(session, "alice")
    slugs = {b.slug for b in awarded}
    assert "first_project" in slugs
    assert "prolific_maker_bronze" in slugs
    assert "prolific_maker_silver" in slugs
    assert "prolific_maker_gold" not in slugs


@pytest.mark.asyncio
async def test_evaluation_is_idempotent(session) -> None:
    await seed_badge_definitions(session)
    session.add(
        Project(
            title="One Bot",
            short_description="x",
            status="wip",
            author_username="bob",
        )
    )
    await session.commit()

    first = await evaluate_user(session, "bob")
    assert len(first) >= 1

    second = await evaluate_user(session, "bob")
    assert second == []  # nothing new the second time

    # And exactly one user_badges row per badge (no duplicates).
    rows = (
        await session.scalars(
            select(UserBadge).where(UserBadge.user_id == "bob")
        )
    ).all()
    assert len({r.badge_id for r in rows}) == len(rows)


@pytest.mark.asyncio
async def test_remixer_badge_awarded(session) -> None:
    await seed_badge_definitions(session)

    # bob authors a remix (remixed_from_id set, even pointing at himself
    # for the test — the evaluator doesn't validate the FK target).
    session.add(
        Project(
            title="Original",
            status="wip",
            author_username="bob",
        )
    )
    await session.commit()
    parent = (
        await session.scalars(select(Project).where(Project.title == "Original"))
    ).one()

    session.add(
        Project(
            title="Bob's Remix",
            status="wip",
            author_username="bob",
            remixed_from_id=parent.id,
            remix_description="changed something important",
        )
    )
    await session.commit()

    awarded = await evaluate_user(session, "bob")
    slugs = {b.slug for b in awarded}
    assert "remixer_bronze" in slugs


@pytest.mark.asyncio
async def test_streak_counts_consecutive_months(session) -> None:
    await seed_badge_definitions(session)

    # Three projects in consecutive months — should trip the bronze streak.
    base = _dt.datetime(2025, 1, 15)
    for offset_months in (0, 1, 2):
        year, month = base.year, base.month + offset_months
        # Normalise overflow (simple — month+offset won't exceed 12 in test).
        ts = _dt.datetime(year, month, 15)
        session.add(
            Project(
                title=f"Streak {offset_months}",
                status="wip",
                author_username="streaky",
                created_at=ts,
                updated_at=ts,
            )
        )
    await session.commit()

    awarded = await evaluate_user(session, "streaky")
    slugs = {b.slug for b in awarded}
    assert "streak_bronze" in slugs
    assert "streak_silver" not in slugs


# --- Endpoints -----------------------------------------------------------


@pytest.mark.asyncio
async def test_get_catalog_public(client) -> None:
    resp = await client.get("/api/badges")
    assert resp.status_code == 200
    catalog = resp.json()
    assert len(catalog) == len(BADGE_CATALOG)
    slugs = {b["slug"] for b in catalog}
    assert "first_project" in slugs
    assert "prolific_maker_gold" in slugs
    # Tiered families are present.
    assert "remixer_bronze" in slugs and "remixer_gold" in slugs


@pytest.mark.asyncio
async def test_get_user_badges_public(client) -> None:
    # Make alice earn first_project, then read it back without auth.
    await client.post(
        "/api/projects",
        json={"title": "Alice Earns A Badge"},
        headers=make_auth_header("alice"),
    )
    resp = await client.get("/api/users/alice/badges")
    assert resp.status_code == 200
    slugs = {b["slug"] for b in resp.json()}
    assert "first_project" in slugs


@pytest.mark.asyncio
async def test_evaluate_requires_self_or_admin(client) -> None:
    # alice creates a project.
    await client.post(
        "/api/projects",
        json={"title": "Alice's Bot"},
        headers=make_auth_header("alice"),
    )

    # bob cannot evaluate alice.
    resp = await client.post(
        "/api/badges/evaluate/alice",
        headers=make_auth_header("bob"),
    )
    assert resp.status_code == 403

    # alice can evaluate herself.
    resp = await client.post(
        "/api/badges/evaluate/alice",
        headers=make_auth_header("alice"),
    )
    assert resp.status_code == 200
    body = resp.json()
    assert body["username"] == "alice"
    # Already awarded on create — re-running adds nothing.
    assert body["newly_awarded"] == []
    assert body["total_earned"] >= 1


@pytest.mark.asyncio
async def test_evaluate_unauthenticated_401(client) -> None:
    resp = await client.post("/api/badges/evaluate/alice")
    assert resp.status_code == 401


@pytest.mark.asyncio
async def test_admin_retro_award(client) -> None:
    # Two users with projects, neither evaluated yet (we'll bypass the
    # auto-eval by going through the DB... but the API already evaluates
    # on POST so we can't easily skip. Instead, just assert the retro
    # endpoint returns a mapping and respects admin auth.
    await client.post(
        "/api/projects",
        json={"title": "Alice Bot"},
        headers=make_auth_header("alice"),
    )
    await client.post(
        "/api/projects",
        json={"title": "Bob Bot"},
        headers=make_auth_header("bob"),
    )

    # Non-admin cannot retro-award.
    resp = await client.post(
        "/api/admin/badges/retro-award",
        headers=make_auth_header("alice"),
    )
    assert resp.status_code == 403

    # Admin can (default admin from config is "kev").
    resp = await client.post(
        "/api/admin/badges/retro-award",
        headers=make_auth_header("kev"),
    )
    assert resp.status_code == 200
    mapping = resp.json()
    assert "alice" in mapping and "bob" in mapping


@pytest.mark.asyncio
async def test_list_user_badges_helper(session) -> None:
    await seed_badge_definitions(session)
    session.add(
        Project(title="X", status="wip", author_username="carol")
    )
    await session.commit()
    await evaluate_user(session, "carol")
    rows = await list_user_badges(session, "carol")
    assert any(r.slug == "first_project" for r in rows)
    # earned_at populated.
    assert all(r.earned_at is not None for r in rows)
