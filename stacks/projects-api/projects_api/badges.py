"""Badge catalog + evaluation engine — issue #106.

The badge system has two halves:

1. **Catalog seeding** (``seed_badge_definitions``): an idempotent upsert
   keyed by ``slug`` so we can change the human-readable name / description
   / icon / threshold without orphaning earned rows. Runs on every startup
   from the FastAPI lifespan.

2. **Evaluation** (``evaluate_user``): computes which badges a single user
   should currently have, given their activity in projects-api's DB, and
   inserts any missing ``user_badges`` rows. Returns the list of *newly*
   awarded badges so the caller (e.g. POST /api/projects -> create) can
   surface a toast.

Data sources used by the evaluator
----------------------------------

projects-api owns ``projects``, ``makes``, ``downloads`` and remix
attribution. It does NOT own comments or likes — those live in Chatter /
the page-counter service. To keep this PR additive and synchronous, the
"comments" and "likes" badges are *defined* (so the gallery shows them as
goals) but their evaluator simply counts what we have access to; both
return 0 today so those badges are never awarded yet. They will start
being awarded automatically once the comments / likes data lands in
projects-api or once we add a cross-service lookup.

Tiered families
---------------

Bronze / silver / gold tiers share a slug *prefix* (e.g.
``prolific_maker_bronze``, ``prolific_maker_silver``,
``prolific_maker_gold``) and a ``category``. Crossing the silver threshold
implies bronze should also be awarded — the evaluator handles this
naturally because each tier is its own definition row with its own
threshold check.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from datetime import datetime
from typing import Awaitable, Callable

from sqlalchemy import func, select
from sqlalchemy.exc import IntegrityError
from sqlalchemy.ext.asyncio import AsyncSession

from .models import BadgeDefinition, Download, Make, Project, UserBadge
from .schemas import EarnedBadgeResponse

logger = logging.getLogger(__name__)


# --- Catalog definition --------------------------------------------------


@dataclass(frozen=True)
class _BadgeSpec:
    slug: str
    name: str
    description: str
    icon: str  # Font Awesome class — no leading "fas " prefix expected
    category: str
    threshold_type: str
    threshold_value: int
    tier: str


# The canonical badge catalog. Keep this list in sync with the user-facing
# spec in issue #106. Slugs are forever — once a slug ships, never rename
# it (existing user_badges rows are keyed by id, but the slug shows up in
# admin tools and analytics, so stability matters).
BADGE_CATALOG: tuple[_BadgeSpec, ...] = (
    _BadgeSpec(
        slug="first_project",
        name="First Project",
        description="Published your first project — welcome to the maker community!",
        icon="fa-solid fa-seedling",
        category="first_project",
        threshold_type="project_count",
        threshold_value=1,
        tier="single",
    ),
    # Prolific Maker family — 5 / 10 / 25 projects.
    _BadgeSpec(
        slug="prolific_maker_bronze",
        name="Prolific Maker (Bronze)",
        description="Published 5 projects.",
        icon="fa-solid fa-medal",
        category="prolific_maker",
        threshold_type="project_count",
        threshold_value=5,
        tier="bronze",
    ),
    _BadgeSpec(
        slug="prolific_maker_silver",
        name="Prolific Maker (Silver)",
        description="Published 10 projects.",
        icon="fa-solid fa-medal",
        category="prolific_maker",
        threshold_type="project_count",
        threshold_value=10,
        tier="silver",
    ),
    _BadgeSpec(
        slug="prolific_maker_gold",
        name="Prolific Maker (Gold)",
        description="Published 25 projects.",
        icon="fa-solid fa-trophy",
        category="prolific_maker",
        threshold_type="project_count",
        threshold_value=25,
        tier="gold",
    ),
    # Popular Project family — 10 / 100 / 1000 downloads.
    _BadgeSpec(
        slug="popular_project_bronze",
        name="Popular Project (Bronze)",
        description="Your projects have been downloaded 10 times.",
        icon="fa-solid fa-download",
        category="popular_project",
        threshold_type="downloads",
        threshold_value=10,
        tier="bronze",
    ),
    _BadgeSpec(
        slug="popular_project_silver",
        name="Popular Project (Silver)",
        description="Your projects have been downloaded 100 times.",
        icon="fa-solid fa-fire",
        category="popular_project",
        threshold_type="downloads",
        threshold_value=100,
        tier="silver",
    ),
    _BadgeSpec(
        slug="popular_project_gold",
        name="Popular Project (Gold)",
        description="Your projects have been downloaded 1,000 times.",
        icon="fa-solid fa-crown",
        category="popular_project",
        threshold_type="downloads",
        threshold_value=1000,
        tier="gold",
    ),
    # Community Builder family — 5 / 25 / 100 makes received on your projects.
    _BadgeSpec(
        slug="community_builder_bronze",
        name="Community Builder (Bronze)",
        description="5 community makes posted on your projects.",
        icon="fa-solid fa-people-group",
        category="community_builder",
        threshold_type="makes_received",
        threshold_value=5,
        tier="bronze",
    ),
    _BadgeSpec(
        slug="community_builder_silver",
        name="Community Builder (Silver)",
        description="25 community makes posted on your projects.",
        icon="fa-solid fa-people-group",
        category="community_builder",
        threshold_type="makes_received",
        threshold_value=25,
        tier="silver",
    ),
    _BadgeSpec(
        slug="community_builder_gold",
        name="Community Builder (Gold)",
        description="100 community makes posted on your projects.",
        icon="fa-solid fa-star",
        category="community_builder",
        threshold_type="makes_received",
        threshold_value=100,
        tier="gold",
    ),
    # Engaged Member family — comments posted. No comments table yet (see
    # module docstring), so these are defined-but-never-awarded for now.
    _BadgeSpec(
        slug="engaged_member_bronze",
        name="Engaged Member (Bronze)",
        description="Posted 10 comments across the community.",
        icon="fa-solid fa-comments",
        category="engaged_member",
        threshold_type="comments",
        threshold_value=10,
        tier="bronze",
    ),
    _BadgeSpec(
        slug="engaged_member_silver",
        name="Engaged Member (Silver)",
        description="Posted 50 comments across the community.",
        icon="fa-solid fa-comments",
        category="engaged_member",
        threshold_type="comments",
        threshold_value=50,
        tier="silver",
    ),
    _BadgeSpec(
        slug="engaged_member_gold",
        name="Engaged Member (Gold)",
        description="Posted 200 comments across the community.",
        icon="fa-solid fa-comments",
        category="engaged_member",
        threshold_type="comments",
        threshold_value=200,
        tier="gold",
    ),
    # Well Liked family — likes received. Same data-source caveat as above.
    _BadgeSpec(
        slug="well_liked_bronze",
        name="Well Liked (Bronze)",
        description="Your projects have received 10 likes.",
        icon="fa-regular fa-heart",
        category="well_liked",
        threshold_type="likes_received",
        threshold_value=10,
        tier="bronze",
    ),
    _BadgeSpec(
        slug="well_liked_silver",
        name="Well Liked (Silver)",
        description="Your projects have received 50 likes.",
        icon="fa-solid fa-heart",
        category="well_liked",
        threshold_type="likes_received",
        threshold_value=50,
        tier="silver",
    ),
    _BadgeSpec(
        slug="well_liked_gold",
        name="Well Liked (Gold)",
        description="Your projects have received 250 likes.",
        icon="fa-solid fa-heart-circle-bolt",
        category="well_liked",
        threshold_type="likes_received",
        threshold_value=250,
        tier="gold",
    ),
    # Streak family — distinct calendar months with at least one project.
    _BadgeSpec(
        slug="streak_bronze",
        name="Streak (Bronze)",
        description="Published projects in 3 consecutive months.",
        icon="fa-solid fa-calendar-day",
        category="streak",
        threshold_type="consecutive_months",
        threshold_value=3,
        tier="bronze",
    ),
    _BadgeSpec(
        slug="streak_silver",
        name="Streak (Silver)",
        description="Published projects in 6 consecutive months.",
        icon="fa-solid fa-calendar-check",
        category="streak",
        threshold_type="consecutive_months",
        threshold_value=6,
        tier="silver",
    ),
    _BadgeSpec(
        slug="streak_gold",
        name="Streak (Gold)",
        description="Published projects in 12 consecutive months.",
        icon="fa-solid fa-calendar-star",
        category="streak",
        threshold_type="consecutive_months",
        threshold_value=12,
        tier="gold",
    ),
    # Remixer family — projects you created that are remixes.
    _BadgeSpec(
        slug="remixer_bronze",
        name="Remixer (Bronze)",
        description="Created your first remix of someone else's project.",
        icon="fa-solid fa-code-branch",
        category="remixer",
        threshold_type="remixes",
        threshold_value=1,
        tier="bronze",
    ),
    _BadgeSpec(
        slug="remixer_silver",
        name="Remixer (Silver)",
        description="Created 5 remixes.",
        icon="fa-solid fa-code-branch",
        category="remixer",
        threshold_type="remixes",
        threshold_value=5,
        tier="silver",
    ),
    _BadgeSpec(
        slug="remixer_gold",
        name="Remixer (Gold)",
        description="Created 15 remixes.",
        icon="fa-solid fa-code-fork",
        category="remixer",
        threshold_type="remixes",
        threshold_value=15,
        tier="gold",
    ),
)


# --- Seeding -------------------------------------------------------------


async def seed_badge_definitions(session: AsyncSession) -> int:
    """Upsert the canonical badge catalog into the DB.

    Idempotent: runs on every startup. Updates name/description/icon/
    threshold for an existing row when they have drifted (so we can tweak
    copy in the code without a migration), and inserts any new slugs.

    Returns the number of rows touched (insert + update).
    """
    existing_rows = (await session.scalars(select(BadgeDefinition))).all()
    existing_by_slug = {row.slug: row for row in existing_rows}

    touched = 0
    for spec in BADGE_CATALOG:
        row = existing_by_slug.get(spec.slug)
        if row is None:
            session.add(
                BadgeDefinition(
                    slug=spec.slug,
                    name=spec.name,
                    description=spec.description,
                    icon=spec.icon,
                    category=spec.category,
                    threshold_type=spec.threshold_type,
                    threshold_value=spec.threshold_value,
                    tier=spec.tier,
                )
            )
            touched += 1
            continue

        # Update mutable copy fields if they've drifted.
        dirty = False
        for attr in ("name", "description", "icon", "category", "threshold_type", "threshold_value", "tier"):
            if getattr(row, attr) != getattr(spec, attr):
                setattr(row, attr, getattr(spec, attr))
                dirty = True
        if dirty:
            touched += 1

    if touched:
        await session.commit()
    return touched


# --- Threshold counters --------------------------------------------------
#
# Each counter is an async function that returns an int for a given
# username. Keeping them isolated makes them easy to test (you can build a
# tiny session-fixture and call them directly) and easy to swap when we
# wire in external sources for comments / likes.


async def _count_projects(session: AsyncSession, username: str) -> int:
    # All non-archived authored projects. We intentionally include WIP and
    # blocked projects toward the count — the badge celebrates effort, not
    # only published-and-live status. (Archived is excluded because users
    # explicitly hide those.)
    val = await session.scalar(
        select(func.count(Project.id)).where(
            Project.author_username == username,
            Project.status != "archived",
        )
    )
    return int(val or 0)


async def _count_downloads(session: AsyncSession, username: str) -> int:
    # Total downloads across all projects owned by the user. Join through
    # Project on author_username to avoid storing the username on Download
    # rows.
    val = await session.scalar(
        select(func.count(Download.id))
        .join(Project, Project.id == Download.project_id)
        .where(Project.author_username == username)
    )
    return int(val or 0)


async def _count_makes_received(session: AsyncSession, username: str) -> int:
    # Makes posted on projects this user owns.
    val = await session.scalar(
        select(func.count(Make.id))
        .join(Project, Project.id == Make.project_id)
        .where(Project.author_username == username)
    )
    return int(val or 0)


async def _count_remixes_created(session: AsyncSession, username: str) -> int:
    # Projects this user authored that are themselves remixes.
    val = await session.scalar(
        select(func.count(Project.id)).where(
            Project.author_username == username,
            Project.remixed_from_id.is_not(None),
        )
    )
    return int(val or 0)


async def _count_comments(session: AsyncSession, username: str) -> int:
    # No local comments table — comments live in Chatter. Return 0 so the
    # Engaged Member badges are defined-but-never-awarded for now. When a
    # comments source lands, swap this for the real count.
    return 0


async def _count_likes_received(session: AsyncSession, username: str) -> int:
    # No local likes table — likes are tracked by the page-counter service.
    # Same treatment as comments above.
    return 0


async def _consecutive_months_with_project(
    session: AsyncSession, username: str
) -> int:
    """Longest run of consecutive calendar months containing a project.

    Builds the set of (year, month) buckets in which the user published a
    project, then walks them in order counting the longest unbroken run.
    Two-month gap breaks the streak; one-month gap (i.e. adjacent month)
    extends it.
    """
    rows = (
        await session.scalars(
            select(Project.created_at).where(
                Project.author_username == username,
                Project.status != "archived",
            )
        )
    ).all()
    if not rows:
        return 0

    months: set[tuple[int, int]] = set()
    for ts in rows:
        if isinstance(ts, datetime):
            months.add((ts.year, ts.month))
    if not months:
        return 0

    sorted_months = sorted(months)
    best = current = 1
    for i in range(1, len(sorted_months)):
        prev_y, prev_m = sorted_months[i - 1]
        cur_y, cur_m = sorted_months[i]
        # Compute month-distance: ((y2*12 + m2) - (y1*12 + m1)) == 1 when adjacent.
        delta = (cur_y * 12 + cur_m) - (prev_y * 12 + prev_m)
        if delta == 1:
            current += 1
        else:
            current = 1
        if current > best:
            best = current
    return best


_CounterFn = Callable[[AsyncSession, str], Awaitable[int]]

_THRESHOLD_COUNTERS: dict[str, _CounterFn] = {
    "project_count": _count_projects,
    "downloads": _count_downloads,
    "makes_received": _count_makes_received,
    "remixes": _count_remixes_created,
    "comments": _count_comments,
    "likes_received": _count_likes_received,
    "consecutive_months": _consecutive_months_with_project,
}


# --- Evaluation ---------------------------------------------------------


async def evaluate_user(
    session: AsyncSession,
    username: str,
) -> list[EarnedBadgeResponse]:
    """Recompute earned badges for ``username`` and award any new ones.

    Returns the list of badges awarded on *this call* (i.e. newly created
    rows in ``user_badges``), so the caller can fire a toast notification.
    Already-earned badges are not re-emitted.

    Idempotent: calling repeatedly without new activity returns ``[]``.

    The function commits its own changes; callers should call it OUTSIDE
    of an in-flight transaction they're about to rollback. In practice,
    event-hook callers commit their own row first (project create, make
    create, etc.) then invoke this on a follow-up session — see the call
    sites for the pattern.
    """
    # Snapshot the catalog. We don't want to interleave network/DB calls
    # for each badge — collect first, then loop.
    definitions = (await session.scalars(select(BadgeDefinition))).all()
    if not definitions:
        return []

    # Pre-fetch what the user already has so we don't double-award.
    earned_rows = (
        await session.scalars(
            select(UserBadge).where(UserBadge.user_id == username)
        )
    ).all()
    already_earned_ids = {row.badge_id for row in earned_rows}

    # Cache the per-threshold-type counts so the same counter doesn't run
    # 3 times for a tiered family. This makes evaluate_user O(distinct
    # threshold types) DB lookups, not O(catalog size).
    counts: dict[str, int] = {}

    newly_awarded: list[EarnedBadgeResponse] = []
    for d in definitions:
        if d.id in already_earned_ids:
            continue
        counter = _THRESHOLD_COUNTERS.get(d.threshold_type)
        if counter is None:
            # Defensive: a future migration may add a threshold_type we
            # haven't wired a counter for yet. Skip rather than crash.
            logger.warning(
                "No counter registered for threshold_type %r (badge %s)",
                d.threshold_type, d.slug,
            )
            continue

        if d.threshold_type not in counts:
            counts[d.threshold_type] = await counter(session, username)
        if counts[d.threshold_type] < d.threshold_value:
            continue

        new_row = UserBadge(user_id=username, badge_id=d.id)
        session.add(new_row)
        try:
            await session.flush()
        except IntegrityError:
            # Race: another concurrent evaluate_user beat us. Rollback the
            # bad insert and skip — the badge is already there.
            await session.rollback()
            continue

        newly_awarded.append(
            EarnedBadgeResponse(
                id=d.id,
                slug=d.slug,
                name=d.name,
                description=d.description,
                icon=d.icon,
                category=d.category,
                tier=d.tier,
                earned_at=new_row.earned_at or datetime.utcnow(),
            )
        )

    if newly_awarded:
        await session.commit()
    return newly_awarded


async def list_user_badges(
    session: AsyncSession, username: str
) -> list[EarnedBadgeResponse]:
    """Return all badges currently earned by ``username``.

    Joined query so we only round-trip the DB once.
    """
    rows = (
        await session.execute(
            select(UserBadge, BadgeDefinition)
            .join(BadgeDefinition, BadgeDefinition.id == UserBadge.badge_id)
            .where(UserBadge.user_id == username)
            .order_by(UserBadge.earned_at.asc())
        )
    ).all()
    return [
        EarnedBadgeResponse(
            id=d.id,
            slug=d.slug,
            name=d.name,
            description=d.description,
            icon=d.icon,
            category=d.category,
            tier=d.tier,
            earned_at=ub.earned_at,
        )
        for ub, d in rows
    ]


async def retro_award_all_users(session: AsyncSession) -> dict[str, int]:
    """Run ``evaluate_user`` against every known author + maker username.

    Used at startup (one-shot after a fresh deploy) and exposed via the
    admin endpoint so badges can be backfilled for everyone who was
    active before the system existed.

    Returns a ``{username: newly_awarded_count}`` mapping for the audit
    log. Crashes are swallowed per-user so one bad row doesn't stop the
    batch.
    """
    # Collect every username we know about across the data sources.
    users: set[str] = set()
    for rows, col in (
        ((await session.scalars(select(Project.author_username))).all(), Project.author_username),
        ((await session.scalars(select(Make.user_id))).all(), Make.user_id),
    ):
        for name in rows:
            if name:
                users.add(name)

    result: dict[str, int] = {}
    for name in sorted(users):
        try:
            awarded = await evaluate_user(session, name)
            result[name] = len(awarded)
        except Exception as exc:  # noqa: BLE001 — best-effort batch
            logger.warning("Retro badge eval failed for %s: %s", name, exc)
            result[name] = 0
    return result
