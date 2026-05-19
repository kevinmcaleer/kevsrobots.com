"""SQLAlchemy models for the Projects API."""

from __future__ import annotations

from datetime import datetime
from typing import Any, Optional

from sqlalchemy import (
    ARRAY,
    Boolean,
    Date,
    DateTime,
    Float,
    ForeignKey,
    Index,
    Integer,
    String,
    Text,
    UniqueConstraint,
    func,
)
from sqlalchemy.dialects.postgresql import JSONB
from sqlalchemy.orm import Mapped, mapped_column
from sqlalchemy.types import JSON

from .db import Base

JsonType = JSONB().with_variant(JSON(), "sqlite")

# Tags array: native text[] on Postgres, JSON list on SQLite (for tests).
TagsType = ARRAY(String).with_variant(JSON(), "sqlite")


class Project(Base):
    __tablename__ = "projects"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    title: Mapped[str] = mapped_column(String(200), nullable=False)
    # Issue #152: URL-friendly slug, unique per author. Nullable so the
    # column can be backfilled on legacy rows by the lifespan helper. The
    # ``UniqueConstraint`` below enforces (author_username, slug) — username
    # uniqueness itself is owned by Chatter (the auth service); our copy of
    # ``author_username`` is a string FK to that identity.
    slug: Mapped[Optional[str]] = mapped_column(String(80), nullable=True, index=True)
    short_description: Mapped[Optional[str]] = mapped_column(String(500))
    content_md: Mapped[Optional[str]] = mapped_column(Text)
    difficulty: Mapped[Optional[str]] = mapped_column(String(20))
    estimated_minutes: Mapped[Optional[int]] = mapped_column(Integer)
    code_repo_url: Mapped[Optional[str]] = mapped_column(Text)
    status: Mapped[str] = mapped_column(String(20), nullable=False, default="wip")
    author_username: Mapped[str] = mapped_column(String(100), nullable=False)
    cover_image: Mapped[Optional[str]] = mapped_column(Text)
    is_blocked: Mapped[bool] = mapped_column(Boolean, nullable=False, default=False, server_default="0")
    moderation_note: Mapped[Optional[str]] = mapped_column(Text)
    # Remix/fork attribution (issue #108). remixed_from_id points at the parent
    # project this one was remixed from. ondelete=SET NULL so deleting an
    # original project doesn't cascade-delete its derivatives. The
    # application layer is responsible for ensuring remix_description is
    # populated whenever remixed_from_id is set.
    remixed_from_id: Mapped[Optional[int]] = mapped_column(
        ForeignKey("projects.id", ondelete="SET NULL"), nullable=True, index=True
    )
    remix_description: Mapped[Optional[str]] = mapped_column(Text)
    # Issue #115: Featured / Staff Pick metadata. Additive — does not change
    # any existing query (hub still selects newest-first). When
    # ``is_featured`` flips true, the other three are populated together;
    # ``DELETE /api/admin/projects/{id}/feature`` clears all four.
    is_featured: Mapped[bool] = mapped_column(
        Boolean, nullable=False, default=False, server_default="0", index=True
    )
    featured_at: Mapped[Optional[datetime]] = mapped_column(DateTime)
    featured_by: Mapped[Optional[str]] = mapped_column(String(100))
    featured_note: Mapped[Optional[str]] = mapped_column(String(200))
    created_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )
    updated_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )

    __table_args__ = (
        # Issue #152: slug is unique within an author's portfolio. Two
        # different users can both have a "smars-walker" slug.
        UniqueConstraint(
            "author_username", "slug", name="uq_projects_author_slug"
        ),
    )


class ProjectReport(Base):
    __tablename__ = "project_reports"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    project_id: Mapped[int] = mapped_column(
        ForeignKey("projects.id", ondelete="CASCADE"), nullable=False, index=True
    )
    reporter_username: Mapped[str] = mapped_column(String(100), nullable=False)
    reason: Mapped[str] = mapped_column(Text, nullable=False)
    status: Mapped[str] = mapped_column(String(20), nullable=False, default="pending")
    created_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )
    reviewed_at: Mapped[Optional[datetime]] = mapped_column(DateTime)
    reviewed_by: Mapped[Optional[str]] = mapped_column(String(100))


class ProjectTag(Base):
    __tablename__ = "project_tags"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    project_id: Mapped[int] = mapped_column(
        ForeignKey("projects.id", ondelete="CASCADE"), nullable=False, index=True
    )
    tag: Mapped[str] = mapped_column(String(50), nullable=False)


class ProjectBOMItem(Base):
    __tablename__ = "project_bom_items"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    project_id: Mapped[int] = mapped_column(
        ForeignKey("projects.id", ondelete="CASCADE"), nullable=False, index=True
    )
    name: Mapped[str] = mapped_column(String(200), nullable=False)
    quantity: Mapped[int] = mapped_column(Integer, nullable=False, default=1)
    unit: Mapped[str] = mapped_column(String(20), nullable=False, default="qty")
    unit_cost: Mapped[Optional[float]] = mapped_column(Float)
    # Issue #149: ISO 4217 currency code that ``unit_cost`` is denominated
    # in (e.g. ``GBP``, ``USD``, ``EUR``). Nullable for back-compat — legacy
    # rows pre-#149 have NULL and the frontend renders the raw number with
    # a "currency not set" tooltip. Validated via the Pydantic ``pattern``
    # on ``BOMItemCreate``.
    currency_code: Mapped[Optional[str]] = mapped_column(String(3))
    supplier_url: Mapped[Optional[str]] = mapped_column(Text)
    sort_order: Mapped[int] = mapped_column(Integer, nullable=False, default=0)
    # Optional link to a part in the shared parts catalog. When set, the
    # part's data is the source of truth and the free-form fields above are
    # kept for back-compat / display. Set to NULL on part delete so rows
    # survive parts catalog churn.
    part_id: Mapped[Optional[int]] = mapped_column(
        ForeignKey("parts.id", ondelete="SET NULL"), nullable=True, index=True
    )


class ProjectLink(Base):
    __tablename__ = "project_links"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    project_id: Mapped[int] = mapped_column(
        ForeignKey("projects.id", ondelete="CASCADE"), nullable=False, index=True
    )
    title: Mapped[str] = mapped_column(String(200), nullable=False)
    url: Mapped[str] = mapped_column(Text, nullable=False)
    link_type: Mapped[str] = mapped_column(String(50), nullable=False, default="article")


class ProjectFile(Base):
    __tablename__ = "project_files"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    project_id: Mapped[int] = mapped_column(
        ForeignKey("projects.id", ondelete="CASCADE"), nullable=False, index=True
    )
    filename: Mapped[str] = mapped_column(String(255), nullable=False)
    file_path: Mapped[str] = mapped_column(Text, nullable=False)
    file_size: Mapped[int] = mapped_column(Integer, nullable=False)
    file_type: Mapped[str] = mapped_column(String(50), nullable=False)
    description: Mapped[Optional[str]] = mapped_column(Text)
    uploaded_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )


class ProjectImage(Base):
    __tablename__ = "project_images"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    project_id: Mapped[int] = mapped_column(
        ForeignKey("projects.id", ondelete="CASCADE"), nullable=False, index=True
    )
    filename: Mapped[str] = mapped_column(String(255), nullable=False)
    file_path: Mapped[str] = mapped_column(Text, nullable=False)
    caption: Mapped[Optional[str]] = mapped_column(Text)
    sort_order: Mapped[int] = mapped_column(Integer, nullable=False, default=0)
    uploaded_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )


class ProjectJournalEntry(Base):
    __tablename__ = "project_journal_entries"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    project_id: Mapped[int] = mapped_column(
        ForeignKey("projects.id", ondelete="CASCADE"), nullable=False, index=True
    )
    title: Mapped[str] = mapped_column(String(200), nullable=False)
    content_md: Mapped[Optional[str]] = mapped_column(Text)
    status: Mapped[str] = mapped_column(String(20), nullable=False, default="in_progress")
    created_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )
    updated_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )


class ProjectJournalImage(Base):
    __tablename__ = "project_journal_images"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    entry_id: Mapped[int] = mapped_column(
        ForeignKey("project_journal_entries.id", ondelete="CASCADE"), nullable=False, index=True
    )
    filename: Mapped[str] = mapped_column(String(255), nullable=False)
    file_path: Mapped[str] = mapped_column(Text, nullable=False)
    uploaded_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )


# --- Community Makes ("I Made This!") -- issue #107 -----------------------
#
# A `Make` is a user-submitted "I built this!" post on someone else's
# project. It carries optional notes + modifications text and 0-5 images.
# The project's author may "heart" individual makes; we model that with a
# single nullable `hearted_at` column on `Make` rather than a join table
# because hearts come from exactly one user (the project author) so a
# many-to-many relation is overkill. If we later want multi-user hearts we
# can promote this to a `make_hearts` table without rewriting the read
# path (the `hearted_by_author` response field stays the same).


class Make(Base):
    __tablename__ = "makes"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    project_id: Mapped[int] = mapped_column(
        ForeignKey("projects.id", ondelete="CASCADE"), nullable=False, index=True
    )
    user_id: Mapped[str] = mapped_column(String(100), nullable=False, index=True)
    notes: Mapped[Optional[str]] = mapped_column(Text)
    modifications: Mapped[Optional[str]] = mapped_column(Text)
    hearted_at: Mapped[Optional[datetime]] = mapped_column(DateTime)
    created_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )


class MakeImage(Base):
    __tablename__ = "make_images"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    make_id: Mapped[int] = mapped_column(
        ForeignKey("makes.id", ondelete="CASCADE"), nullable=False, index=True
    )
    filename: Mapped[str] = mapped_column(String(255), nullable=False)
    file_path: Mapped[str] = mapped_column(Text, nullable=False)
    file_size: Mapped[int] = mapped_column(Integer, nullable=False, default=0)
    caption: Mapped[Optional[str]] = mapped_column(Text)
    sort_order: Mapped[int] = mapped_column(Integer, nullable=False, default=0)
    uploaded_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )


class Download(Base):
    """Tracks individual file downloads for analytics and popularity ranking.

    Anonymous downloads use `ip_hash` (sha256 of client IP + IP_HASH_SALT),
    authenticated downloads use `user_id`. Dedup is enforced at the router
    level: same (project_id, file_id, identity) within 24h counts once.
    """

    __tablename__ = "downloads"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    project_id: Mapped[int] = mapped_column(
        ForeignKey("projects.id", ondelete="CASCADE"), nullable=False
    )
    file_id: Mapped[int] = mapped_column(
        ForeignKey("project_files.id", ondelete="CASCADE"), nullable=False
    )
    user_id: Mapped[Optional[str]] = mapped_column(String(100))
    ip_hash: Mapped[Optional[str]] = mapped_column(String(64))
    downloaded_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )

    __table_args__ = (
        Index("ix_downloads_project_downloaded_at", "project_id", "downloaded_at"),
        Index("ix_downloads_file_downloaded_at", "file_id", "downloaded_at"),
    )


# --- Parts catalog (issue #121, Phase 1) ---------------------------------
#
# A shared, wiki-style catalog of reusable components. BOM rows may link to
# a `Part` so that supplier URLs / specs / images are looked up once and
# reused across projects.
#
# Schema decisions for v1:
#   * `current_revision_id` is a nullable FK to break the circular dep with
#     `part_revisions`. It's set after the first revision row is written.
#   * `usage_count` is a denormalised cache of distinct projects using the
#     part. We update it in the same transaction as BOM mutations (see
#     `routers/bom.py`). A Phase-2 cron will recompute it from
#     `SELECT COUNT(DISTINCT project_id) FROM project_bom_items WHERE part_id=?`
#     to repair any drift.
#   * `status` is a free string for now — `draft`/`verified`/`under_review`.
#     The `merged_into:<id>` value comes in Phase 3 (merge proposals).


class Part(Base):
    __tablename__ = "parts"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    slug: Mapped[str] = mapped_column(String(120), nullable=False, unique=True, index=True)
    name: Mapped[str] = mapped_column(String(200), nullable=False)
    sku: Mapped[Optional[str]] = mapped_column(String(100), index=True)
    mpn: Mapped[Optional[str]] = mapped_column(String(100), index=True)
    description_md: Mapped[Optional[str]] = mapped_column(Text)
    image_url: Mapped[Optional[str]] = mapped_column(Text)
    tags: Mapped[Optional[list]] = mapped_column(TagsType)
    # Issue #122 (Phase 2): lifecycle status. Values: ``draft`` (initial,
    # set on creation), ``verified`` (auto-promoted when the heuristic
    # rules in ``parts_lifecycle.compute_part_status`` all pass), or
    # ``disputed`` (auto-demoted when an open Phase 3 report flags it as
    # wrong/duplicate). Indexed because the catalog browse page filters by
    # status. The legacy ``under_review`` value is still tolerated on read
    # for back-compat with Phase 1 deployments.
    status: Mapped[str] = mapped_column(String(30), nullable=False, default="draft", index=True)
    # Issue #122: timestamp the part was promoted to "verified". Null
    # while in draft / disputed. Reset to null on demotion to disputed.
    verified_at: Mapped[Optional[datetime]] = mapped_column(DateTime)
    # Issue #122: monotonically incrementing "trust counter" so future
    # phases can reason about how strong the verification signal is
    # without re-running the heuristic. Bumped every time a promotion
    # rule passes; never decremented.
    verified_signals: Mapped[int] = mapped_column(
        Integer, nullable=False, default=0, server_default="0"
    )
    # Issue #135: high-level category bucket (e.g. "microcontroller",
    # "sensor", "motor-driver"). Free string with a curated starter list in
    # the editor; null on legacy rows. Kept as a string rather than a FK so
    # we can extend the taxonomy without schema migrations — if it
    # eventually justifies a categories table we can promote it later.
    category: Mapped[Optional[str]] = mapped_column(String(60), index=True)
    # Issue #135: product family grouping (e.g. "raspberry-pi-pico" covers
    # Pico / Pico W / Pico 2 / Pico 2W). Free string with autocomplete
    # against existing values. Same FK-vs-string trade-off as `category`.
    family: Mapped[Optional[str]] = mapped_column(String(80), index=True)
    created_by: Mapped[str] = mapped_column(String(100), nullable=False)
    created_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )
    updated_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )
    # FK is added without `use_alter` because we never INSERT a part with a
    # known revision id — the revision row is written first, then we patch
    # this column.
    current_revision_id: Mapped[Optional[int]] = mapped_column(
        ForeignKey("part_revisions.id", ondelete="SET NULL", use_alter=True, name="fk_parts_current_revision"),
        nullable=True,
    )
    usage_count: Mapped[int] = mapped_column(Integer, nullable=False, default=0, server_default="0")


class PartRelation(Base):
    """Symmetric many-to-many "related parts" links (issue #135).

    Each user-facing link writes a SINGLE row with ``part_id < related_id``
    (canonical ordering) so we never store both (A,B) and (B,A) and the
    unique constraint catches duplicates regardless of which side the
    editor came from. Router code is responsible for swapping the ids
    before writing. Reads union both directions.
    """

    __tablename__ = "part_relations"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    part_id: Mapped[int] = mapped_column(
        ForeignKey("parts.id", ondelete="CASCADE"), nullable=False, index=True
    )
    related_id: Mapped[int] = mapped_column(
        ForeignKey("parts.id", ondelete="CASCADE"), nullable=False, index=True
    )
    created_by: Mapped[str] = mapped_column(String(100), nullable=False)
    created_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )

    __table_args__ = (
        Index("ix_part_relations_unique_pair", "part_id", "related_id", unique=True),
    )


class PartAlias(Base):
    __tablename__ = "part_aliases"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    part_id: Mapped[int] = mapped_column(
        ForeignKey("parts.id", ondelete="CASCADE"), nullable=False, index=True
    )
    alias: Mapped[str] = mapped_column(String(200), nullable=False)

    __table_args__ = (
        # Functional index for case-insensitive search. Postgres respects it
        # for queries against `func.lower(alias)`; SQLite ignores the func
        # expression and falls back to a sequential scan (fine for tests).
        Index("ix_part_aliases_lower_alias", func.lower(alias)),
    )


class PartSupplier(Base):
    __tablename__ = "part_suppliers"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    part_id: Mapped[int] = mapped_column(
        ForeignKey("parts.id", ondelete="CASCADE"), nullable=False, index=True
    )
    supplier_name: Mapped[Optional[str]] = mapped_column(String(120))
    url: Mapped[str] = mapped_column(Text, nullable=False)
    last_checked_at: Mapped[Optional[datetime]] = mapped_column(DateTime)
    last_status: Mapped[Optional[str]] = mapped_column(String(30))
    # Issue #122 (Phase 2): supplier link health check fields. The daily
    # APScheduler job populates ``last_status_code`` after each HEAD-check
    # pass. ``is_broken`` flips true only after ``consecutive_failures``
    # crosses 3 — a single transient 5xx will not mark a supplier broken.
    last_status_code: Mapped[Optional[int]] = mapped_column(Integer)
    is_broken: Mapped[bool] = mapped_column(
        Boolean, nullable=False, default=False, server_default="0"
    )
    consecutive_failures: Mapped[int] = mapped_column(
        Integer, nullable=False, default=0, server_default="0"
    )
    # Issue #149: ISO 3166-1 alpha-2 country code for the supplier link
    # (e.g. ``GB`` for Pimoroni, ``US`` for Adafruit). Nullable means
    # "global / unknown" — legacy rows pre-#149 have NULL. Validated via
    # the Pydantic ``pattern`` on ``PartSupplierInput``.
    country_code: Mapped[Optional[str]] = mapped_column(String(2))


# --- Parts talk pages (issue #122, Phase 2) ------------------------------
#
# Wikipedia-style discussion attached to each Part. One Part may have many
# threads; each thread is an append-only sequence of posts. We deliberately
# do NOT model nested replies — threads stay flat to keep the renderer
# trivial. Editing tracks ``edited_at`` only; we don't store edit history
# for talk content (unlike revisions on the part itself, which DO have
# full snapshot history). Closing a thread is a soft state flag; closed
# threads remain readable but stop accepting new posts.
#
# New tables — ``create_all`` handles them. No ALTER helper needed.


class PartTalkThread(Base):
    __tablename__ = "part_talk_threads"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    part_id: Mapped[int] = mapped_column(
        ForeignKey("parts.id", ondelete="CASCADE"), nullable=False, index=True
    )
    title: Mapped[str] = mapped_column(String(200), nullable=False)
    created_by: Mapped[str] = mapped_column(String(100), nullable=False)
    created_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )
    updated_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )
    closed: Mapped[bool] = mapped_column(
        Boolean, nullable=False, default=False, server_default="0"
    )
    closed_by: Mapped[Optional[str]] = mapped_column(String(100))
    closed_at: Mapped[Optional[datetime]] = mapped_column(DateTime)

    __table_args__ = (
        # Listing query orders by updated_at desc within a part.
        Index("ix_part_talk_threads_part_updated", "part_id", "updated_at"),
    )


class PartTalkPost(Base):
    __tablename__ = "part_talk_posts"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    thread_id: Mapped[int] = mapped_column(
        ForeignKey("part_talk_threads.id", ondelete="CASCADE"),
        nullable=False,
        index=True,
    )
    author_username: Mapped[str] = mapped_column(String(100), nullable=False)
    content_md: Mapped[str] = mapped_column(Text, nullable=False)
    created_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )
    edited_at: Mapped[Optional[datetime]] = mapped_column(DateTime)


# --- User follows (issue #140) -------------------------------------------
#
# A directional social-graph row: `follower_id` follows `followee_id`.
# Both are Chatter usernames (we don't carry our own user table — the
# auth service is the source of truth). The pair is the PK so a duplicate
# follow is an upsert-conditional no-op; on the API side we treat
# "already following" as 200 OK rather than a 4xx so the UI button is
# safely idempotent. ``created_at`` is kept so we can surface "followed
# you N days ago" later without a schema change.
#
# No ALTER helper is needed — `create_all` builds this table from scratch
# on every Postgres deployment. SQLite tests pick it up the same way.


class UserFollow(Base):
    __tablename__ = "user_follows"

    follower_id: Mapped[str] = mapped_column(String(100), primary_key=True)
    followee_id: Mapped[str] = mapped_column(String(100), primary_key=True)
    created_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )

    __table_args__ = (
        # Index follower_id for "who am I following?" queries; the PK
        # already covers (follower_id, followee_id) so we only need the
        # reverse index for "who follows X?".
        Index("ix_user_follows_followee_id", "followee_id"),
    )


# --- User profiles (issue #111) ------------------------------------------
#
# Identity lives in Chatter — we do not own a `users` table. To attach
# extra profile data (bio, location, social links, featured badges) we
# keep a lightweight `user_profiles` row keyed by Chatter username. Rows
# are created lazily on first PUT; absent rows are returned as "no extra
# profile data" rather than 404. This avoids a sign-up hook and keeps
# Chatter as the source of truth for "does this user exist?".
#
# Migration: the table is new so `create_all` builds it on fresh
# deployments. The companion `add_user_profile_columns_if_missing`
# helper in db.py is a defensive ALTER for any deployment that already
# has a partial `user_profiles` table from a preview branch.


class UserProfile(Base):
    __tablename__ = "user_profiles"

    username: Mapped[str] = mapped_column(String(100), primary_key=True)
    bio: Mapped[Optional[str]] = mapped_column(String(500))
    location: Mapped[Optional[str]] = mapped_column(String(120))
    website_url: Mapped[Optional[str]] = mapped_column(String(200))
    # social_links: {"github": "...", "twitter": "...", "youtube": "...", "mastodon": "..."}
    social_links: Mapped[Optional[dict]] = mapped_column(JsonType)
    # Up to 3 badge slug strings the user has pinned to the showcase.
    featured_badge_slugs: Mapped[Optional[list]] = mapped_column(JsonType)
    # Issue #150: preferred display currency (ISO 4217). NULL means
    # "auto-detect / show native". Validated against an allow-list at
    # the API layer; we do not enforce a FK here so adding new currencies
    # is a code-only change.
    preferred_currency: Mapped[Optional[str]] = mapped_column(String(3))
    created_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )
    updated_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )


# --- User activity feed (issue #111) -------------------------------------
#
# Tiny event log so the profile page can show a "what they've been up to"
# strip. Rows are emitted from existing routes when actions fire; we do
# not backfill historical activity. `subject_url` is precomputed so the
# renderer stays dumb.


class UserActivity(Base):
    __tablename__ = "user_activity"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    user_id: Mapped[str] = mapped_column(String(100), nullable=False, index=True)
    # kind: project_published | project_updated | comment_posted |
    # make_posted | badge_earned | collection_created
    kind: Mapped[str] = mapped_column(String(40), nullable=False)
    subject_id: Mapped[Optional[int]] = mapped_column(Integer)
    subject_title: Mapped[Optional[str]] = mapped_column(String(200))
    subject_url: Mapped[Optional[str]] = mapped_column(String(400))
    created_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False, index=True
    )


# --- Parts moderation: reports + merge proposals (issue #123, Phase 3) ---
#
# Two parallel community-moderation surfaces sit on top of the Phase 1/2
# catalog:
#
#   * ``part_reports`` — anyone logged-in can flag a part as wrong / spam /
#     duplicate / other. An admin works through the queue at
#     ``/admin/parts/`` and accepts or dismisses each report. There is no
#     account-age gate on reporting: we'd rather see noise from new
#     accounts than miss spam they're trying to flag. The
#     ``(part_id, resolved_at)`` index makes the "open reports for this
#     part" and "queue of unresolved reports" queries cheap.
#
#   * ``part_merge_proposals`` + ``part_merge_votes`` — anyone with a
#     14-day-old account can propose merging part A into part B, and any
#     14-day-old account can vote approve / reject on the proposal. Auto-
#     merge fires when the thresholds in
#     ``routers/parts_moderation._check_merge_threshold`` are met. The
#     proposer can withdraw their own proposal; admins can withdraw any.
#     One open proposal per (source, target) pair is enforced at the
#     application layer (the obvious partial-unique-index on
#     ``WHERE resolved_at IS NULL`` is Postgres-only; we keep the gate in
#     Python so SQLite tests behave identically).
#
# Brand-new tables — no ALTER helper needed; ``create_all`` builds them
# on every Postgres deployment. SQLite tests pick them up the same way.


class PartReport(Base):
    __tablename__ = "part_reports"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    part_id: Mapped[int] = mapped_column(
        ForeignKey("parts.id", ondelete="CASCADE"), nullable=False, index=True
    )
    reporter_username: Mapped[str] = mapped_column(String(100), nullable=False)
    # reason: "spam" | "wrong" | "duplicate" | "other"
    reason: Mapped[str] = mapped_column(String(20), nullable=False)
    note: Mapped[Optional[str]] = mapped_column(String(500))
    created_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )
    resolved_at: Mapped[Optional[datetime]] = mapped_column(DateTime)
    resolved_by: Mapped[Optional[str]] = mapped_column(String(100))
    # resolution: "accepted" | "dismissed" (NULL while open)
    resolution: Mapped[Optional[str]] = mapped_column(String(20))

    __table_args__ = (
        # Speeds up both the admin queue (open reports: resolved_at IS NULL)
        # and the per-part "do I have an open report against this?" probe.
        Index("ix_part_reports_part_resolved", "part_id", "resolved_at"),
    )


class PartMergeProposal(Base):
    __tablename__ = "part_merge_proposals"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    source_part_id: Mapped[int] = mapped_column(
        ForeignKey("parts.id", ondelete="CASCADE"), nullable=False, index=True
    )
    target_part_id: Mapped[int] = mapped_column(
        ForeignKey("parts.id", ondelete="CASCADE"), nullable=False, index=True
    )
    proposer_username: Mapped[str] = mapped_column(String(100), nullable=False)
    rationale: Mapped[str] = mapped_column(String(2000), nullable=False)
    created_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )
    resolved_at: Mapped[Optional[datetime]] = mapped_column(DateTime)
    # outcome: "merged" | "rejected" | "withdrawn" (NULL while open)
    outcome: Mapped[Optional[str]] = mapped_column(String(20))

    __table_args__ = (
        # Sort the open-proposals list newest-first; the same index covers
        # "all resolved proposals, recent first" too.
        Index("ix_part_merge_proposals_resolved_created", "resolved_at", "created_at"),
    )


class PartMergeVote(Base):
    __tablename__ = "part_merge_votes"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    proposal_id: Mapped[int] = mapped_column(
        ForeignKey("part_merge_proposals.id", ondelete="CASCADE"),
        nullable=False,
        index=True,
    )
    voter_username: Mapped[str] = mapped_column(String(100), nullable=False)
    # vote: "approve" | "reject"
    vote: Mapped[str] = mapped_column(String(10), nullable=False)
    voted_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )

    __table_args__ = (
        # One vote per voter per proposal — changing your mind UPDATEs the
        # existing row (handled in the router).
        UniqueConstraint(
            "proposal_id", "voter_username", name="uq_part_merge_votes_proposal_voter"
        ),
    )


class PartRevision(Base):
    __tablename__ = "part_revisions"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    part_id: Mapped[int] = mapped_column(
        ForeignKey("parts.id", ondelete="CASCADE"), nullable=False, index=True
    )
    author: Mapped[str] = mapped_column(String(100), nullable=False)
    created_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )
    change_summary: Mapped[str] = mapped_column(String(200), nullable=False)
    # Snapshot of the editable fields at the time of the revision.
    name: Mapped[str] = mapped_column(String(200), nullable=False)
    sku: Mapped[Optional[str]] = mapped_column(String(100))
    mpn: Mapped[Optional[str]] = mapped_column(String(100))
    description_md: Mapped[Optional[str]] = mapped_column(Text)
    image_url: Mapped[Optional[str]] = mapped_column(Text)
    tags: Mapped[Optional[list]] = mapped_column(TagsType)
    # Issue #135: snapshot category + family in the revision history so we
    # can roll them back along with the other editable fields. Older
    # revision rows pre-dating #135 will have NULLs here, which is fine.
    category: Mapped[Optional[str]] = mapped_column(String(60))
    family: Mapped[Optional[str]] = mapped_column(String(80))
    # Suppliers are denormalised into the revision row as JSON so we don't
    # need a sibling table per revision. Each entry: {name, url}.
    suppliers_json: Mapped[Optional[list]] = mapped_column(JsonType)


# --- User flags (issue #136 — mass-deletion auto-disable) -----------------
#
# We don't own a real user table — identity comes from the Chatter JWT. But
# we *do* need a place to flag locally-relevant state like "this user has
# been auto-disabled for mass-deleting wiki content". One row per username,
# lazily inserted the first time we need to set a flag.
#
# `is_disabled=True` is checked by `get_current_user` and `get_optional_user`
# in `auth.py`; any request from a disabled account 403s before reaching a
# router.


class User(Base):
    __tablename__ = "users"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    username: Mapped[str] = mapped_column(
        String(100), nullable=False, unique=True, index=True
    )
    is_disabled: Mapped[bool] = mapped_column(
        Boolean, nullable=False, default=False, server_default="0"
    )
    disabled_reason: Mapped[Optional[str]] = mapped_column(Text)
    disabled_at: Mapped[Optional[datetime]] = mapped_column(DateTime)
    created_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )


# --- Staff picks (issue #115) --------------------------------------------
#
# Editorial collections of projects ("May 2026 Staff Picks", "Holiday Build
# Round-up", etc.). Staff picks are independent of the per-project
# ``is_featured`` flag — a project may be in many picks across time, or in
# none, regardless of its featured state. The pick row itself is a draft
# until ``is_published`` flips true.
#
# No migration helper is needed: ``create_all`` builds these brand-new
# tables on every deploy. Only column-additions to existing tables need a
# helper (see ``add_remix_columns_if_missing``).


class StaffPick(Base):
    __tablename__ = "staff_picks"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    title: Mapped[str] = mapped_column(String(200), nullable=False)
    description: Mapped[Optional[str]] = mapped_column(String(500))
    period_start: Mapped[Optional[datetime]] = mapped_column(Date)
    period_end: Mapped[Optional[datetime]] = mapped_column(Date)
    created_by: Mapped[str] = mapped_column(String(100), nullable=False)
    created_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )
    is_published: Mapped[bool] = mapped_column(
        Boolean, nullable=False, default=False, server_default="0", index=True
    )
    cover_image_url: Mapped[Optional[str]] = mapped_column(Text)


class StaffPickItem(Base):
    __tablename__ = "staff_pick_items"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    staff_pick_id: Mapped[int] = mapped_column(
        ForeignKey("staff_picks.id", ondelete="CASCADE"), nullable=False, index=True
    )
    project_id: Mapped[int] = mapped_column(
        ForeignKey("projects.id", ondelete="CASCADE"), nullable=False, index=True
    )
    editor_note: Mapped[Optional[str]] = mapped_column(String(300))
    order_index: Mapped[int] = mapped_column(Integer, nullable=False, default=0)

    __table_args__ = (
        # A project can only appear once in any given pick. The (pick, project)
        # tuple is the natural key; ``id`` is kept as a stable handle for the
        # admin UI's PATCH/DELETE endpoints so renaming a pick's items
        # doesn't require composite-key URLs.
        UniqueConstraint(
            "staff_pick_id", "project_id", name="uq_staff_pick_items_pick_project"
        ),
    )


# --- Badges & Achievements (issue #106) ----------------------------------
#
# Two tables: ``badge_definitions`` (catalog of available badges, seeded at
# startup) and ``user_badges`` (which users have earned which badges).
#
# Design decisions:
#   * ``user_id`` on ``user_badges`` is the **username string** (matching
#     ``Project.author_username``, ``Make.user_id``, etc.). There is no
#     local ``users`` table in projects-api — Chatter is the source of
#     truth. Storing the username here keeps the data model consistent with
#     the rest of this service.
#   * ``slug`` is the stable identifier for a badge across redeploys;
#     ``id`` is an internal autoinc used by FK joins. We upsert seed data
#     by slug so re-running the seeder doesn't duplicate rows.
#   * ``tier`` values: ``"bronze" | "silver" | "gold" | "single"``. Tiered
#     families share a slug **prefix** (e.g. ``prolific_maker_bronze``,
#     ``prolific_maker_silver``, ``prolific_maker_gold``); the ``category``
#     column groups them so the gallery UI can render a family card.


class BadgeDefinition(Base):
    __tablename__ = "badge_definitions"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    slug: Mapped[str] = mapped_column(String(80), nullable=False, unique=True, index=True)
    name: Mapped[str] = mapped_column(String(120), nullable=False)
    description: Mapped[str] = mapped_column(Text, nullable=False)
    icon: Mapped[str] = mapped_column(String(120), nullable=False)
    category: Mapped[str] = mapped_column(String(60), nullable=False, index=True)
    threshold_type: Mapped[str] = mapped_column(String(60), nullable=False)
    threshold_value: Mapped[int] = mapped_column(Integer, nullable=False, default=1)
    tier: Mapped[str] = mapped_column(String(20), nullable=False, default="single")
    created_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )


class UserBadge(Base):
    __tablename__ = "user_badges"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    user_id: Mapped[str] = mapped_column(String(100), nullable=False, index=True)
    badge_id: Mapped[int] = mapped_column(
        ForeignKey("badge_definitions.id", ondelete="CASCADE"),
        nullable=False,
        index=True,
    )
    earned_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )

    __table_args__ = (
        Index("ux_user_badges_user_badge", "user_id", "badge_id", unique=True),
    )


# --- Feedback (issue #138) -----------------------------------------------
#
# User-submitted feedback from the floating widget on the public site.
# The frontend (web/assets/js/feedback-widget.js + admin inbox at
# /admin/feedback) ships ahead of this model; the API spec lives at
# web/feedback/API_SPEC.md and is the contract.
#
# Identity columns mirror the rest of this service: usernames are the
# string source of truth (Chatter owns the real users table), and we
# record `user_id` as the username string. We also snapshot a separate
# `username` column for parity with the spec's response shape, even
# though they're the same value today — keeping it lets us later
# decouple "who submitted" from "who is currently logged-in if they
# rename" without a schema change.
#
# Brand-new table — no ALTER helper needed; create_all builds it on
# fresh deployments. Postgres CHECK constraints are intentionally
# omitted here; the Pydantic schemas enforce sentiment/status enums and
# the 10..2000-char message bound on the way in.


class Feedback(Base):
    __tablename__ = "feedback"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    # Username of the submitter (Chatter is the source of truth for the
    # real user record). Stored as a plain string so we don't carry a
    # local users table; matches Project.author_username / Make.user_id.
    user_id: Mapped[str] = mapped_column(String(100), nullable=False, index=True)
    username: Mapped[str] = mapped_column(String(100), nullable=False)
    sentiment: Mapped[str] = mapped_column(String(20), nullable=False, index=True)
    message: Mapped[str] = mapped_column(Text, nullable=False)
    email: Mapped[Optional[str]] = mapped_column(String(320))
    status: Mapped[str] = mapped_column(
        String(20), nullable=False, default="unread", server_default="unread", index=True
    )
    page_url: Mapped[str] = mapped_column(Text, nullable=False)
    referrer: Mapped[Optional[str]] = mapped_column(Text)
    user_agent: Mapped[Optional[str]] = mapped_column(Text)
    viewport: Mapped[Optional[str]] = mapped_column(String(40))
    # `screenshot_path` is the on-disk / NAS key returned by storage.save_file.
    # `screenshot_url` is the long-lived public URL the admin UI can render.
    screenshot_path: Mapped[Optional[str]] = mapped_column(Text)
    screenshot_url: Mapped[Optional[str]] = mapped_column(Text)
    # Admin bookkeeping for the "mark read" workflow.
    read_at: Mapped[Optional[datetime]] = mapped_column(DateTime)
    read_by_user_id: Mapped[Optional[str]] = mapped_column(String(100))
    created_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )
    updated_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )

    __table_args__ = (
        Index("ix_feedback_status_created", "status", "created_at"),
        Index("ix_feedback_user_created", "user_id", "created_at"),
    )
