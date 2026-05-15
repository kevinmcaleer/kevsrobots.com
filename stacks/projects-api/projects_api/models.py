"""SQLAlchemy models for the Projects API."""

from __future__ import annotations

from datetime import datetime
from typing import Any, Optional

from sqlalchemy import (
    ARRAY,
    Boolean,
    DateTime,
    Float,
    ForeignKey,
    Index,
    Integer,
    String,
    Text,
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
    created_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )
    updated_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
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
    status: Mapped[str] = mapped_column(String(30), nullable=False, default="draft")
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
    # Suppliers are denormalised into the revision row as JSON so we don't
    # need a sibling table per revision. Each entry: {name, url}.
    suppliers_json: Mapped[Optional[list]] = mapped_column(JsonType)
