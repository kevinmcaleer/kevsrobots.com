"""SQLAlchemy models for the Nibsy API.

Schema covers issue #66 (content, clicks, impressions) and adds the
precomputed recommendations table from #73a so we land both at once and
avoid a follow-up migration.

The Postgres-flavoured columns (`ARRAY`, `JSONB`) fall back to portable
SQLite-friendly types when the engine is SQLite. This lets the pytest
suite run on `aiosqlite` without a real Postgres.
"""

from __future__ import annotations

from datetime import datetime
from typing import Any, Optional

from sqlalchemy import (
    DateTime,
    ForeignKey,
    Integer,
    String,
    Text,
    func,
)
from sqlalchemy.dialects.postgresql import ARRAY, JSONB
from sqlalchemy.orm import Mapped, mapped_column
from sqlalchemy.types import JSON

from .db import Base


# Use JSONB on Postgres, plain JSON elsewhere (SQLite for tests).
JsonType = JSONB().with_variant(JSON(), "sqlite")
# Use Postgres TEXT[] on Postgres, JSON-encoded list on SQLite.
TagsType = ARRAY(Text).with_variant(JSON(), "sqlite")


class NibsyContent(Base):
    """A piece of content (course, post, video, robot, review, glossary term)."""

    __tablename__ = "nibsy_content"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    content_type: Mapped[str] = mapped_column(String(50), nullable=False)
    title: Mapped[str] = mapped_column(Text, nullable=False)
    # URL is the natural upsert key — UNIQUE so `INSERT ... ON CONFLICT` keys cleanly.
    url: Mapped[str] = mapped_column(Text, nullable=False, unique=True)
    description: Mapped[Optional[str]] = mapped_column(Text)
    tags: Mapped[Optional[list[str]]] = mapped_column(TagsType)
    date_published: Mapped[Optional[datetime]] = mapped_column(DateTime)
    content_metadata: Mapped[Optional[dict[str, Any]]] = mapped_column(
        "metadata", JsonType
    )
    created_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )
    updated_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )


class NibsyClick(Base):
    """A click on a recommended item (populated by #68's tracking endpoint)."""

    __tablename__ = "nibsy_clicks"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    content_id: Mapped[Optional[int]] = mapped_column(
        ForeignKey("nibsy_content.id", ondelete="SET NULL")
    )
    content_url: Mapped[str] = mapped_column(Text, nullable=False)
    source_page: Mapped[str] = mapped_column(Text, nullable=False)
    session_id: Mapped[Optional[str]] = mapped_column(String(100))
    ip_address: Mapped[Optional[str]] = mapped_column(String(45))
    user_agent: Mapped[Optional[str]] = mapped_column(Text)
    clicked_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )


class NibsyImpression(Base):
    """An impression of a recommendation card (populated by #68)."""

    __tablename__ = "nibsy_impressions"

    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    content_id: Mapped[Optional[int]] = mapped_column(
        ForeignKey("nibsy_content.id", ondelete="SET NULL")
    )
    source_page: Mapped[str] = mapped_column(Text, nullable=False)
    session_id: Mapped[Optional[str]] = mapped_column(String(100))
    shown_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )


class NibsyTrending(Base):
    """Precomputed trending scores per content item (#72)."""

    __tablename__ = "nibsy_trending"

    content_id: Mapped[int] = mapped_column(
        ForeignKey("nibsy_content.id", ondelete="CASCADE"), primary_key=True
    )
    trending_score: Mapped[float] = mapped_column(nullable=False, default=0.0)
    nibsy_clicks: Mapped[int] = mapped_column(Integer, nullable=False, default=0)
    page_views: Mapped[int] = mapped_column(Integer, nullable=False, default=0)
    youtube_views: Mapped[int] = mapped_column(Integer, nullable=False, default=0)
    computed_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )


class NibsyRecommendation(Base):
    """Precomputed top-N recommendations per source URL (#73a).

    The generator job (#73a/#73b) populates this offline; the read path
    (#67) reads from it. Neither is in scope for this PR — only the table
    lands here so the next PRs can write/read without a migration.
    """

    __tablename__ = "nibsy_recommendations"

    # The page being read — primary key so upserts replace in place.
    source_url: Mapped[str] = mapped_column(Text, primary_key=True)
    # Ordered array of {content_id, url, title, type, reason}
    recommendations: Mapped[list[dict[str, Any]]] = mapped_column(
        JsonType, nullable=False
    )
    generated_at: Mapped[datetime] = mapped_column(
        DateTime, server_default=func.now(), nullable=False
    )
    # Bump when the algorithm changes so consumers can invalidate caches.
    generator_version: Mapped[str] = mapped_column(
        Text, nullable=False, server_default="v0"
    )
