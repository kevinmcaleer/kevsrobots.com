"""Pydantic response models."""

from __future__ import annotations

from datetime import datetime
from typing import Optional

from pydantic import BaseModel, Field


class HealthResponse(BaseModel):
    """Payload returned by `GET /health`."""

    status: str = "ok"
    content_count: int = 0
    recommendation_count: int = 0


class IngestStats(BaseModel):
    """Outcome of an ingestion run."""

    added: int = 0
    updated: int = 0
    unchanged: int = 0
    skipped: int = 0
    removed: int = 0
    errors: int = 0
    files_processed: list[str] = Field(default_factory=list)
    error_details: list[str] = Field(default_factory=list)


class GenerationStats(BaseModel):
    """Outcome of a recommendations generation run (#74)."""

    sources_processed: int = 0
    recommendations_written: int = 0
    sources_skipped: int = 0
    removed_orphans: int = 0
    duration_ms: int = 0
    generator_version: str = "heuristic-v0"


class RecommendationItem(BaseModel):
    """A single recommendation returned to the widget."""

    content_id: int
    url: str
    title: str
    type: str
    reason: str
    score: Optional[float] = None
    # What a page needs to draw a card (the read-next section on posts).
    description: Optional[str] = None
    cover: Optional[str] = None
    date: Optional[str] = None
    # The YouTube video a card should offer: a post's first `videos:` id,
    # or the video itself. The site is paid for by these, so always sent.
    video_id: Optional[str] = None
    level: Optional[str] = None
    duration: Optional[str] = None


class SeenPage(BaseModel):
    """One page an anonymous reader's browser says they read."""

    page: str = Field(..., max_length=300)
    # When, as epoch milliseconds (what the browser stores). Missing = now.
    at: Optional[int] = None


class RecommendationsRequest(BaseModel):
    """Body of `POST /api/recommendations`.

    The POST form exists so a reader's local history travels in the body,
    which access logs never record, rather than in the query string.
    """

    page: str = Field(..., max_length=300)
    limit: int = Field(5, ge=1, le=6)
    exclude: list[int] = Field(default_factory=list, max_length=50)
    # Only recommend these content types ("post", "video", ...). Empty = any.
    types: list[str] = Field(default_factory=list, max_length=10)
    # Pages read in this browser, newest first. Used for this answer only.
    seen: list[SeenPage] = Field(default_factory=list, max_length=100)
    # Also return up to this many related courses.
    courses: int = Field(0, ge=0, le=3)


class RecommendationsResponse(BaseModel):
    """Payload returned by `GET /api/recommendations`."""

    source_url: str
    recommendations: list[RecommendationItem] = Field(default_factory=list)
    generated_at: Optional[datetime] = None
    generator_version: Optional[str] = None
    # What the picks are based on: "page" (the page itself), "course" (a
    # lesson mapped to its course) or "fallback" (trending for the section).
    basis: Optional[str] = None
    # True when the request carried a valid login; the widget uses it to
    # decide whether to report the page view.
    logged_in: bool = False
    # True when the user's viewing history shaped these picks.
    personalised: bool = False
    # Related courses, when the request asked for them.
    courses: list[RecommendationItem] = Field(default_factory=list)


class AdminStatus(BaseModel):
    """Summary for the admin portal Nibsy panel (#158).

    Cheap counts + last-generated timestamp so the admin page can show
    "what state is the recommender in?" without paging through tables.
    """

    content_count: int = 0
    recommendation_count: int = 0
    last_generated_at: Optional[datetime] = None
