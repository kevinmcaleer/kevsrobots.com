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


class RecommendationsResponse(BaseModel):
    """Payload returned by `GET /api/recommendations`."""

    source_url: str
    recommendations: list[RecommendationItem] = Field(default_factory=list)
    generated_at: Optional[datetime] = None
    generator_version: Optional[str] = None
