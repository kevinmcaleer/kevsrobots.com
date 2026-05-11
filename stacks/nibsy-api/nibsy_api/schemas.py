"""Pydantic response models."""

from __future__ import annotations

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
