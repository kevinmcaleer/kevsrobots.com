"""Pydantic request/response models."""

from __future__ import annotations

from datetime import datetime
from typing import Optional

from pydantic import BaseModel, Field


class ProjectCreate(BaseModel):
    title: str = Field(..., min_length=5, max_length=200)
    short_description: Optional[str] = Field(None, max_length=500)
    content_md: Optional[str] = None
    difficulty: Optional[str] = Field(None, pattern="^(beginner|intermediate|advanced)$")
    estimated_minutes: Optional[int] = Field(None, ge=1)
    code_repo_url: Optional[str] = None
    tags: list[str] = Field(default_factory=list)


class ProjectUpdate(BaseModel):
    title: Optional[str] = Field(None, min_length=5, max_length=200)
    short_description: Optional[str] = Field(None, max_length=500)
    content_md: Optional[str] = None
    difficulty: Optional[str] = Field(None, pattern="^(beginner|intermediate|advanced)$")
    estimated_minutes: Optional[int] = Field(None, ge=1)
    code_repo_url: Optional[str] = None
    status: Optional[str] = Field(None, pattern="^(wip|completed|archived)$")
    cover_image: Optional[str] = None
    tags: Optional[list[str]] = None


class RemixedFromRef(BaseModel):
    """Minimal reference to a parent project (issue #108).

    Embedded inside :class:`ProjectResponse` when the project is a remix.
    ``slug`` is reserved for when project slugs land — keep optional now.
    """

    id: int
    slug: Optional[str] = None
    title: str
    author_username: str


class ProjectResponse(BaseModel):
    id: int
    title: str
    short_description: Optional[str]
    content_md: Optional[str]
    difficulty: Optional[str]
    estimated_minutes: Optional[int]
    code_repo_url: Optional[str]
    status: str
    author_username: str
    cover_image: Optional[str]
    tags: list[str] = Field(default_factory=list)
    created_at: datetime
    updated_at: datetime
    # Issue #108: remix / fork attribution.
    remixed_from: Optional[RemixedFromRef] = None
    remix_description: Optional[str] = None
    remixes_count: int = 0
    is_remix: bool = False


class ProjectListItem(BaseModel):
    id: int
    title: str
    short_description: Optional[str]
    difficulty: Optional[str]
    estimated_minutes: Optional[int]
    status: str
    author_username: str
    cover_image: Optional[str]
    tags: list[str] = Field(default_factory=list)
    created_at: datetime
    # Issue #108: surface remix indicator on list cards.
    is_remix: bool = False


class ProjectRemixCreate(BaseModel):
    """Body for ``POST /api/projects/{id}/remix`` (issue #108).

    ``remix_description`` is mandatory and must be at least 10 characters
    so the resulting attribution actually documents what changed.
    """

    title: Optional[str] = Field(None, min_length=5, max_length=200)
    remix_description: str = Field(..., min_length=10, max_length=2000)


class BOMItemCreate(BaseModel):
    name: str = Field(..., max_length=200)
    quantity: int = Field(1, ge=1)
    unit: str = Field("qty", max_length=20)
    unit_cost: Optional[float] = None
    supplier_url: Optional[str] = None
    sort_order: int = 0


class BOMItemResponse(BaseModel):
    id: int
    name: str
    quantity: int
    unit: str
    unit_cost: Optional[float]
    supplier_url: Optional[str]
    sort_order: int


class LinkCreate(BaseModel):
    title: str = Field(..., max_length=200)
    url: str
    link_type: str = Field("article", pattern="^(article|video|tutorial|documentation|other)$")


class LinkResponse(BaseModel):
    id: int
    title: str
    url: str
    link_type: str


class JournalEntryCreate(BaseModel):
    title: str = Field(..., max_length=200)
    content_md: Optional[str] = None
    status: str = Field("in_progress", pattern="^(planning|in_progress|completed)$")


class JournalEntryResponse(BaseModel):
    id: int
    title: str
    content_md: Optional[str]
    status: str
    created_at: datetime
    updated_at: datetime


class FileResponse(BaseModel):
    id: int
    filename: str
    file_size: int
    file_type: str
    description: Optional[str]
    uploaded_at: datetime


class ImageResponse(BaseModel):
    id: int
    filename: str
    caption: Optional[str]
    sort_order: int
    uploaded_at: datetime


# --- Moderation schemas ---


class ReportCreate(BaseModel):
    reason: str = Field(..., min_length=5, max_length=1000)


class ReportResponse(BaseModel):
    id: int
    project_id: int
    reporter_username: str
    reason: str
    status: str
    created_at: datetime
    reviewed_at: Optional[datetime] = None
    reviewed_by: Optional[str] = None


class ReportUpdate(BaseModel):
    status: str = Field(..., pattern="^(reviewed|dismissed)$")


class ModerationNoteUpdate(BaseModel):
    moderation_note: Optional[str] = None


class BlockedProjectResponse(BaseModel):
    id: int
    title: str
    author_username: str
    is_blocked: bool
    moderation_note: Optional[str]
    created_at: datetime
