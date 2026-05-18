"""Pydantic request/response models."""

from __future__ import annotations

from datetime import date, datetime
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
    download_count: int = 0
    # Issue #115: surface featured state on the detail view so the project
    # page can render the "Staff Pick" badge near the title.
    is_featured: bool = False
    featured_at: Optional[datetime] = None
    featured_by: Optional[str] = None
    featured_note: Optional[str] = None


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
    download_count: int = 0
    # Issue #115: surface featured indicator on list cards so the hub can
    # render the gold ribbon without a second API call per project.
    is_featured: bool = False
    featured_note: Optional[str] = None


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
    part_id: Optional[int] = None


class BOMItemResponse(BaseModel):
    id: int
    name: str
    quantity: int
    unit: str
    unit_cost: Optional[float]
    supplier_url: Optional[str]
    sort_order: int
    part_id: Optional[int] = None
    # Populated when ``part_id`` is set so the BOM row can link to
    # ``/parts/view.html?slug=…`` and fall back to the part's catalogued
    # supplier if the row's own ``supplier_url`` is empty.
    part_slug: Optional[str] = None
    part_primary_supplier_url: Optional[str] = None


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
    download_count: int = 0


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


# --- Community Makes ("I Made This!") -- issue #107 -----------------------


class MakeCreate(BaseModel):
    notes: Optional[str] = Field(None, max_length=5000)
    modifications: Optional[str] = Field(None, max_length=5000)


class MakeImageResponse(BaseModel):
    id: int
    filename: str
    caption: Optional[str] = None
    sort_order: int
    file_size: int


class MakeResponse(BaseModel):
    id: int
    project_id: int
    user_id: str
    created_at: datetime
    notes: Optional[str] = None
    modifications: Optional[str] = None
    images: list[MakeImageResponse] = Field(default_factory=list)
    hearted_by_author: bool = False
    # Populated only when returning a single make (detail view); harmless
    # when omitted from list responses.
    project_title: Optional[str] = None


# --- Download tracking schemas ---


class DownloadLogResponse(BaseModel):
    """Returned when a download is logged. ``dedup`` is True when the
    request was within the 24h dedup window for the same identity."""

    logged: bool
    dedup: bool


class FileDownloadStats(BaseModel):
    file_id: int
    filename: str
    total: int
    last_7d: int
    last_30d: int


class DailyDownloadCount(BaseModel):
    date: str  # ISO date, YYYY-MM-DD
    count: int


class ProjectDownloadStats(BaseModel):
    project_id: int
    total: int
    last_7d: int
    last_30d: int
    per_file: list[FileDownloadStats] = Field(default_factory=list)
    daily: list[DailyDownloadCount] = Field(default_factory=list)


class PopularProjectItem(BaseModel):
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
    download_count: int = 0


# --- Parts catalog (issue #121) ---


class PartSupplierInput(BaseModel):
    name: Optional[str] = Field(None, max_length=120)
    url: str = Field(..., max_length=2000)


class PartSupplierResponse(BaseModel):
    id: int
    name: Optional[str] = None
    url: str
    last_checked_at: Optional[datetime] = None
    last_status: Optional[str] = None


class PartCreate(BaseModel):
    name: str = Field(..., min_length=2, max_length=200)
    sku: Optional[str] = Field(None, max_length=100)
    mpn: Optional[str] = Field(None, max_length=100)
    description_md: Optional[str] = None
    image_url: Optional[str] = Field(None, max_length=2000)
    supplier_url: Optional[str] = Field(None, max_length=2000)
    supplier_name: Optional[str] = Field(None, max_length=120)
    tags: list[str] = Field(default_factory=list)


class PartUpdate(BaseModel):
    name: Optional[str] = Field(None, min_length=2, max_length=200)
    sku: Optional[str] = Field(None, max_length=100)
    mpn: Optional[str] = Field(None, max_length=100)
    description_md: Optional[str] = None
    image_url: Optional[str] = Field(None, max_length=2000)
    tags: Optional[list[str]] = None
    suppliers: Optional[list[PartSupplierInput]] = None
    change_summary: str = Field(..., min_length=1, max_length=200)


class PartSearchResult(BaseModel):
    id: int
    slug: str
    name: str
    sku: Optional[str] = None
    status: str
    usage_count: int
    primary_supplier_url: Optional[str] = None


class PartRevisionSummary(BaseModel):
    id: int
    author: str
    created_at: datetime
    change_summary: str


class PartRevisionDetail(BaseModel):
    id: int
    author: str
    created_at: datetime
    change_summary: str
    name: str
    sku: Optional[str] = None
    mpn: Optional[str] = None
    description_md: Optional[str] = None
    image_url: Optional[str] = None
    tags: list[str] = Field(default_factory=list)
    suppliers: list[PartSupplierInput] = Field(default_factory=list)


class PartDetail(BaseModel):
    id: int
    slug: str
    name: str
    sku: Optional[str] = None
    mpn: Optional[str] = None
    description_md: Optional[str] = None
    image_url: Optional[str] = None
    tags: list[str] = Field(default_factory=list)
    status: str
    created_by: str
    created_at: datetime
    updated_at: datetime
    current_revision_id: Optional[int] = None
    usage_count: int
    suppliers: list[PartSupplierResponse] = Field(default_factory=list)
    recent_revisions: list[PartRevisionSummary] = Field(default_factory=list)


# --- Featured projects + staff picks (issue #115) ------------------------


class FeatureProjectRequest(BaseModel):
    """Body for ``POST /api/admin/projects/{id}/feature``.

    ``note`` is the editor's reason for featuring (e.g. "Beautifully
    documented servo walker build"). The 200-char cap matches
    ``Project.featured_note``; longer notes are rejected by both pydantic
    here and SQLAlchemy at insert time.
    """

    note: Optional[str] = Field(None, max_length=200)


class FeaturedProjectResponse(BaseModel):
    """Returned by ``GET /api/projects/featured`` and the admin toggle.

    Same shape as :class:`ProjectListItem` so the hub can render a
    featured carousel using the same card template, plus the editorial
    metadata. ``featured_at`` is when the admin promoted the project so
    callers can sort "newest featured first".
    """

    id: int
    title: str
    short_description: Optional[str] = None
    difficulty: Optional[str] = None
    estimated_minutes: Optional[int] = None
    status: str
    author_username: str
    cover_image: Optional[str] = None
    tags: list[str] = Field(default_factory=list)
    created_at: datetime
    is_featured: bool = True
    featured_at: Optional[datetime] = None
    featured_by: Optional[str] = None
    featured_note: Optional[str] = None


class StaffPickCreate(BaseModel):
    title: str = Field(..., min_length=3, max_length=200)
    description: Optional[str] = Field(None, max_length=500)
    period_start: Optional[date] = None
    period_end: Optional[date] = None
    cover_image_url: Optional[str] = Field(None, max_length=2000)
    is_published: bool = False


class StaffPickUpdate(BaseModel):
    title: Optional[str] = Field(None, min_length=3, max_length=200)
    description: Optional[str] = Field(None, max_length=500)
    period_start: Optional[date] = None
    period_end: Optional[date] = None
    cover_image_url: Optional[str] = Field(None, max_length=2000)
    is_published: Optional[bool] = None


class StaffPickItemCreate(BaseModel):
    project_id: int
    editor_note: Optional[str] = Field(None, max_length=300)
    order_index: Optional[int] = Field(None, ge=0)


class StaffPickItemUpdate(BaseModel):
    editor_note: Optional[str] = Field(None, max_length=300)
    order_index: Optional[int] = Field(None, ge=0)


class StaffPickProjectRef(BaseModel):
    """Minimal project reference embedded inside a staff pick item."""

    id: int
    title: str
    short_description: Optional[str] = None
    author_username: str
    cover_image: Optional[str] = None
    difficulty: Optional[str] = None
    status: str
    is_featured: bool = False


class StaffPickItemResponse(BaseModel):
    """An entry in a staff pick. ``project`` is denormalised so the public
    page can render the card without a second round-trip per item."""

    id: int
    project_id: int
    editor_note: Optional[str] = None
    order_index: int
    project: Optional[StaffPickProjectRef] = None


class StaffPickResponse(BaseModel):
    id: int
    title: str
    description: Optional[str] = None
    period_start: Optional[date] = None
    period_end: Optional[date] = None
    created_by: str
    created_at: datetime
    is_published: bool
    cover_image_url: Optional[str] = None
    item_count: int = 0


class StaffPickDetail(StaffPickResponse):
    """Single-pick view that also embeds the ordered list of items."""

    items: list[StaffPickItemResponse] = Field(default_factory=list)
