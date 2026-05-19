"""Pydantic request/response models."""

from __future__ import annotations

from datetime import date, datetime
from typing import Optional

from pydantic import BaseModel, Field


# --- Badges & Achievements (issue #106) ----------------------------------
# Defined up here because ProjectResponse / MakeResponse reference
# EarnedBadgeResponse in their ``newly_awarded_badges`` field.


class BadgeDefinitionResponse(BaseModel):
    """One row in the badge catalog. Public — exposed via /api/badges."""

    id: int
    slug: str
    name: str
    description: str
    icon: str
    category: str
    threshold_type: str
    threshold_value: int
    tier: str  # "bronze" | "silver" | "gold" | "single"


class EarnedBadgeResponse(BaseModel):
    """A badge a user has earned. Combines the catalog row + earned_at."""

    id: int
    slug: str
    name: str
    description: str
    icon: str
    category: str
    tier: str
    earned_at: datetime


class BadgeEvaluationResponse(BaseModel):
    """Returned by POST /api/badges/evaluate/{username}.

    ``newly_awarded`` is the list of badges that crossed the threshold on
    *this* evaluation pass — callers (frontend or another API route) can
    use it to show "you earned a new badge!" toast notifications.
    """

    username: str
    newly_awarded: list[EarnedBadgeResponse] = Field(default_factory=list)
    total_earned: int = 0


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
    # Issue #152: when ``regenerate_slug`` is true, the server picks a new
    # slug derived from the (possibly updated) title. Existing slugs are
    # sticky by default — renaming a project does NOT change its URL
    # unless the user explicitly opts in. This keeps inbound links from
    # blogs / search engines stable until the author asks for a refresh.
    regenerate_slug: Optional[bool] = None


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
    # Issue #152: URL-friendly slug, unique per author. Combined with
    # ``author_username`` forms the canonical ``/projects/<owner>/<slug>``
    # URL on the frontend.
    slug: Optional[str] = None
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
    # Issue #106: badges awarded as a side-effect of this request (e.g.
    # project create crossing the "First Project" threshold). Frontend uses
    # this to surface a toast notification. Always omitted from
    # response bodies that didn't trigger evaluation.
    newly_awarded_badges: list[EarnedBadgeResponse] = Field(default_factory=list)


class ProjectListItem(BaseModel):
    id: int
    # Issue #152: surface slug on list cards so the hub / search / profile
    # pages can emit ``/projects/<owner>/<slug>`` URLs without an extra
    # per-card fetch. Purely additive — clients that don't read it keep
    # working.
    slug: Optional[str] = None
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
    # Issue #149: ISO 4217 currency code (e.g. ``GBP``, ``USD``, ``EUR``).
    # Optional for back-compat — legacy rows without a code render as a
    # raw number on the frontend with a "currency not set" tooltip. The
    # pattern is strict (exactly three uppercase letters); the dropdown
    # in the editor only offers a handful of common codes plus
    # "other / unknown" → NULL.
    currency_code: Optional[str] = Field(None, pattern=r"^[A-Z]{3}$")
    supplier_url: Optional[str] = None
    sort_order: int = 0
    part_id: Optional[int] = None


class BOMItemResponse(BaseModel):
    id: int
    name: str
    quantity: int
    unit: str
    unit_cost: Optional[float]
    currency_code: Optional[str] = None
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
    # Issue #106: badges awarded as a side-effect of posting this make.
    newly_awarded_badges: list[EarnedBadgeResponse] = Field(default_factory=list)


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
    # Issue #152: surface slug so the popular row can emit canonical URLs.
    slug: Optional[str] = None
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
    # Issue #149: ISO 3166-1 alpha-2 country code for the supplier (e.g.
    # ``GB`` for Pimoroni, ``US`` for Adafruit). Optional — NULL means
    # "global / unknown". The pattern enforces exactly two uppercase
    # letters; the dropdown on the edit page only offers a curated list
    # plus "other / unknown" → NULL.
    country_code: Optional[str] = Field(None, pattern=r"^[A-Z]{2}$")


class PartSupplierResponse(BaseModel):
    id: int
    name: Optional[str] = None
    url: str
    last_checked_at: Optional[datetime] = None
    last_status: Optional[str] = None
    country_code: Optional[str] = None


class PartCreate(BaseModel):
    name: str = Field(..., min_length=2, max_length=200)
    sku: Optional[str] = Field(None, max_length=100)
    mpn: Optional[str] = Field(None, max_length=100)
    description_md: Optional[str] = None
    image_url: Optional[str] = Field(None, max_length=2000)
    supplier_url: Optional[str] = Field(None, max_length=2000)
    supplier_name: Optional[str] = Field(None, max_length=120)
    tags: list[str] = Field(default_factory=list)
    # Issue #135: optional taxonomy + family fields at create time.
    category: Optional[str] = Field(None, max_length=60)
    family: Optional[str] = Field(None, max_length=80)


class PartUpdate(BaseModel):
    name: Optional[str] = Field(None, min_length=2, max_length=200)
    sku: Optional[str] = Field(None, max_length=100)
    mpn: Optional[str] = Field(None, max_length=100)
    description_md: Optional[str] = None
    image_url: Optional[str] = Field(None, max_length=2000)
    tags: Optional[list[str]] = None
    suppliers: Optional[list[PartSupplierInput]] = None
    change_summary: str = Field(..., min_length=1, max_length=200)
    # Issue #135: optional taxonomy + family fields. Use an explicit empty
    # string to clear; ``None`` means "leave unchanged" (consistent with the
    # other Optional fields above).
    category: Optional[str] = Field(None, max_length=60)
    family: Optional[str] = Field(None, max_length=80)


class PartSearchResult(BaseModel):
    id: int
    slug: str
    name: str
    sku: Optional[str] = None
    status: str
    usage_count: int
    primary_supplier_url: Optional[str] = None
    # Issue #135: expose category/family on search results so the parts
    # list / autocomplete UIs can show them without a second roundtrip.
    category: Optional[str] = None
    family: Optional[str] = None


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
    category: Optional[str] = None
    family: Optional[str] = None


class PartRelatedRef(BaseModel):
    """Minimal reference to another part (issue #135).

    Used for the "Related parts" + "Same family" lists on the detail view.
    """

    id: int
    slug: str
    name: str
    sku: Optional[str] = None
    status: str
    category: Optional[str] = None
    family: Optional[str] = None


class PartRelationCreate(BaseModel):
    """Body for ``POST /api/parts/{slug}/relations`` (issue #135).

    Identify the other part by slug (the UI deals in slugs) — the server
    resolves to an id and writes the canonical ordered pair.
    """

    related_slug: str = Field(..., min_length=1, max_length=120)


# --- User follows (issue #140) ---


class FollowToggleResponse(BaseModel):
    """Returned by POST/DELETE /api/users/{username}/follow."""

    follower: str
    followee: str
    following: bool


class FollowCountResponse(BaseModel):
    username: str
    count: int


class FollowingListResponse(BaseModel):
    """Returned by GET /api/users/me/following."""

    follower: str
    following: list[str] = Field(default_factory=list)


# --- User badges (issue #140) ---
#
# Badges are computed on-the-fly from existing data — we do not persist
# any earned-badge state. The catalog below is fixed; if you change the
# rules, bump the catalog version so older browsers' caches refresh.


class BadgeResponse(BaseModel):
    key: str
    name: str
    description: str
    icon: str  # Font Awesome class fragment, e.g. "fa-rocket"
    color: str = "primary"  # Bootstrap colour token


class UserBadgesResponse(BaseModel):
    username: str
    badges: list[BadgeResponse] = Field(default_factory=list)


# --- User profiles (issue #111) ---
#
# Public profile shape. ``stats`` is an aggregated snapshot of the user's
# activity. All fields are nullable / default-zero so the renderer can
# rely on the schema staying consistent for users who haven't edited
# their profile.


class UserSocialLinks(BaseModel):
    github: Optional[str] = None
    twitter: Optional[str] = None
    youtube: Optional[str] = None
    mastodon: Optional[str] = None


class UserProfileStats(BaseModel):
    projects: int = 0
    makes: int = 0
    downloads: int = 0
    likes: int = 0
    followers: int = 0
    following: int = 0


class UserProfileResponse(BaseModel):
    username: str
    avatar_url: Optional[str] = None
    bio: Optional[str] = None
    location: Optional[str] = None
    website_url: Optional[str] = None
    social_links: UserSocialLinks = Field(default_factory=UserSocialLinks)
    joined_at: Optional[datetime] = None
    featured_badge_slugs: list[str] = Field(default_factory=list)
    stats: UserProfileStats = Field(default_factory=UserProfileStats)
    # Issue #150: ISO 4217 code or None ("auto-detect / show native").
    preferred_currency: Optional[str] = Field(None, pattern=r"^[A-Z]{3}$")


class UserProfileUpdate(BaseModel):
    """PUT body for ``/api/users/me/profile``.

    Length / shape validation enforced server-side mirrors the client-side
    checks in ``profile-edit.js``. URL fields must be empty or
    ``http(s)://…`` — we validate in the route (not via ``HttpUrl``) so
    callers can clear a field by sending an empty string.
    """

    bio: Optional[str] = Field(None, max_length=500)
    location: Optional[str] = Field(None, max_length=120)
    website_url: Optional[str] = Field(None, max_length=200)
    social_links: Optional[UserSocialLinks] = None
    featured_badge_slugs: Optional[list[str]] = Field(None, max_length=3)
    # Issue #150: preferred display currency. ``None`` means "clear"
    # (back to auto-detect). Must be a 3-letter ISO 4217 alpha code.
    # The route validates the value is in the supported allow-list.
    preferred_currency: Optional[str] = Field(None, pattern=r"^[A-Z]{3}$")


class UserActivityItem(BaseModel):
    id: int
    kind: str
    subject_id: Optional[int] = None
    subject_title: Optional[str] = None
    subject_url: Optional[str] = None
    created_at: datetime


class UserActivityResponse(BaseModel):
    username: str
    items: list[UserActivityItem] = Field(default_factory=list)
    limit: int
    offset: int


class UserListItem(BaseModel):
    """A single user in a followers / following list."""

    username: str


class UserListResponse(BaseModel):
    username: str
    users: list[UserListItem] = Field(default_factory=list)
    total: int
    limit: int
    offset: int


class FollowCheckResponse(BaseModel):
    """Returned by GET /api/users/me/follows/{username}."""

    following: bool


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
    # Issue #135
    category: Optional[str] = None
    family: Optional[str] = None
    related_parts: list[PartRelatedRef] = Field(default_factory=list)
    family_parts: list[PartRelatedRef] = Field(default_factory=list)


# --- Parts moderation (issue #123, Phase 3) ------------------------------


PART_REPORT_REASONS = ("spam", "wrong", "duplicate", "other")
PART_REPORT_RESOLUTIONS = ("accepted", "dismissed")
PART_MERGE_VOTES = ("approve", "reject")


class PartReportCreate(BaseModel):
    """Body of POST /api/parts/{slug}/report."""

    reason: str = Field(..., pattern=r"^(spam|wrong|duplicate|other)$")
    note: Optional[str] = Field(None, max_length=500)


class PartReportPartRef(BaseModel):
    """Minimal part reference embedded in admin queue rows."""

    id: int
    slug: str
    name: str
    status: str


class PartReportResponse(BaseModel):
    id: int
    part_id: int
    reporter_username: str
    reason: str
    note: Optional[str] = None
    created_at: datetime
    resolved_at: Optional[datetime] = None
    resolved_by: Optional[str] = None
    resolution: Optional[str] = None
    # Embedded for admin queue rendering — None on the user-facing
    # ``POST /report`` reply since the caller already knows the part.
    part: Optional[PartReportPartRef] = None


class PartReportListResponse(BaseModel):
    items: list[PartReportResponse] = Field(default_factory=list)
    total: int = 0


class PartReportResolve(BaseModel):
    """Body of PATCH /api/admin/parts/reports/{id}."""

    resolution: str = Field(..., pattern=r"^(accepted|dismissed)$")


class PartMergeProposalCreate(BaseModel):
    """Body of POST /api/parts/{slug}/merge-proposal."""

    target_slug: str = Field(..., min_length=1, max_length=120)
    rationale: str = Field(..., min_length=10, max_length=2000)


class PartMergePartRef(BaseModel):
    """Embedded source/target on a proposal."""

    id: int
    slug: str
    name: str
    status: str


class PartMergeProposalResponse(BaseModel):
    id: int
    proposer_username: str
    rationale: str
    created_at: datetime
    resolved_at: Optional[datetime] = None
    outcome: Optional[str] = None
    source: PartMergePartRef
    target: PartMergePartRef
    approves: int = 0
    rejects: int = 0


class PartMergeProposalListResponse(BaseModel):
    items: list[PartMergeProposalResponse] = Field(default_factory=list)
    total: int = 0


class PartMergeVoteCreate(BaseModel):
    """Body of POST /api/parts/merge-proposals/{id}/vote."""

    vote: str = Field(..., pattern=r"^(approve|reject)$")


class PartMergeVoteResponse(BaseModel):
    proposal_id: int
    voter_username: str
    vote: str
    approves: int
    rejects: int
    # When auto-merge fires on this vote, the proposal flips to
    # ``outcome=merged`` and we surface that in the response so the UI can
    # show a "merged!" toast without polling.
    outcome: Optional[str] = None
    resolved_at: Optional[datetime] = None


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
    # Issue #152: surface slug so the featured carousel can emit canonical
    # /projects/<owner>/<slug> URLs.
    slug: Optional[str] = None
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
    # Issue #152: include slug so the staff-pick page can link to
    # /projects/<owner>/<slug> instead of /projects/view.html?id=.
    slug: Optional[str] = None
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


# --- Feedback (issue #138) -----------------------------------------------
#
# Schemas for the feedback API. The widget POSTs the create payload as
# JSON when there's no screenshot, or as multipart/form-data when there
# is — the router accepts both shapes and folds them into
# :class:`FeedbackCreate` before validation. The admin response shape
# matches what the inbox UI already renders (see
# web/admin/feedback.html and web/feedback/API_SPEC.md).


FEEDBACK_SENTIMENTS = ("love", "like", "issue", "idea")
FEEDBACK_STATUSES = ("unread", "read", "archived")


class FeedbackCreate(BaseModel):
    """Body of POST /api/feedback (JSON variant).

    Identity fields (``user_id``, ``username``) are intentionally
    absent here — the router resolves them from the auth dependency so
    a client can't impersonate another user by spoofing the body.
    """

    sentiment: str = Field(..., pattern=r"^(love|like|issue|idea)$")
    message: str = Field(..., min_length=10, max_length=2000)
    # Email is opt-in; Pydantic's EmailStr would force a dependency on
    # email-validator, so we keep it loose and validate the shape in the
    # router (the widget already checks client-side).
    email: Optional[str] = Field(None, max_length=320)
    page_url: str = Field(..., min_length=1, max_length=2048)
    referrer: Optional[str] = Field(None, max_length=2048)
    user_agent: Optional[str] = Field(None, max_length=500)
    viewport: Optional[str] = Field(None, max_length=40)


class FeedbackCreateResponse(BaseModel):
    """Minimal 201 payload — the widget only needs id+timestamp."""

    id: int
    status: str
    created_at: datetime


class FeedbackResponse(BaseModel):
    """Full row shape returned to admins."""

    id: int
    sentiment: str
    message: str
    email: Optional[str] = None
    username: str
    user_id: str
    status: str
    page_url: str
    referrer: Optional[str] = None
    user_agent: Optional[str] = None
    viewport: Optional[str] = None
    screenshot_url: Optional[str] = None
    read_at: Optional[datetime] = None
    read_by_user_id: Optional[str] = None
    created_at: datetime
    updated_at: datetime


class FeedbackListResponse(BaseModel):
    """Paginated list returned by GET /api/admin/feedback."""

    items: list[FeedbackResponse] = Field(default_factory=list)
    total: int = 0


class FeedbackUpdateStatus(BaseModel):
    """Body of PATCH /api/admin/feedback/{id}."""

    status: str = Field(..., pattern=r"^(unread|read|archived)$")


class FeedbackCountsResponse(BaseModel):
    """Header strip for the admin inbox."""

    total: int = 0
    unread: int = 0
    read: int = 0
    archived: int = 0
    by_sentiment: dict[str, int] = Field(
        default_factory=lambda: {"love": 0, "like": 0, "issue": 0, "idea": 0}
    )
