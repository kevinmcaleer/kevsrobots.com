"""Parts catalog endpoints (issue #121, Phase 1 MVP).

This router implements:

* ``GET    /api/parts``                 — fuzzy search across name + aliases
                                          + sku + mpn.
* ``POST   /api/parts``                 — create a draft part (14-day gated).
* ``GET    /api/parts/{slug}``          — full part incl. suppliers + last
                                          10 revisions.
* ``PUT    /api/parts/{slug}``          — write a new revision (14-day
                                          gated). Rejects if no field
                                          actually changed.
* ``GET    /api/parts/{slug}/revisions``— paginated revision list.
* ``POST   /api/parts/{slug}/revisions/{rev_id}/restore`` — non-destructive
                                          restore (writes a new revision
                                          whose snapshot equals rev_id).

Account-age + T&Cs gate
-----------------------
Mutating endpoints depend on :func:`require_terms_accepted_aged` which
layers the T&Cs acceptance gate (terms-gate) on top of
:func:`get_current_user_aged`. The age gate calls
:func:`fetch_account_created_at` against Chatter's ``/api/me``; if Chatter
is unreachable or omits the ``account_created_at`` field, it fails closed
(HTTP 403 "Account age cannot be verified"). The T&Cs gate raises a 403
with ``detail: terms_not_accepted`` until the user has POSTed the current
version to ``/api/users/me/accept-terms``. Tests monkeypatch
``fetch_account_created_at`` directly and the test client bypasses the
T&Cs gate by default (see ``tests/conftest.py``).

Usage-count semantics
---------------------
``parts.usage_count`` is a denormalised cache of *distinct projects* that
reference the part. It's maintained transactionally in
``routers/bom.py`` as BOM rows are inserted / updated / deleted. If it
ever drifts (e.g. due to a manual DB poke), a Phase-2 cron will recompute
it from ``SELECT COUNT(DISTINCT project_id) FROM project_bom_items WHERE
part_id=?``.

Search v1
---------
We use ``lower()``-substring matching against ``parts.name``,
``parts.sku``, ``parts.mpn`` and ``part_aliases.alias``. This is fine for
the small corpus we'll have in Phase 1. When the catalog grows we'll swap
this for ``pg_trgm`` (or a dedicated search index) without changing the
API contract.
"""

from __future__ import annotations

import re
from datetime import datetime, timezone
from typing import Optional

from fastapi import APIRouter, Depends, HTTPException, Query, status
from sqlalchemy import delete, func, or_, select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import require_terms_accepted_aged
from ..db import get_session
from ..mass_deletion import (
    check_and_handle_mass_deletion,
    snapshot_from_inputs,
    snapshot_from_part,
)
from ..models import (
    LibrarySymbol,
    Part,
    PartAlias,
    PartPhoto,
    PartRelation,
    PartRevision,
    PartSupplier,
)
from ..parts_lifecycle import compute_part_status
from ..parts_taxonomy import canonicalize, merge_for_suggestions
from ..schemas import (
    PartCreate,
    PartDetail,
    PartRelatedRef,
    PartRelationCreate,
    PartRevisionDetail,
    PartRevisionSummary,
    PartSearchResult,
    PartSupplierInput,
    PartPhotoResponse,
    PartSupplierResponse,
    PartSymbolRef,
    PartUpdate,
)


# Part categories are a small, self-organising vocabulary — the curated
# starter list + canonicalisation rules live in ``parts_taxonomy``.

router = APIRouter(prefix="/api/parts", tags=["parts"])


# --- helpers ------------------------------------------------------------


_SLUG_BAD_CHARS = re.compile(r"[^a-z0-9]+")


def _slugify(name: str) -> str:
    base = _SLUG_BAD_CHARS.sub("-", name.lower()).strip("-")
    # Cap at 80 chars; the DB column is 120 so disambig suffixes have room.
    return (base[:80] or "part").rstrip("-")


async def _unique_slug(session: AsyncSession, base: str) -> str:
    """Append -2, -3, ... until the slug is unique."""
    candidate = base
    suffix = 2
    while True:
        existing = await session.scalar(select(Part.id).where(Part.slug == candidate))
        if existing is None:
            return candidate
        candidate = f"{base}-{suffix}"
        suffix += 1


def _now_naive_utc() -> datetime:
    return datetime.now(timezone.utc).replace(tzinfo=None)


async def _load_suppliers(session: AsyncSession, part_id: int) -> list[PartSupplier]:
    return list(
        (
            await session.scalars(
                select(PartSupplier)
                .where(PartSupplier.part_id == part_id)
                .order_by(PartSupplier.id)
            )
        ).all()
    )


def _supplier_response(s: PartSupplier) -> PartSupplierResponse:
    return PartSupplierResponse(
        id=s.id,
        name=s.supplier_name,
        url=s.url,
        last_checked_at=s.last_checked_at,
        last_status=s.last_status,
        last_status_code=s.last_status_code,
        is_broken=bool(s.is_broken),
        consecutive_failures=int(s.consecutive_failures or 0),
        country_code=s.country_code,
        unit_cost=s.unit_cost,
        currency_code=s.currency_code,
    )


def _suppliers_to_json(suppliers: list[PartSupplier]) -> list[dict]:
    # Issue #149: persist country_code into the denormalised revision
    # snapshot so history stays consistent. Supplier-pricing feature:
    # also snapshot unit_cost + currency_code so price changes show up
    # in the next revision (and can be restored).
    return [
        {
            "name": s.supplier_name,
            "url": s.url,
            "country_code": s.country_code,
            "unit_cost": s.unit_cost,
            "currency_code": s.currency_code,
        }
        for s in suppliers
    ]


async def _recompute_usage_count(session: AsyncSession, part_id: int) -> int:
    """Recompute usage_count as ``COUNT(DISTINCT project_id)`` of BOM rows.

    Cheaper alternative used by the BOM mutation hooks (in ``bom.py``).
    Exposed here for the rare case the cache needs to be refreshed.
    """
    from ..models import ProjectBOMItem

    count = await session.scalar(
        select(func.count(func.distinct(ProjectBOMItem.project_id))).where(
            ProjectBOMItem.part_id == part_id
        )
    )
    return int(count or 0)


def _to_related_ref(p: Part) -> PartRelatedRef:
    return PartRelatedRef(
        id=p.id,
        slug=p.slug,
        name=p.name,
        sku=p.sku,
        status=p.status,
        category=p.category,
        family=p.family,
    )


async def _load_related_parts(session: AsyncSession, part_id: int) -> list[Part]:
    """Return parts connected via ``part_relations`` from either side.

    Rows are stored with ``part_id < related_id``, but the editor may have
    added the link from either part's perspective. We union both directions
    so the related-parts list looks symmetric.
    """
    left = await session.scalars(
        select(PartRelation.related_id).where(PartRelation.part_id == part_id)
    )
    right = await session.scalars(
        select(PartRelation.part_id).where(PartRelation.related_id == part_id)
    )
    ids = sorted({*left.all(), *right.all()})
    if not ids:
        return []
    parts = (
        await session.scalars(
            select(Part).where(Part.id.in_(ids)).order_by(Part.name.asc())
        )
    ).all()
    return list(parts)


async def _load_family_parts(
    session: AsyncSession, part_id: int, family: Optional[str]
) -> list[Part]:
    if not family:
        return []
    parts = (
        await session.scalars(
            select(Part)
            .where(Part.family == family, Part.id != part_id)
            .order_by(Part.name.asc())
            .limit(20)
        )
    ).all()
    return list(parts)


async def _resolve_symbol(
    session: AsyncSession, symbol_id: Optional[int]
) -> Optional[LibrarySymbol]:
    """Look up a library symbol by id, or None when unset / missing.

    A stale ``symbol_id`` (symbol deleted out from under the part) reads
    back as no link rather than erroring — the FK is ON DELETE SET NULL so
    this should be rare, but we degrade gracefully either way.
    """
    if not symbol_id:
        return None
    return await session.get(LibrarySymbol, symbol_id)


def _symbol_ref(sym: Optional[LibrarySymbol]) -> Optional[PartSymbolRef]:
    if sym is None:
        return None
    return PartSymbolRef(
        id=sym.id,
        name=sym.name,
        category=sym.category or "Custom",
        ref_des_prefix=sym.ref_des_prefix or "U",
        current_revision_id=sym.current_revision_id,
        symbol_data=sym.symbol_data,
    )


async def _validate_symbol_id(session: AsyncSession, symbol_id: int) -> None:
    """Reject a link to a non-existent library symbol with a clean 400."""
    if await session.get(LibrarySymbol, symbol_id) is None:
        raise HTTPException(
            status.HTTP_400_BAD_REQUEST,
            detail=f"Library symbol {symbol_id} not found",
        )


def _photo_cover_url(photo: PartPhoto, slug: str) -> str:
    """Render-ready URL for a photo used as a part cover: external URL as-is,
    uploaded → the /view path (frontend prefixes the API base)."""
    if photo.external_url:
        return photo.external_url
    return f"/api/parts/{slug}/photos/{photo.id}/view"


async def first_photo_cover_urls(
    session: AsyncSession, parts_by_id: dict[int, str]
) -> dict[int, str]:
    """Batched first-photo cover URL per part id (keyed by id; value is the
    render-ready URL). ``parts_by_id`` maps part_id -> slug. One query for the
    whole set — used by the catalog search + BOM list to avoid N+1."""
    if not parts_by_id:
        return {}
    rows = (
        await session.scalars(
            select(PartPhoto)
            .where(PartPhoto.part_id.in_(parts_by_id.keys()))
            .order_by(PartPhoto.part_id.asc(), PartPhoto.sort_order.asc(), PartPhoto.id.asc())
        )
    ).all()
    out: dict[int, str] = {}
    for p in rows:
        if p.part_id in out:
            continue  # first (lowest sort_order) wins
        slug = parts_by_id.get(p.part_id)
        if slug:
            out[p.part_id] = _photo_cover_url(p, slug)
    return out


async def _load_photos(session: AsyncSession, part: Part) -> list[PartPhotoResponse]:
    """Part gallery photos, ordered. ``url`` is render-ready — the /view
    route for uploads, the external URL for links (matched in parts_photos)."""
    photos = (
        await session.scalars(
            select(PartPhoto)
            .where(PartPhoto.part_id == part.id)
            .order_by(PartPhoto.sort_order.asc(), PartPhoto.id.asc())
        )
    ).all()
    out: list[PartPhotoResponse] = []
    for p in photos:
        if p.external_url:
            url, is_external = p.external_url, True
        else:
            url = f"/api/parts/{part.slug}/photos/{p.id}/view"
            is_external = False
        out.append(PartPhotoResponse(
            id=p.id, title=p.title, url=url, is_external=is_external,
            sort_order=p.sort_order, created_by=p.created_by, created_at=p.created_at,
        ))
    return out


async def _part_detail(session: AsyncSession, part: Part) -> PartDetail:
    suppliers = await _load_suppliers(session, part.id)
    revs = (
        await session.scalars(
            select(PartRevision)
            .where(PartRevision.part_id == part.id)
            .order_by(PartRevision.created_at.desc(), PartRevision.id.desc())
            .limit(10)
        )
    ).all()
    related = await _load_related_parts(session, part.id)
    family_parts = await _load_family_parts(session, part.id, part.family)
    return PartDetail(
        id=part.id,
        slug=part.slug,
        name=part.name,
        sku=part.sku,
        mpn=part.mpn,
        description_md=part.description_md,
        tags=list(part.tags or []),
        status=part.status,
        created_by=part.created_by,
        created_at=part.created_at,
        updated_at=part.updated_at,
        current_revision_id=part.current_revision_id,
        usage_count=part.usage_count,
        category=part.category,
        family=part.family,
        suppliers=[_supplier_response(s) for s in suppliers],
        recent_revisions=[
            PartRevisionSummary(
                id=r.id,
                author=r.author,
                created_at=r.created_at,
                change_summary=r.change_summary,
            )
            for r in revs
        ],
        related_parts=[_to_related_ref(p) for p in related],
        family_parts=[_to_related_ref(p) for p in family_parts],
        verified_at=part.verified_at,
        verified_signals=int(part.verified_signals or 0),
        symbol_id=part.symbol_id,
        symbol=_symbol_ref(await _resolve_symbol(session, part.symbol_id)),
        photos=await _load_photos(session, part),
    )


async def _primary_supplier_url(session: AsyncSession, part_id: int) -> Optional[str]:
    row = await session.scalar(
        select(PartSupplier.url)
        .where(PartSupplier.part_id == part_id)
        .order_by(PartSupplier.id)
        .limit(1)
    )
    return row


# --- endpoints ----------------------------------------------------------


@router.get("", response_model=list[PartSearchResult])
async def search_parts(
    q: Optional[str] = Query(None, max_length=200),
    limit: int = Query(10, ge=1, le=50),
    category: Optional[str] = Query(None, max_length=60),
    family: Optional[str] = Query(None, max_length=80),
    session: AsyncSession = Depends(get_session),
) -> list[PartSearchResult]:
    """Substring search across name / sku / mpn / aliases.

    Empty or whitespace-only ``q`` returns the most-used parts so the
    BOM-editor autosuggest can show something useful on focus.

    Optional ``category`` / ``family`` filters narrow the result set —
    useful for the catalog browse page and the related-parts picker on
    the edit screen (issue #135).
    """
    stmt = select(Part)
    if q and q.strip():
        needle = f"%{q.strip().lower()}%"
        alias_part_ids_subq = (
            select(PartAlias.part_id)
            .where(func.lower(PartAlias.alias).like(needle))
        )
        stmt = stmt.where(
            or_(
                func.lower(Part.name).like(needle),
                func.lower(func.coalesce(Part.sku, "")).like(needle),
                func.lower(func.coalesce(Part.mpn, "")).like(needle),
                Part.id.in_(alias_part_ids_subq),
            )
        )
    if category:
        # Canonicalise the filter so ?category=sensor / single-board-computer
        # matches the stored canonical label ("Sensor", "Single Board
        # Computer") regardless of the caller's casing / slug style.
        canon = canonicalize(category, await _distinct_categories_by_usage(session))
        if canon:
            stmt = stmt.where(Part.category == canon)
    if family:
        stmt = stmt.where(Part.family == family)
    stmt = stmt.order_by(Part.usage_count.desc(), Part.name.asc()).limit(limit)
    parts = (await session.scalars(stmt)).all()

    covers = await first_photo_cover_urls(session, {p.id: p.slug for p in parts})
    results: list[PartSearchResult] = []
    for p in parts:
        results.append(
            PartSearchResult(
                id=p.id,
                slug=p.slug,
                name=p.name,
                sku=p.sku,
                status=p.status,
                usage_count=p.usage_count,
                primary_supplier_url=await _primary_supplier_url(session, p.id),
                category=p.category,
                family=p.family,
                cover_url=covers.get(p.id) or p.image_url,
            )
        )
    return results


# --- categories + families metadata (issue #135) ----------------------


async def _distinct_categories_by_usage(session: AsyncSession) -> list[str]:
    """Distinct ``parts.category`` values, most-used first."""
    rows = (
        await session.execute(
            select(Part.category, func.count(Part.id))
            .where(Part.category.is_not(None), Part.category != "")
            .group_by(Part.category)
            .order_by(func.count(Part.id).desc(), Part.category.asc())
        )
    ).all()
    return [row[0] for row in rows if row[0]]


@router.get("/_meta/categories", response_model=list[str])
async def list_categories(
    session: AsyncSession = Depends(get_session),
) -> list[str]:
    """Autocomplete vocabulary for the category field.

    Curated starter labels first (canonical order), then any categories
    users have actually coined, ranked by usage — so the list both seeds
    an empty catalog and converges on real usage as it grows. Deduped
    case/format-insensitively so slug + label variants don't double up.
    """
    return merge_for_suggestions(await _distinct_categories_by_usage(session))


@router.get("/_meta/families", response_model=list[str])
async def list_families(
    q: Optional[str] = Query(None, max_length=80),
    limit: int = Query(20, ge=1, le=100),
    session: AsyncSession = Depends(get_session),
) -> list[str]:
    """Distinct family values for autocomplete on the edit screen.

    Empty ``q`` returns the most-common families. Substring matching is
    case-insensitive.
    """
    stmt = select(Part.family, func.count(Part.id).label("n")).where(
        Part.family.is_not(None), Part.family != ""
    )
    if q and q.strip():
        needle = f"%{q.strip().lower()}%"
        stmt = stmt.where(func.lower(Part.family).like(needle))
    stmt = stmt.group_by(Part.family).order_by(func.count(Part.id).desc(), Part.family.asc()).limit(limit)
    rows = (await session.execute(stmt)).all()
    return [row[0] for row in rows if row[0]]


@router.post("", response_model=PartDetail, status_code=201)
async def create_part(
    body: PartCreate,
    user: str = Depends(require_terms_accepted_aged),
    session: AsyncSession = Depends(get_session),
) -> PartDetail:
    slug = await _unique_slug(session, _slugify(body.name))
    now = _now_naive_utc()
    tags = [t.strip() for t in body.tags if t and t.strip()]
    category = canonicalize(body.category, await _distinct_categories_by_usage(session))
    family = (body.family or "").strip() or None
    if body.symbol_id is not None:
        await _validate_symbol_id(session, body.symbol_id)
    part = Part(
        slug=slug,
        name=body.name,
        sku=body.sku or None,
        mpn=body.mpn or None,
        description_md=body.description_md,
        tags=tags,
        status="draft",
        category=category,
        family=family,
        symbol_id=body.symbol_id,
        created_by=user,
        created_at=now,
        updated_at=now,
        usage_count=0,
    )
    session.add(part)
    await session.flush()

    suppliers: list[PartSupplier] = []
    if body.supplier_url:
        sup = PartSupplier(
            part_id=part.id,
            supplier_name=body.supplier_name,
            url=body.supplier_url,
        )
        session.add(sup)
        await session.flush()
        suppliers.append(sup)

    rev = PartRevision(
        part_id=part.id,
        author=user,
        change_summary="Initial draft",
        name=part.name,
        sku=part.sku,
        mpn=part.mpn,
        description_md=part.description_md,
        image_url=part.image_url,
        tags=list(tags),
        category=category,
        family=family,
        symbol_id=part.symbol_id,
        suppliers_json=_suppliers_to_json(suppliers),
        created_at=now,
    )
    session.add(rev)
    await session.flush()
    part.current_revision_id = rev.id

    await session.commit()
    await session.refresh(part)
    return await _part_detail(session, part)


# --- related parts + family helpers ------------------------------------


def _canonical_pair(a: int, b: int) -> tuple[int, int]:
    """Return (small, large) so we always store the same row regardless of
    which side the editor linked from."""
    return (a, b) if a < b else (b, a)


async def _get_part_by_slug(session: AsyncSession, slug: str) -> Part:
    part = await session.scalar(select(Part).where(Part.slug == slug))
    if part is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Part not found")
    return part


@router.get("/{slug}", response_model=PartDetail)
async def get_part(
    slug: str,
    session: AsyncSession = Depends(get_session),
) -> PartDetail:
    part = await _get_part_by_slug(session, slug)
    return await _part_detail(session, part)


def _normalize_tags(tags: Optional[list[str]]) -> list[str]:
    if not tags:
        return []
    return [t.strip() for t in tags if t and t.strip()]


def _supplier_signature(suppliers: list[dict]) -> list[tuple]:
    """Hashable view of suppliers for change detection (order matters).

    Issue #149: include ``country_code`` so editing just the country (and
    nothing else) is recognised as a real change and writes a revision.

    Supplier-pricing feature: also include ``unit_cost`` +
    ``currency_code`` so a pure price change (no other field touched)
    still triggers a new revision and ends up in ``suppliers_json`` for
    the audit trail.
    """
    return [
        (
            s.get("name"),
            s.get("url"),
            s.get("country_code"),
            s.get("unit_cost"),
            s.get("currency_code"),
        )
        for s in suppliers
    ]


@router.put("/{slug}", response_model=PartDetail)
async def update_part(
    slug: str,
    body: PartUpdate,
    user: str = Depends(require_terms_accepted_aged),
    session: AsyncSession = Depends(get_session),
) -> PartDetail:
    part = await _get_part_by_slug(session, slug)

    # Resolve the *next* state for each editable field. None on the body
    # means "no change requested for this field".
    next_name = body.name if body.name is not None else part.name
    next_sku = body.sku if body.sku is not None else part.sku
    next_mpn = body.mpn if body.mpn is not None else part.mpn
    next_desc = body.description_md if body.description_md is not None else part.description_md
    # image_url is retired (images now live in part_photos). Carry the dormant
    # value through unchanged so revision snapshots / the guard stay stable.
    next_image = part.image_url
    next_tags = _normalize_tags(body.tags) if body.tags is not None else list(part.tags or [])
    # Empty string is interpreted as "clear" for category / family — the
    # editor sends "" when the user resets the dropdown / blanks the input.
    # Category is canonicalised (converging slug/casing variants onto one
    # label); ``None`` means "leave unchanged".
    if body.category is None:
        next_category = part.category
    else:
        next_category = canonicalize(
            body.category, await _distinct_categories_by_usage(session)
        )
    if body.family is None:
        next_family = part.family
    else:
        stripped = body.family.strip()
        next_family = stripped or None

    # Part↔symbol link: omitted → unchanged; present (incl. null) → set.
    # ``model_fields_set`` is what lets ``null`` mean "clear" rather than
    # being indistinguishable from the field's default.
    if "symbol_id" in body.model_fields_set:
        next_symbol_id = body.symbol_id
        if next_symbol_id is not None and next_symbol_id != part.symbol_id:
            await _validate_symbol_id(session, next_symbol_id)
    else:
        next_symbol_id = part.symbol_id

    current_suppliers = await _load_suppliers(session, part.id)
    current_supplier_json = _suppliers_to_json(current_suppliers)
    if body.suppliers is not None:
        next_supplier_json = [
            {
                "name": (s.name or None),
                "url": s.url,
                "country_code": (s.country_code or None),
                "unit_cost": s.unit_cost,
                "currency_code": (s.currency_code or None),
            }
            for s in body.suppliers
        ]
    else:
        next_supplier_json = current_supplier_json

    # Diff vs current — reject if nothing actually changed.
    changed = (
        next_name != part.name
        or next_sku != part.sku
        or next_mpn != part.mpn
        or next_desc != part.description_md
        or list(next_tags) != list(part.tags or [])
        or next_category != part.category
        or next_family != part.family
        or next_symbol_id != part.symbol_id
        or _supplier_signature(next_supplier_json) != _supplier_signature(current_supplier_json)
    )
    if not changed:
        raise HTTPException(
            status.HTTP_400_BAD_REQUEST,
            detail="No fields changed",
        )

    # Issue #136: mass-deletion guard. Done BEFORE we mutate any session
    # state for the in-progress edit, so a triggered result can safely
    # commit only the rollback + user-disable side effects without the
    # destructive write tagging along. If the guard raises, the
    # surrounding request handler returns 500 and the session is rolled
    # back by FastAPI's get_session dependency — meaning the destructive
    # write does NOT happen, which is the correctness guarantee we want.
    prev_snapshot = snapshot_from_part(part, current_suppliers)
    next_snapshot = snapshot_from_inputs(
        name=next_name,
        sku=next_sku,
        mpn=next_mpn,
        description_md=next_desc,
        image_url=next_image,
        tags=list(next_tags),
        suppliers_json=next_supplier_json,
    )
    result = await check_and_handle_mass_deletion(
        session, user, prev_snapshot, next_snapshot
    )
    if result.triggered:
        # Commit the rollback + user-disable that the guard already
        # added to the session. Do NOT apply the in-progress edit.
        await session.commit()
        raise HTTPException(
            status.HTTP_403_FORBIDDEN,
            detail=(
                f"Mass deletion detected — your account has been disabled "
                f"and {result.restored_count} part(s) you recently edited "
                f"have been restored. Contact an admin to appeal."
            ),
        )

    now = _now_naive_utc()
    # Write the revision first so we have an id to point `current_revision_id` at.
    rev = PartRevision(
        part_id=part.id,
        author=user,
        change_summary=body.change_summary,
        name=next_name,
        sku=next_sku,
        mpn=next_mpn,
        description_md=next_desc,
        image_url=next_image,
        tags=list(next_tags),
        category=next_category,
        family=next_family,
        symbol_id=next_symbol_id,
        suppliers_json=next_supplier_json,
        created_at=now,
    )
    session.add(rev)
    await session.flush()

    # Apply to the part itself.
    part.name = next_name
    part.sku = next_sku
    part.mpn = next_mpn
    part.description_md = next_desc
    part.image_url = next_image
    part.tags = list(next_tags)
    part.category = next_category
    part.family = next_family
    part.symbol_id = next_symbol_id
    part.current_revision_id = rev.id
    part.updated_at = now

    # Replace suppliers en-bloc (simplest correct behaviour for v1).
    if body.suppliers is not None:
        await session.execute(
            delete(PartSupplier).where(PartSupplier.part_id == part.id)
        )
        for entry in next_supplier_json:
            session.add(
                PartSupplier(
                    part_id=part.id,
                    supplier_name=entry.get("name"),
                    url=entry.get("url"),
                    country_code=entry.get("country_code"),
                    unit_cost=entry.get("unit_cost"),
                    currency_code=entry.get("currency_code"),
                )
            )

    await session.commit()
    await session.refresh(part)
    # Issue #122 Phase 2: a new revision is a lifecycle signal — try
    # promoting if all the rules now pass. Safe no-op when they don't.
    await compute_part_status(session, part, commit=True)
    await session.refresh(part)
    return await _part_detail(session, part)


@router.get("/{slug}/revisions", response_model=list[PartRevisionSummary])
async def list_revisions(
    slug: str,
    limit: int = Query(20, ge=1, le=100),
    offset: int = Query(0, ge=0),
    session: AsyncSession = Depends(get_session),
) -> list[PartRevisionSummary]:
    part = await _get_part_by_slug(session, slug)
    revs = (
        await session.scalars(
            select(PartRevision)
            .where(PartRevision.part_id == part.id)
            .order_by(PartRevision.created_at.desc(), PartRevision.id.desc())
            .limit(limit)
            .offset(offset)
        )
    ).all()
    return [
        PartRevisionSummary(
            id=r.id,
            author=r.author,
            created_at=r.created_at,
            change_summary=r.change_summary,
        )
        for r in revs
    ]


@router.get("/{slug}/revisions/{rev_id}", response_model=PartRevisionDetail)
async def get_revision(
    slug: str,
    rev_id: int,
    session: AsyncSession = Depends(get_session),
) -> PartRevisionDetail:
    part = await _get_part_by_slug(session, slug)
    rev = await session.get(PartRevision, rev_id)
    if rev is None or rev.part_id != part.id:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Revision not found")
    suppliers_raw = rev.suppliers_json or []
    suppliers = [
        PartSupplierInput(
            name=s.get("name"),
            url=s.get("url", ""),
            country_code=s.get("country_code"),
            unit_cost=s.get("unit_cost"),
            currency_code=s.get("currency_code"),
        )
        for s in suppliers_raw
        if s.get("url")
    ]
    return PartRevisionDetail(
        id=rev.id,
        author=rev.author,
        created_at=rev.created_at,
        change_summary=rev.change_summary,
        name=rev.name,
        sku=rev.sku,
        mpn=rev.mpn,
        description_md=rev.description_md,
        tags=list(rev.tags or []),
        suppliers=suppliers,
        category=rev.category,
        family=rev.family,
        symbol_id=rev.symbol_id,
    )


@router.post("/{slug}/revisions/{rev_id}/restore", response_model=PartDetail)
async def restore_revision(
    slug: str,
    rev_id: int,
    user: str = Depends(require_terms_accepted_aged),
    session: AsyncSession = Depends(get_session),
) -> PartDetail:
    """Non-destructive restore: write a NEW revision whose snapshot
    equals ``rev_id``. History stays linear."""
    part = await _get_part_by_slug(session, slug)
    source = await session.get(PartRevision, rev_id)
    if source is None or source.part_id != part.id:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Revision not found")

    # Issue #136: a "restore" to an empty / much-shorter old revision is
    # equivalent to a mass-delete. Run the same guard as PUT.
    current_suppliers = await _load_suppliers(session, part.id)
    prev_snapshot = snapshot_from_part(part, current_suppliers)
    next_snapshot = snapshot_from_inputs(
        name=source.name,
        sku=source.sku,
        mpn=source.mpn,
        description_md=source.description_md,
        image_url=source.image_url,
        tags=list(source.tags or []),
        suppliers_json=list(source.suppliers_json or []),
    )
    result = await check_and_handle_mass_deletion(
        session, user, prev_snapshot, next_snapshot
    )
    if result.triggered:
        await session.commit()
        raise HTTPException(
            status.HTTP_403_FORBIDDEN,
            detail=(
                f"Mass deletion detected — your account has been disabled "
                f"and {result.restored_count} part(s) you recently edited "
                f"have been restored. Contact an admin to appeal."
            ),
        )

    now = _now_naive_utc()
    suppliers_json = list(source.suppliers_json or [])
    new_rev = PartRevision(
        part_id=part.id,
        author=user,
        change_summary=f"Restored from revision #{source.id}",
        name=source.name,
        sku=source.sku,
        mpn=source.mpn,
        description_md=source.description_md,
        image_url=source.image_url,
        tags=list(source.tags or []),
        category=source.category,
        family=source.family,
        symbol_id=source.symbol_id,
        suppliers_json=suppliers_json,
        created_at=now,
    )
    session.add(new_rev)
    await session.flush()

    part.name = source.name
    part.sku = source.sku
    part.mpn = source.mpn
    part.description_md = source.description_md
    part.image_url = source.image_url
    part.tags = list(source.tags or [])
    part.category = source.category
    part.family = source.family
    part.symbol_id = source.symbol_id
    part.current_revision_id = new_rev.id
    part.updated_at = now

    await session.execute(
        delete(PartSupplier).where(PartSupplier.part_id == part.id)
    )
    for entry in suppliers_json:
        if not entry.get("url"):
            continue
        session.add(
            PartSupplier(
                part_id=part.id,
                supplier_name=entry.get("name"),
                url=entry.get("url"),
                country_code=entry.get("country_code"),
                unit_cost=entry.get("unit_cost"),
                currency_code=entry.get("currency_code"),
            )
        )

    await session.commit()
    await session.refresh(part)
    # Issue #122 Phase 2: restore is also a new-revision signal.
    await compute_part_status(session, part, commit=True)
    await session.refresh(part)
    return await _part_detail(session, part)


# --- related parts (issue #135) ----------------------------------------


@router.get("/{slug}/related", response_model=list[PartRelatedRef])
async def list_related_parts(
    slug: str,
    session: AsyncSession = Depends(get_session),
) -> list[PartRelatedRef]:
    """List all parts linked to ``slug`` via ``part_relations``."""
    part = await _get_part_by_slug(session, slug)
    related = await _load_related_parts(session, part.id)
    return [_to_related_ref(p) for p in related]


@router.post("/{slug}/related", response_model=list[PartRelatedRef], status_code=201)
async def add_related_part(
    slug: str,
    body: PartRelationCreate,
    user: str = Depends(require_terms_accepted_aged),
    session: AsyncSession = Depends(get_session),
) -> list[PartRelatedRef]:
    """Link ``slug`` to another part identified by ``related_slug``.

    The 14-day account-age gate applies (via ``get_current_user_aged``).
    Idempotent: re-adding an existing link is a no-op rather than a 409
    so the UI can stay simple.
    """
    part = await _get_part_by_slug(session, slug)
    target_slug = (body.related_slug or "").strip()
    if not target_slug or target_slug == slug:
        raise HTTPException(
            status.HTTP_400_BAD_REQUEST,
            detail="related_slug must refer to a different part",
        )
    target = await session.scalar(select(Part).where(Part.slug == target_slug))
    if target is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Related part not found")
    if target.id == part.id:
        raise HTTPException(
            status.HTTP_400_BAD_REQUEST,
            detail="A part cannot be related to itself",
        )

    low, high = _canonical_pair(part.id, target.id)
    existing = await session.scalar(
        select(PartRelation.id).where(
            PartRelation.part_id == low,
            PartRelation.related_id == high,
        )
    )
    if existing is None:
        session.add(
            PartRelation(
                part_id=low,
                related_id=high,
                created_by=user,
                created_at=_now_naive_utc(),
            )
        )
        await session.commit()

    related = await _load_related_parts(session, part.id)
    return [_to_related_ref(p) for p in related]


@router.delete("/{slug}/related/{related_slug}", status_code=204)
async def remove_related_part(
    slug: str,
    related_slug: str,
    user: str = Depends(require_terms_accepted_aged),
    session: AsyncSession = Depends(get_session),
) -> None:
    part = await _get_part_by_slug(session, slug)
    target = await session.scalar(select(Part).where(Part.slug == related_slug))
    if target is None:
        # Treat "delete a relation to a non-existent part" as a no-op so the
        # UI doesn't need to keep stale slugs in sync.
        return None
    low, high = _canonical_pair(part.id, target.id)
    await session.execute(
        delete(PartRelation).where(
            PartRelation.part_id == low,
            PartRelation.related_id == high,
        )
    )
    await session.commit()
    return None


@router.get("/{slug}/family", response_model=list[PartRelatedRef])
async def list_family_parts(
    slug: str,
    session: AsyncSession = Depends(get_session),
) -> list[PartRelatedRef]:
    """Other parts that share this part's ``family`` value.

    Returns ``[]`` when the part has no family set or is the only one in
    its family.
    """
    part = await _get_part_by_slug(session, slug)
    family_parts = await _load_family_parts(session, part.id, part.family)
    return [_to_related_ref(p) for p in family_parts]
