"""Editorial Staff Picks collections (issue #115).

A staff pick is a curated set of projects with a title, optional period,
and editor notes per item. The pick row stays as a draft until an admin
flips ``is_published`` true; only then does it surface on the public
``/api/staff-picks`` listing.

The admin endpoints rely on ``get_admin_user`` from the moderation
router — we deliberately do not roll our own admin dependency.
"""

from __future__ import annotations

from typing import Optional

from fastapi import APIRouter, Depends, HTTPException, Query, status
from sqlalchemy import func, select
from sqlalchemy.exc import IntegrityError
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import get_optional_user
from ..config import get_settings
from ..db import get_session
from ..models import Project, StaffPick, StaffPickItem
from ..schemas import (
    StaffPickCreate,
    StaffPickDetail,
    StaffPickItemCreate,
    StaffPickItemResponse,
    StaffPickItemUpdate,
    StaffPickProjectRef,
    StaffPickResponse,
    StaffPickUpdate,
)
from .moderation import get_admin_user

router = APIRouter(tags=["staff_picks"])


def _is_admin(username: Optional[str]) -> bool:
    if not username:
        return False
    return username in get_settings().admin_usernames_list


def _pick_summary(pick: StaffPick, item_count: int) -> StaffPickResponse:
    return StaffPickResponse(
        id=pick.id,
        title=pick.title,
        description=pick.description,
        period_start=pick.period_start,
        period_end=pick.period_end,
        created_by=pick.created_by,
        created_at=pick.created_at,
        is_published=bool(pick.is_published),
        cover_image_url=pick.cover_image_url,
        item_count=item_count,
    )


async def _count_items(session: AsyncSession, pick_id: int) -> int:
    n = await session.scalar(
        select(func.count(StaffPickItem.id)).where(StaffPickItem.staff_pick_id == pick_id)
    )
    return int(n or 0)


async def _load_items(session: AsyncSession, pick_id: int) -> list[StaffPickItemResponse]:
    """Return items in display order with the project reference embedded."""
    rows = (
        await session.scalars(
            select(StaffPickItem)
            .where(StaffPickItem.staff_pick_id == pick_id)
            .order_by(StaffPickItem.order_index.asc(), StaffPickItem.id.asc())
        )
    ).all()
    items: list[StaffPickItemResponse] = []
    for row in rows:
        project = await session.get(Project, row.project_id)
        project_ref: Optional[StaffPickProjectRef] = None
        if project is not None:
            project_ref = StaffPickProjectRef(
                id=project.id,
                slug=getattr(project, "slug", None),
                title=project.title,
                short_description=project.short_description,
                author_username=project.author_username,
                cover_image=project.cover_image,
                difficulty=project.difficulty,
                status=project.status,
                is_featured=bool(getattr(project, "is_featured", False)),
            )
        items.append(
            StaffPickItemResponse(
                id=row.id,
                project_id=row.project_id,
                editor_note=row.editor_note,
                order_index=row.order_index,
                project=project_ref,
            )
        )
    return items


# ---- Admin: pick CRUD ---------------------------------------------------


@router.post(
    "/api/admin/staff-picks",
    response_model=StaffPickResponse,
    status_code=201,
)
async def create_staff_pick(
    body: StaffPickCreate,
    admin: str = Depends(get_admin_user),
    session: AsyncSession = Depends(get_session),
) -> StaffPickResponse:
    pick = StaffPick(
        title=body.title.strip(),
        description=(body.description or "").strip() or None,
        period_start=body.period_start,
        period_end=body.period_end,
        cover_image_url=body.cover_image_url,
        is_published=bool(body.is_published),
        created_by=admin,
    )
    session.add(pick)
    await session.commit()
    await session.refresh(pick)
    return _pick_summary(pick, 0)


@router.put(
    "/api/admin/staff-picks/{pick_id}",
    response_model=StaffPickResponse,
)
async def update_staff_pick(
    pick_id: int,
    body: StaffPickUpdate,
    admin: str = Depends(get_admin_user),
    session: AsyncSession = Depends(get_session),
) -> StaffPickResponse:
    pick = await session.get(StaffPick, pick_id)
    if pick is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Staff pick not found")
    data = body.model_dump(exclude_unset=True)
    for field, value in data.items():
        if field == "title" and isinstance(value, str):
            value = value.strip()
        setattr(pick, field, value)
    await session.commit()
    await session.refresh(pick)
    return _pick_summary(pick, await _count_items(session, pick.id))


@router.delete(
    "/api/admin/staff-picks/{pick_id}",
    status_code=204,
)
async def delete_staff_pick(
    pick_id: int,
    admin: str = Depends(get_admin_user),
    session: AsyncSession = Depends(get_session),
) -> None:
    pick = await session.get(StaffPick, pick_id)
    if pick is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Staff pick not found")
    await session.delete(pick)
    await session.commit()


# ---- Admin: item CRUD ---------------------------------------------------


@router.post(
    "/api/admin/staff-picks/{pick_id}/items",
    response_model=StaffPickItemResponse,
    status_code=201,
)
async def add_pick_item(
    pick_id: int,
    body: StaffPickItemCreate,
    admin: str = Depends(get_admin_user),
    session: AsyncSession = Depends(get_session),
) -> StaffPickItemResponse:
    pick = await session.get(StaffPick, pick_id)
    if pick is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Staff pick not found")
    project = await session.get(Project, body.project_id)
    if project is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Project not found")

    # Default order_index = (max existing) + 1 so the new entry lands at
    # the end of the list. The admin UI can PATCH to reorder.
    if body.order_index is None:
        max_idx = await session.scalar(
            select(func.coalesce(func.max(StaffPickItem.order_index), -1)).where(
                StaffPickItem.staff_pick_id == pick_id
            )
        )
        # ``max_idx`` is -1 when no items yet, else the highest existing
        # index. Don't use `or -1` here — Python's truthiness treats 0 as
        # falsy, which would clobber a legitimate max of 0.
        base = -1 if max_idx is None else int(max_idx)
        next_idx = base + 1
    else:
        next_idx = int(body.order_index)

    item = StaffPickItem(
        staff_pick_id=pick_id,
        project_id=body.project_id,
        editor_note=(body.editor_note or "").strip() or None,
        order_index=next_idx,
    )
    session.add(item)
    try:
        await session.commit()
    except IntegrityError:
        await session.rollback()
        raise HTTPException(
            status.HTTP_409_CONFLICT,
            detail="Project is already in this staff pick",
        )
    await session.refresh(item)

    return StaffPickItemResponse(
        id=item.id,
        project_id=item.project_id,
        editor_note=item.editor_note,
        order_index=item.order_index,
        project=StaffPickProjectRef(
            id=project.id,
            title=project.title,
            short_description=project.short_description,
            author_username=project.author_username,
            cover_image=project.cover_image,
            difficulty=project.difficulty,
            status=project.status,
            is_featured=bool(getattr(project, "is_featured", False)),
        ),
    )


@router.patch(
    "/api/admin/staff-picks/{pick_id}/items/{item_id}",
    response_model=StaffPickItemResponse,
)
async def update_pick_item(
    pick_id: int,
    item_id: int,
    body: StaffPickItemUpdate,
    admin: str = Depends(get_admin_user),
    session: AsyncSession = Depends(get_session),
) -> StaffPickItemResponse:
    item = await session.get(StaffPickItem, item_id)
    if item is None or item.staff_pick_id != pick_id:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Item not found")
    data = body.model_dump(exclude_unset=True)
    if "editor_note" in data:
        note = data["editor_note"]
        item.editor_note = (note or "").strip() or None
    if "order_index" in data and data["order_index"] is not None:
        item.order_index = int(data["order_index"])
    await session.commit()
    await session.refresh(item)
    project = await session.get(Project, item.project_id)
    project_ref: Optional[StaffPickProjectRef] = None
    if project is not None:
        project_ref = StaffPickProjectRef(
            id=project.id,
            title=project.title,
            short_description=project.short_description,
            author_username=project.author_username,
            cover_image=project.cover_image,
            difficulty=project.difficulty,
            status=project.status,
            is_featured=bool(getattr(project, "is_featured", False)),
        )
    return StaffPickItemResponse(
        id=item.id,
        project_id=item.project_id,
        editor_note=item.editor_note,
        order_index=item.order_index,
        project=project_ref,
    )


@router.delete(
    "/api/admin/staff-picks/{pick_id}/items/{item_id}",
    status_code=204,
)
async def remove_pick_item(
    pick_id: int,
    item_id: int,
    admin: str = Depends(get_admin_user),
    session: AsyncSession = Depends(get_session),
) -> None:
    item = await session.get(StaffPickItem, item_id)
    if item is None or item.staff_pick_id != pick_id:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Item not found")
    await session.delete(item)
    await session.commit()


# ---- Public + admin list / detail --------------------------------------


@router.get(
    "/api/staff-picks",
    response_model=list[StaffPickResponse],
)
async def list_staff_picks(
    published: Optional[bool] = Query(
        True,
        description=(
            "Default true. Set false to include drafts — only admins are "
            "allowed to do this; anonymous callers asking for drafts get an "
            "empty list rather than a 403 so the public UI stays simple."
        ),
    ),
    limit: int = Query(20, ge=1, le=100),
    user: Optional[str] = Depends(get_optional_user),
    session: AsyncSession = Depends(get_session),
) -> list[StaffPickResponse]:
    query = select(StaffPick).order_by(StaffPick.created_at.desc()).limit(limit)
    if published is True:
        query = query.where(StaffPick.is_published == True)  # noqa: E712
    elif published is False:
        # Asking for the unpublished view requires admin auth. If the user
        # isn't an admin, silently return an empty list — the public UI
        # never sends ``published=false`` anyway.
        if not _is_admin(user):
            return []
        query = query.where(StaffPick.is_published == False)  # noqa: E712
    # ``published`` is explicitly None → return both, but only for admins.
    elif not _is_admin(user):
        query = query.where(StaffPick.is_published == True)  # noqa: E712

    picks = list((await session.scalars(query)).all())
    out: list[StaffPickResponse] = []
    for p in picks:
        out.append(_pick_summary(p, await _count_items(session, p.id)))
    return out


@router.get(
    "/api/staff-picks/{pick_id}",
    response_model=StaffPickDetail,
)
async def get_staff_pick(
    pick_id: int,
    user: Optional[str] = Depends(get_optional_user),
    session: AsyncSession = Depends(get_session),
) -> StaffPickDetail:
    pick = await session.get(StaffPick, pick_id)
    if pick is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Staff pick not found")
    # Unpublished picks are visible only to admins. Anonymous + non-admin
    # callers get a 404 rather than 403 so we don't leak the existence of
    # in-progress drafts.
    if not pick.is_published and not _is_admin(user):
        raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Staff pick not found")
    items = await _load_items(session, pick.id)
    summary = _pick_summary(pick, len(items))
    return StaffPickDetail(**summary.model_dump(), items=items)
