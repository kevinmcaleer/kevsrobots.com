"""Admin feedback inbox endpoints (issue #138).

Backs the admin inbox at ``web/admin/feedback.html``. All routes here
require an admin user (see ``auth.require_admin``); non-admins get
403, unauthenticated callers get 401.
"""

from __future__ import annotations

import logging
from datetime import datetime, timezone
from typing import Optional

from fastapi import APIRouter, Depends, HTTPException, Query, status
from fastapi.responses import Response
from sqlalchemy import func, or_, select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import require_feedback_admin
from ..db import get_session
from ..models import Feedback
from ..schemas import (
    FEEDBACK_SENTIMENTS,
    FEEDBACK_STATUSES,
    FeedbackCountsResponse,
    FeedbackListResponse,
    FeedbackResponse,
    FeedbackUpdateStatus,
)
from ..storage import delete_file, read_file

logger = logging.getLogger(__name__)

router = APIRouter(tags=["admin-feedback"])


def _row_to_response(row: Feedback) -> FeedbackResponse:
    """Convert an ORM row to the API response shape.

    ``screenshot_url`` is computed from the stable view-route URL if a
    screenshot is stored — admins always hit the API for the bytes, so
    we don't need to expose a separate signed URL.
    """
    screenshot_url: Optional[str] = row.screenshot_url
    if screenshot_url is None and row.screenshot_path:
        screenshot_url = f"/api/admin/feedback/{row.id}/screenshot"
    return FeedbackResponse(
        id=row.id,
        sentiment=row.sentiment,
        message=row.message,
        email=row.email,
        username=row.username,
        user_id=row.user_id,
        status=row.status,
        page_url=row.page_url,
        referrer=row.referrer,
        user_agent=row.user_agent,
        viewport=row.viewport,
        screenshot_url=screenshot_url,
        read_at=row.read_at,
        read_by_user_id=row.read_by_user_id,
        admin_notes=row.admin_notes,
        created_at=row.created_at,
        updated_at=row.updated_at,
    )


def _parse_sentiments(values: list[str]) -> list[str]:
    """Accept repeated ``?sentiment=love&sentiment=like`` AND CSV
    (``?sentiment=love,like``) and validate against the known set.

    Returns an empty list when nothing was passed; the caller treats
    that as "no sentiment filter applied". Unknown values 400.
    """
    out: list[str] = []
    for raw in values:
        for part in raw.split(","):
            tok = part.strip()
            if not tok:
                continue
            if tok not in FEEDBACK_SENTIMENTS:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail=f"Unknown sentiment: {tok}",
                )
            out.append(tok)
    return out


@router.get(
    "/api/admin/feedback",
    response_model=FeedbackListResponse,
)
async def list_feedback(
    status_filter: Optional[str] = Query(default=None, alias="status"),
    sentiment: list[str] = Query(default_factory=list),
    q: Optional[str] = Query(default=None),
    user_id: Optional[str] = Query(default=None),
    limit: int = Query(default=50, ge=1, le=200),
    offset: int = Query(default=0, ge=0),
    admin: str = Depends(require_feedback_admin),
    session: AsyncSession = Depends(get_session),
) -> FeedbackListResponse:
    """List feedback rows, newest-first.

    Sentiment may be repeated or comma-separated. The ``q`` filter is a
    case-insensitive substring on either ``message`` or ``page_url``. The
    ``user_id`` filter narrows to one reporter — e.g. ``_snakie_anon`` for
    reports raised from the Snakie desktop app (its Bug Tracker uses this).
    """
    base_filters = []
    if user_id:
        base_filters.append(Feedback.user_id == user_id)
    if status_filter:
        if status_filter not in FEEDBACK_STATUSES:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Unknown status: {status_filter}",
            )
        base_filters.append(Feedback.status == status_filter)

    sentiments = _parse_sentiments(sentiment)
    if sentiments:
        base_filters.append(Feedback.sentiment.in_(sentiments))

    if q:
        pattern = f"%{q.lower()}%"
        base_filters.append(
            or_(
                func.lower(Feedback.message).like(pattern),
                func.lower(Feedback.page_url).like(pattern),
            )
        )

    # Count first — same WHERE clauses, no ORDER BY / LIMIT.
    count_q = select(func.count(Feedback.id))
    for f in base_filters:
        count_q = count_q.where(f)
    total = int((await session.execute(count_q)).scalar_one() or 0)

    list_q = select(Feedback)
    for f in base_filters:
        list_q = list_q.where(f)
    # Secondary sort on id DESC breaks ties when two rows share a
    # created_at down to the second (common in tests that POST rapidly).
    list_q = (
        list_q.order_by(Feedback.created_at.desc(), Feedback.id.desc())
        .limit(limit)
        .offset(offset)
    )

    rows = (await session.scalars(list_q)).all()
    return FeedbackListResponse(
        items=[_row_to_response(r) for r in rows],
        total=total,
    )


@router.get(
    "/api/admin/feedback/counts",
    response_model=FeedbackCountsResponse,
)
async def feedback_counts(
    admin: str = Depends(require_feedback_admin),
    session: AsyncSession = Depends(get_session),
) -> FeedbackCountsResponse:
    """Aggregate counts for the inbox header strip."""
    by_status_rows = (
        await session.execute(
            select(Feedback.status, func.count(Feedback.id)).group_by(Feedback.status)
        )
    ).all()
    by_status: dict[str, int] = {s: 0 for s in FEEDBACK_STATUSES}
    for s, c in by_status_rows:
        if s in by_status:
            by_status[s] = int(c)

    by_sentiment_rows = (
        await session.execute(
            select(Feedback.sentiment, func.count(Feedback.id)).group_by(
                Feedback.sentiment
            )
        )
    ).all()
    by_sentiment: dict[str, int] = {s: 0 for s in FEEDBACK_SENTIMENTS}
    for s, c in by_sentiment_rows:
        if s in by_sentiment:
            by_sentiment[s] = int(c)

    total = sum(by_status.values())
    return FeedbackCountsResponse(
        total=total,
        unread=by_status["unread"],
        read=by_status["read"],
        archived=by_status["archived"],
        by_sentiment=by_sentiment,
    )


@router.get(
    "/api/admin/feedback/{feedback_id}",
    response_model=FeedbackResponse,
)
async def get_feedback(
    feedback_id: int,
    admin: str = Depends(require_feedback_admin),
    session: AsyncSession = Depends(get_session),
) -> FeedbackResponse:
    row = await session.get(Feedback, feedback_id)
    if row is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Feedback not found",
        )
    return _row_to_response(row)


@router.patch(
    "/api/admin/feedback/{feedback_id}",
    response_model=FeedbackResponse,
)
async def update_feedback_status(
    feedback_id: int,
    body: FeedbackUpdateStatus,
    admin: str = Depends(require_feedback_admin),
    session: AsyncSession = Depends(get_session),
) -> FeedbackResponse:
    """Update a feedback row's status and/or the maintainer's private notes.

    Only fields PRESENT in the request body are applied (via
    ``model_fields_set``), so a Bug-Tracker "Close" sends ``{"status":
    "archived"}`` and a notes save sends ``{"admin_notes": "..."}`` without
    disturbing the other. Sending neither is a harmless no-op.

    When transitioning *into* ``read``, we stamp ``read_at`` and
    ``read_by_user_id`` so the admin UI can show "marked read 5m ago by
    @kev". A re-PATCH to ``read`` after it's already read is a no-op on
    those fields (we keep the original reader).
    """
    row = await session.get(Feedback, feedback_id)
    if row is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Feedback not found",
        )

    sent = body.model_fields_set
    changed = False

    if "status" in sent and body.status is not None:
        previous = row.status
        row.status = body.status
        if body.status == "read" and previous != "read":
            # read_at / read_by are stamped now; we intentionally do NOT clear
            # them on a later transition back to unread/archived — that history
            # is useful in the audit trail.
            row.read_at = datetime.now(timezone.utc).replace(tzinfo=None)
            row.read_by_user_id = admin
        changed = True

    if "admin_notes" in sent:
        # May be None to clear the notes.
        row.admin_notes = body.admin_notes
        changed = True

    if changed:
        row.updated_at = datetime.now(timezone.utc).replace(tzinfo=None)
        await session.commit()
        await session.refresh(row)
    return _row_to_response(row)


@router.delete(
    "/api/admin/feedback/{feedback_id}",
    status_code=204,
)
async def delete_feedback(
    feedback_id: int,
    admin: str = Depends(require_feedback_admin),
    session: AsyncSession = Depends(get_session),
) -> Response:
    """Hard delete a row and any associated screenshot file."""
    row = await session.get(Feedback, feedback_id)
    if row is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Feedback not found",
        )

    path = row.screenshot_path
    await session.delete(row)
    await session.commit()

    if path:
        try:
            delete_file(path)
        except Exception:  # pragma: no cover — defensive
            logger.warning(
                "Failed to delete feedback screenshot at %s", path, exc_info=True
            )
    return Response(status_code=204)


@router.get("/api/admin/feedback/{feedback_id}/screenshot")
async def view_feedback_screenshot(
    feedback_id: int,
    admin: str = Depends(require_feedback_admin),
    session: AsyncSession = Depends(get_session),
) -> Response:
    """Serve back the screenshot bytes for an admin viewer.

    Gated behind the admin dep so that screenshots — which may contain
    PII the user pasted — are not publicly addressable.
    """
    row = await session.get(Feedback, feedback_id)
    if row is None or not row.screenshot_path:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Screenshot not found",
        )

    data = read_file(row.screenshot_path)
    if data is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Screenshot file missing in storage",
        )

    # Best-effort MIME from the stored extension.
    ext = row.screenshot_path.rsplit(".", 1)[-1].lower() if "." in row.screenshot_path else "png"
    media_types = {
        "png": "image/png",
        "jpg": "image/jpeg",
        "jpeg": "image/jpeg",
        "gif": "image/gif",
        "webp": "image/webp",
    }
    return Response(
        content=data,
        media_type=media_types.get(ext, "application/octet-stream"),
    )
