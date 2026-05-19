"""User-facing feedback endpoint (issue #138).

Backs the floating widget at ``web/_includes/feedback_widget.html``.
The widget POSTs either JSON (no screenshot) or
``multipart/form-data`` (with a single ``screenshot`` file). Both
variants land here and end up as the same ``Feedback`` row.

Identity (``user_id``, ``username``) is always taken from the auth
dependency — never from the body — so a logged-in user can't
impersonate someone else by spoofing fields.
"""

from __future__ import annotations

import logging
import re
from typing import Optional

from fastapi import APIRouter, Depends, HTTPException, Request, status
from pydantic import ValidationError
from sqlalchemy.ext.asyncio import AsyncSession
from starlette.datastructures import UploadFile
from starlette.formparsers import MultiPartException

from ..auth import get_current_user
from ..db import get_session
from ..models import Feedback
from ..schemas import FeedbackCreate, FeedbackCreateResponse
from ..storage import (
    FEEDBACK_ALLOWED_IMAGE_EXTENSIONS,
    FEEDBACK_ALLOWED_MIME_TYPES,
    FEEDBACK_MAX_SCREENSHOT_SIZE,
    generate_feedback_filename,
    save_feedback_screenshot,
)

logger = logging.getLogger(__name__)

router = APIRouter(tags=["feedback"])

# A pragmatic email regex — same shape the widget already enforces
# client-side. We re-check server-side because the JSON variant has no
# Content-Type enforcement and a non-widget client could omit it.
_EMAIL_RE = re.compile(r"^[^\s@]+@[^\s@]+\.[^\s@]+$")


def _coerce_optional_str(value: object) -> Optional[str]:
    """Treat empty strings as null so form-data ``""`` doesn't become a
    real empty value on the row."""
    if value is None:
        return None
    if isinstance(value, str):
        s = value.strip()
        return s or None
    return None


@router.post(
    "/api/feedback",
    response_model=FeedbackCreateResponse,
    status_code=201,
)
async def create_feedback(
    request: Request,
    user: str = Depends(get_current_user),
    session: AsyncSession = Depends(get_session),
) -> FeedbackCreateResponse:
    """Create a new feedback row.

    Accepts JSON (``application/json``) or multipart form-data. The
    multipart variant may include a single ``screenshot`` file part —
    must be one of PNG/JPEG/WEBP/GIF and at most 4 MB.
    """
    content_type = (request.headers.get("content-type") or "").lower()
    is_multipart = content_type.startswith("multipart/form-data")

    payload: dict[str, object] = {}
    screenshot_upload: Optional[UploadFile] = None

    if is_multipart:
        # Starlette's default max_part_size is 1 MB — we need the 4 MB cap
        # for screenshots per the spec. Anything bigger raises
        # MultiPartException; surface it as the spec's 413.
        try:
            form = await request.form(max_part_size=FEEDBACK_MAX_SCREENSHOT_SIZE)
        except MultiPartException as exc:
            raise HTTPException(
                status_code=status.HTTP_413_REQUEST_ENTITY_TOO_LARGE,
                detail=str(exc),
            )
        for key in (
            "sentiment",
            "message",
            "email",
            "page_url",
            "referrer",
            "user_agent",
            "viewport",
        ):
            if key in form:
                payload[key] = _coerce_optional_str(form.get(key)) if key in (
                    "email",
                    "referrer",
                    "user_agent",
                    "viewport",
                ) else form.get(key)
        candidate = form.get("screenshot")
        # NB: ``form.get`` returns starlette's ``UploadFile``, not fastapi's
        # subclass — importing from starlette.datastructures keeps the
        # isinstance check honest across both versions.
        if isinstance(candidate, UploadFile):
            screenshot_upload = candidate
    else:
        # JSON body — handle bad/missing JSON ourselves so we return a
        # 400 with a useful detail instead of FastAPI's default 422 page.
        try:
            body = await request.json()
        except Exception:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Body must be valid JSON or multipart/form-data",
            )
        if not isinstance(body, dict):
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Body must be a JSON object",
            )
        payload = dict(body)
        # Normalise empty-string optionals to None for consistency.
        for k in ("email", "referrer", "user_agent", "viewport"):
            if k in payload:
                payload[k] = _coerce_optional_str(payload[k])

    # Strip any client-supplied identity fields — the auth dep is the
    # source of truth and we never trust the body for these.
    for forbidden in ("user_id", "username", "id", "created_at", "status"):
        payload.pop(forbidden, None)

    try:
        data = FeedbackCreate.model_validate(payload)
    except ValidationError as exc:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=exc.errors(),
        )

    if data.email and not _EMAIL_RE.match(data.email):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Invalid email address",
        )

    screenshot_path: Optional[str] = None
    screenshot_url: Optional[str] = None
    if screenshot_upload is not None:
        screenshot_bytes = await screenshot_upload.read()
        if len(screenshot_bytes) > FEEDBACK_MAX_SCREENSHOT_SIZE:
            raise HTTPException(
                status_code=status.HTTP_413_REQUEST_ENTITY_TOO_LARGE,
                detail="Screenshot exceeds 4 MB limit",
            )
        # Content-Type the client claimed for the file part.
        mime = (screenshot_upload.content_type or "").lower()
        # Belt-and-braces: also check the filename extension. Browsers
        # always send a sensible MIME when uploading from a file picker,
        # but a curl client could omit it.
        original_name = screenshot_upload.filename or ""
        ext_ok = any(
            original_name.lower().endswith(ext)
            for ext in FEEDBACK_ALLOWED_IMAGE_EXTENSIONS
        )
        if mime and mime not in FEEDBACK_ALLOWED_MIME_TYPES and not ext_ok:
            raise HTTPException(
                status_code=status.HTTP_415_UNSUPPORTED_MEDIA_TYPE,
                detail="Screenshot must be PNG, JPEG, WEBP, or GIF",
            )
        if not mime and not ext_ok:
            raise HTTPException(
                status_code=status.HTTP_415_UNSUPPORTED_MEDIA_TYPE,
                detail="Screenshot must be PNG, JPEG, WEBP, or GIF",
            )

        safe_name = generate_feedback_filename(original_name)
        screenshot_path = save_feedback_screenshot(screenshot_bytes, safe_name)
        if screenshot_path is None:
            # Storage failure shouldn't drop the feedback — log and
            # continue without the attachment. This trades attachment
            # availability for not losing the user's message.
            logger.warning("Failed to save feedback screenshot %s", safe_name)
        else:
            # Public URL is served by the admin viewer route below; admin UI
            # uses the full URL so we keep the scheme/host relative.
            screenshot_url = None  # populated lazily by admin response builder

    record = Feedback(
        user_id=user,
        username=user,
        sentiment=data.sentiment,
        message=data.message,
        email=data.email,
        page_url=data.page_url,
        referrer=data.referrer,
        user_agent=data.user_agent,
        viewport=data.viewport,
        screenshot_path=screenshot_path,
        screenshot_url=screenshot_url,
    )
    session.add(record)
    await session.commit()
    await session.refresh(record)

    return FeedbackCreateResponse(
        id=record.id,
        status=record.status,
        created_at=record.created_at,
    )
