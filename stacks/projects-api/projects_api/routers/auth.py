"""Auth introspection endpoints — issue #139.

Exposes ``GET /api/auth/me`` so the frontend can detect login state in a
single round-trip rather than probing a protected resource. The username
and ``is_admin`` flag are derived locally (JWT ``sub`` + the
``ADMIN_USERNAMES`` allow-list); the rest of the profile (``id``,
``email``, ``created_at``, ``avatar_url``) is enriched from Chatter's
``/api/me`` endpoint when reachable, and falls back to ``None`` on any
failure so a Chatter outage doesn't make our endpoint 500.

Cache-Control is always ``no-store`` — this is per-user data that must
never be cached at the CDN or in the browser.
"""

from __future__ import annotations

from datetime import datetime, timezone
from typing import Any, Optional

from fastapi import APIRouter, Depends, Response
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from ..auth import extract_token, fetch_chatter_me, get_current_user
from ..config import get_settings
from ..db import get_session
from ..models import User

router = APIRouter(tags=["auth"])


def _parse_iso8601(value: Any) -> Optional[str]:
    """Normalise a Chatter timestamp into an ISO-8601 ``...Z`` string.

    Accepts strings (with or without trailing ``Z``) and numeric epochs.
    Returns ``None`` if the value can't be coerced.
    """
    if value is None:
        return None
    if isinstance(value, (int, float)):
        try:
            return datetime.utcfromtimestamp(value).strftime("%Y-%m-%dT%H:%M:%SZ")
        except (OverflowError, OSError, ValueError):
            return None
    if isinstance(value, str):
        raw = value
        try:
            if raw.endswith("Z"):
                raw = raw[:-1] + "+00:00"
            dt = datetime.fromisoformat(raw)
            if dt.tzinfo is not None:
                dt = dt.astimezone(timezone.utc).replace(tzinfo=None)
            return dt.strftime("%Y-%m-%dT%H:%M:%SZ")
        except ValueError:
            # Last-ditch: return the original string if it at least
            # vaguely looks like a timestamp.
            return value if "T" in value or "-" in value else None
    return None


@router.get("/api/auth/me")
async def auth_me(
    response: Response,
    username: str = Depends(get_current_user),
    token: Optional[str] = Depends(extract_token),
    session: AsyncSession = Depends(get_session),
) -> dict[str, Any]:
    """Return the authenticated user's profile.

    Raises 401 (via :func:`get_current_user`) if no valid session token
    is present. On success, enriches the local view (username,
    is_admin) with Chatter's ``/api/me`` payload when available.

    Also exposes the T&Cs acceptance state (terms-gate) so the frontend
    can decide upfront whether to block uploads — no need to wait for a
    write to 403 with ``terms_not_accepted`` first.
    """
    response.headers["Cache-Control"] = "no-store"

    settings = get_settings()
    is_admin = username in settings.admin_usernames_list

    chatter: Optional[dict[str, Any]] = None
    if token:
        chatter = await fetch_chatter_me(token)

    if chatter is None:
        chatter = {}

    # Chatter is the source of truth for id, email, created_at, avatar_url.
    # If unreachable, return None for those fields rather than 500.
    user_id = chatter.get("id")
    if not isinstance(user_id, int):
        user_id = None

    email = chatter.get("email")
    if not isinstance(email, str):
        email = None

    created_at = _parse_iso8601(
        chatter.get("account_created_at") or chatter.get("created_at")
    )

    avatar_url = chatter.get("avatar_url")
    if not isinstance(avatar_url, str):
        avatar_url = None

    # T&Cs acceptance — read the local users row (lazy; null when missing).
    terms_accepted_at: Optional[str] = None
    terms_accepted_version: Optional[str] = None
    user_row = await session.scalar(select(User).where(User.username == username))
    if user_row is not None:
        if user_row.terms_accepted_at is not None:
            terms_accepted_at = user_row.terms_accepted_at.strftime(
                "%Y-%m-%dT%H:%M:%SZ"
            )
        terms_accepted_version = user_row.terms_accepted_version

    return {
        "id": user_id,
        "username": username,
        "email": email,
        "is_admin": is_admin,
        "created_at": created_at,
        "avatar_url": avatar_url,
        "terms_accepted_at": terms_accepted_at,
        "terms_accepted_version": terms_accepted_version,
        "current_terms_version": settings.current_terms_version,
    }
