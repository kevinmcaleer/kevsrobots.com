"""Auth middleware — verifies Chatter JWT tokens.

Also exposes a 14-day account-age gate used by the parts catalog
(issue #121). The JWT only carries the `sub` claim today, so we fall back
to looking up the user's account creation date from Chatter and caching
it in-process for 1 hour. The lookup is hidden behind a single coroutine
so tests can monkeypatch it without spinning up an HTTP mock.
"""

from __future__ import annotations

import logging
import time
from datetime import datetime, timezone
from typing import Any, Optional

import httpx
from fastapi import Cookie, Depends, Header, HTTPException, status
from jose import JWTError, jwt

from .config import get_settings

logger = logging.getLogger(__name__)

# In-process cache: username -> (account_created_at | None, expires_epoch).
# A `None` value is cached too (briefly) so a Chatter outage doesn't melt
# us under retries; the cache TTL is short enough for stale entries to be
# self-healing.
_ACCOUNT_AGE_CACHE: dict[str, tuple[Optional[datetime], float]] = {}
_ACCOUNT_AGE_TTL_SECONDS = 3600  # 1 hour
_ACCOUNT_AGE_NEGATIVE_TTL_SECONDS = 60  # cache lookup failures briefly
MIN_ACCOUNT_AGE_DAYS = 14


def get_current_user(
    access_token: Optional[str] = Cookie(default=None),
    authorization: Optional[str] = Header(default=None),
) -> str:
    """Extract and verify the username from a Chatter JWT token.

    Checks the access_token cookie first (set by chatter with
    domain=.kevsrobots.com), then the Authorization header.
    Returns the username (sub claim) or raises 401.
    """

    settings = get_settings()
    token = None

    if access_token and access_token.startswith("Bearer "):
        token = access_token[7:]
    elif authorization and authorization.startswith("Bearer "):
        token = authorization[7:]

    if not token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated",
        )

    try:
        payload = jwt.decode(
            token, settings.jwt_secret, algorithms=[settings.jwt_algorithm]
        )
        username = payload.get("sub")
        if not username:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid token",
            )
        return username
    except JWTError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired token",
        )


def get_optional_user(
    access_token: Optional[str] = Cookie(default=None),
    authorization: Optional[str] = Header(default=None),
) -> Optional[str]:
    """Like get_current_user but returns None instead of raising 401."""

    try:
        return get_current_user(access_token, authorization)
    except HTTPException:
        return None


def require_admin(user: str = Depends(get_current_user)) -> str:
    """FastAPI dependency: 401 if not logged in, 403 if not in the admin list.

    Mirrors the pattern in ``routers/moderation.get_admin_user`` so new
    admin-only routers can wire admin gating with a single import. The
    ``ADMIN_USERNAMES`` env var (comma-separated) is the source of
    truth — see ``config.Settings.admin_usernames_list``.
    """
    settings = get_settings()
    if user not in settings.admin_usernames_list:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Admin access required",
        )
    return user


def _extract_token(access_token: Optional[str], authorization: Optional[str]) -> Optional[str]:
    if access_token and access_token.startswith("Bearer "):
        return access_token[7:]
    if authorization and authorization.startswith("Bearer "):
        return authorization[7:]
    return None


def extract_token(
    access_token: Optional[str] = Cookie(default=None),
    authorization: Optional[str] = Header(default=None),
) -> Optional[str]:
    """FastAPI dependency: return the raw bearer token, or None."""
    return _extract_token(access_token, authorization)


async def fetch_chatter_me(token: str) -> Optional[dict[str, Any]]:
    """Fetch the raw Chatter ``/api/me`` payload for the current user.

    Returns the JSON dict on success, or None on any network/parse
    failure. Tests monkeypatch this — keep the signature stable.

    We intentionally do not cache this: the payload may carry mutable
    fields (email, avatar_url) that we don't want to serve stale.
    Latency is bounded by the short httpx timeout.
    """
    settings = get_settings()
    url = f"{settings.chatter_base_url.rstrip('/')}/api/me"
    try:
        async with httpx.AsyncClient(timeout=5.0) as http:
            resp = await http.get(
                url,
                headers={"Authorization": f"Bearer {token}"},
            )
            if resp.status_code == 200:
                data = resp.json()
                if isinstance(data, dict):
                    return data
    except Exception as exc:  # network / parsing — best-effort
        logger.warning("Chatter /api/me lookup failed: %s", exc)
    return None


def _parse_iso8601(value: str) -> Optional[datetime]:
    """Parse an ISO-8601 timestamp from Chatter. Returns naive UTC."""
    try:
        # Accept trailing "Z" which fromisoformat() doesn't grok pre-3.11.
        if value.endswith("Z"):
            value = value[:-1] + "+00:00"
        dt = datetime.fromisoformat(value)
        if dt.tzinfo is not None:
            dt = dt.astimezone(timezone.utc).replace(tzinfo=None)
        return dt
    except (TypeError, ValueError):
        return None


async def fetch_account_created_at(username: str, token: str) -> Optional[datetime]:
    """Ask Chatter when this account was created.

    Returns a naive UTC datetime, or None if the field couldn't be
    determined (network error, missing field, etc.). Caches both
    successful and failed lookups in-process for a short TTL.

    Tests monkeypatch this function — keep the signature stable.
    """
    now = time.time()
    cached = _ACCOUNT_AGE_CACHE.get(username)
    if cached is not None and cached[1] > now:
        return cached[0]

    settings = get_settings()
    url = f"{settings.chatter_base_url.rstrip('/')}/api/me"
    created: Optional[datetime] = None
    try:
        async with httpx.AsyncClient(timeout=5.0) as http:
            resp = await http.get(
                url,
                headers={"Authorization": f"Bearer {token}"},
            )
            if resp.status_code == 200:
                data = resp.json()
                raw = data.get("account_created_at") or data.get("created_at")
                if isinstance(raw, str):
                    created = _parse_iso8601(raw)
                elif isinstance(raw, (int, float)):
                    created = datetime.utcfromtimestamp(raw)
    except Exception as exc:  # network / parsing — fail closed below
        logger.warning("Chatter account-age lookup failed for %s: %s", username, exc)

    ttl = _ACCOUNT_AGE_TTL_SECONDS if created is not None else _ACCOUNT_AGE_NEGATIVE_TTL_SECONDS
    _ACCOUNT_AGE_CACHE[username] = (created, now + ttl)
    return created


def _clear_account_age_cache() -> None:
    """Test helper: drop the in-process cache."""
    _ACCOUNT_AGE_CACHE.clear()


async def get_current_user_aged(
    access_token: Optional[str] = Cookie(default=None),
    authorization: Optional[str] = Header(default=None),
) -> str:
    """Like ``get_current_user`` but also enforces a 14-day account-age gate.

    Used by parts-catalog mutation endpoints (POST/PUT/restore). On any
    lookup failure we fail closed — better to ask the user to come back
    later than to let a brand-new account spam-edit the wiki.
    """
    username = get_current_user(access_token=access_token, authorization=authorization)
    token = _extract_token(access_token, authorization)
    if token is None:
        # get_current_user would have raised already, but be defensive.
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated",
        )

    created_at = await fetch_account_created_at(username, token)
    if created_at is None:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Account age cannot be verified",
        )

    now = datetime.utcnow()
    age_days = (now - created_at).total_seconds() / 86400.0
    if age_days < MIN_ACCOUNT_AGE_DAYS:
        remaining = max(0, MIN_ACCOUNT_AGE_DAYS - int(age_days))
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail=f"Account must be at least {MIN_ACCOUNT_AGE_DAYS} days old to edit the parts catalog. Come back in {remaining} day(s).",
        )
    return username
