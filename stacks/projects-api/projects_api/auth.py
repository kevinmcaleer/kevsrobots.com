"""Auth middleware — verifies Chatter JWT tokens.

Also exposes a 14-day account-age gate used by the parts catalog
(issue #121). The JWT only carries the `sub` claim today, so we fall back
to looking up the user's account creation date from Chatter and caching
it in-process for 1 hour. The lookup is hidden behind a single coroutine
so tests can monkeypatch it without spinning up an HTTP mock.
"""

from __future__ import annotations

import hmac
import logging
import time
from datetime import datetime, timezone
from typing import Any, Optional

import httpx
from fastapi import Cookie, Depends, Header, HTTPException, status
from jose import JWTError, jwt
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from .config import get_settings
from .db import get_session

logger = logging.getLogger(__name__)

# In-process cache: username -> (account_created_at | None, expires_epoch).
# A `None` value is cached too (briefly) so a Chatter outage doesn't melt
# us under retries; the cache TTL is short enough for stale entries to be
# self-healing.
_ACCOUNT_AGE_CACHE: dict[str, tuple[Optional[datetime], float]] = {}
_ACCOUNT_AGE_TTL_SECONDS = 3600  # 1 hour
_ACCOUNT_AGE_NEGATIVE_TTL_SECONDS = 60  # cache lookup failures briefly
MIN_ACCOUNT_AGE_DAYS = 14


def _decode_username(
    access_token: Optional[str],
    authorization: Optional[str],
) -> str:
    """Extract and verify the username from a Chatter JWT token.

    Pure JWT decode — no DB access. Used internally; callers that want the
    full live check (including ``is_disabled``) should use
    :func:`get_current_user`.
    """

    settings = get_settings()
    token = _extract_token(access_token, authorization)

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


async def _is_user_disabled(
    session: AsyncSession, username: str
) -> tuple[bool, Optional[str]]:
    """Look up ``(is_disabled, disabled_reason)`` for ``username``.

    Returns ``(False, None)`` if there's no row for this user (never been
    flagged) or if the lookup fails — we fail *open* on transient DB
    errors so we don't lock everyone out of the API just because a query
    timed out. The auto-disable path itself runs in the same transaction
    as the mutation it's blocking, so it has stronger guarantees.
    """
    # Local import to avoid a circular: models -> db -> auth -> models.
    from .models import User

    try:
        row = await session.scalar(
            select(User).where(User.username == username)
        )
        if row is None:
            return False, None
        return bool(row.is_disabled), row.disabled_reason
    except Exception as exc:
        logger.warning("Disabled-user lookup failed for %s: %s", username, exc)
        return False, None


async def get_current_user(
    access_token: Optional[str] = Cookie(default=None),
    authorization: Optional[str] = Header(default=None),
    session: AsyncSession = Depends(get_session),
) -> str:
    """Extract and verify the username from a Chatter JWT token.

    Checks the access_token cookie first (set by chatter with
    domain=.kevsrobots.com), then the Authorization header.
    Returns the username (sub claim) or raises 401.

    Also enforces the local ``users.is_disabled`` flag — disabled accounts
    get 403 on every authenticated endpoint (issue #136).
    """
    username = _decode_username(access_token, authorization)
    disabled, reason = await _is_user_disabled(session, username)
    if disabled:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail=reason or "Account disabled",
        )
    return username


async def get_optional_user(
    access_token: Optional[str] = Cookie(default=None),
    authorization: Optional[str] = Header(default=None),
    session: AsyncSession = Depends(get_session),
) -> Optional[str]:
    """Like get_current_user but returns None instead of raising 401.

    Disabled accounts still raise 403 — we don't want a disabled user to
    silently drop to anonymous and keep using endpoints that vary by
    auth state.
    """

    try:
        return await get_current_user(access_token, authorization, session)
    except HTTPException as exc:
        if exc.status_code == status.HTTP_403_FORBIDDEN:
            raise
        return None


async def _check_terms_accepted(session: AsyncSession, user: str) -> None:
    """Raise 403 ``terms_not_accepted`` if ``user`` hasn't accepted the
    current T&Cs version.

    Lazily creates a ``users`` row with ``is_disabled=False`` and
    ``terms_accepted_at=None`` when none exists yet, so the subsequent
    ``POST /api/users/me/accept-terms`` call can update it in place.

    Shared by :func:`require_terms_accepted` and
    :func:`require_terms_accepted_aged` so the gate behaviour is identical
    regardless of which auth dependency the route uses.
    """
    # Local import to avoid the models -> db -> auth -> models cycle.
    from .models import User

    settings = get_settings()
    current_version = settings.current_terms_version

    row = await session.scalar(select(User).where(User.username == user))
    if row is None:
        # Lazily create the row so accept-terms can update it later.
        row = User(username=user, is_disabled=False)
        session.add(row)
        try:
            await session.commit()
        except Exception:  # noqa: BLE001 — racing duplicate insert is fine
            await session.rollback()
            row = await session.scalar(
                select(User).where(User.username == user)
            )

    accepted = (
        row is not None
        and row.terms_accepted_at is not None
        and row.terms_accepted_version == current_version
    )
    if not accepted:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail={
                "detail": "terms_not_accepted",
                "current_version": current_version,
            },
        )


async def require_terms_accepted(
    user: str = Depends(get_current_user),
    session: AsyncSession = Depends(get_session),
) -> str:
    """FastAPI dependency: 403 when the user hasn't accepted the current T&Cs.

    The shape of the 403 body is a stable contract the frontend depends on
    to detect "this is a terms-acceptance problem, not a generic auth
    error"::

        {"detail": "terms_not_accepted", "current_version": "1.0"}

    A user is considered "accepted" when their ``users`` row has a non-null
    ``terms_accepted_at`` AND ``terms_accepted_version`` exactly matches
    ``settings.current_terms_version``. Older versions are treated as not
    accepted so a version bump forces everyone to re-agree.

    Edge case: if the ``users`` row doesn't exist yet (first write from a
    brand-new account), we auto-create it with ``is_disabled=False`` and
    ``terms_accepted_at=None`` and THEN raise the 403. This keeps the row
    available for the subsequent ``POST /api/users/me/accept-terms`` call.
    """
    await _check_terms_accepted(session, user)
    return user


async def require_terms_accepted_aged(
    access_token: Optional[str] = Cookie(default=None),
    authorization: Optional[str] = Header(default=None),
    session: AsyncSession = Depends(get_session),
) -> str:
    """Combined dependency: 14-day account-age gate + T&Cs acceptance gate.

    Used by parts-catalog mutation endpoints (POST/PUT/restore) and the
    parts-moderation merge endpoints — they already required
    :func:`get_current_user_aged`, this wrapper layers the terms-acceptance
    check on top so both gates fire in the right order (auth -> disabled
    -> age -> terms). The 403 ``terms_not_accepted`` payload shape matches
    :func:`require_terms_accepted` so the frontend's gate-handling path
    works regardless of which dependency raised it.
    """
    username = await get_current_user_aged(
        access_token=access_token,
        authorization=authorization,
        session=session,
    )
    await _check_terms_accepted(session, username)
    return username


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


# Sentinel admin identity recorded when a request authenticates via the
# first-party desktop admin key rather than a real Chatter admin account — so
# "read/closed by" audit fields are clearly attributable (never a real username).
SNAKIE_ADMIN = "_snakie_admin"


def _is_snakie_admin(x_snakie_admin_key: Optional[str]) -> bool:
    """True when the maintainer's desktop app presents the configured
    ``X-Snakie-Admin-Key``.

    Constant-time compared on UTF-8 bytes (``hmac.compare_digest`` rejects
    ``str`` inputs with non-ASCII characters, which would otherwise 500). Returns
    False when no key is configured, so the desktop admin path is DISABLED by
    default and the inbox stays Chatter-admin-only.
    """
    key = get_settings().snakie_admin_key
    if not key or not x_snakie_admin_key:
        return False
    return hmac.compare_digest(x_snakie_admin_key.encode("utf-8"), key.encode("utf-8"))


async def require_feedback_admin(
    x_snakie_admin_key: Optional[str] = Header(default=None, alias="X-Snakie-Admin-Key"),
    access_token: Optional[str] = Cookie(default=None),
    authorization: Optional[str] = Header(default=None),
    session: AsyncSession = Depends(get_session),
) -> str:
    """Admin gate for the feedback inbox accepting EITHER auth path:

    * the maintainer's FIRST-PARTY desktop app (Snakie's dev-mode Bug Tracker)
      presenting the shared ``X-Snakie-Admin-Key`` — returns :data:`SNAKIE_ADMIN`.
      Gated by ``snakie_admin_key`` so it's off unless configured;
    * a logged-in Chatter ADMIN (the website inbox) — same rule as
      :func:`require_admin`.

    Returns the resolved admin identity. 401 when neither auth is present, 403
    when a Chatter user isn't an admin.
    """
    if _is_snakie_admin(x_snakie_admin_key):
        return SNAKIE_ADMIN
    username = await get_current_user(access_token, authorization, session)
    settings = get_settings()
    if username not in settings.admin_usernames_list:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Admin access required",
        )
    return username


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
    session: AsyncSession = Depends(get_session),
) -> str:
    """Like ``get_current_user`` but also enforces a 14-day account-age gate.

    Used by parts-catalog mutation endpoints (POST/PUT/restore). On any
    lookup failure we fail closed — better to ask the user to come back
    later than to let a brand-new account spam-edit the wiki.
    """
    username = await get_current_user(
        access_token=access_token,
        authorization=authorization,
        session=session,
    )
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
