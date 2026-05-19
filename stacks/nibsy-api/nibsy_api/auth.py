"""Auth middleware for Nibsy admin endpoints.

Lifted from ``stacks/projects-api/projects_api/auth.py`` — projects-api is
the source of truth for the Chatter-aware auth pattern. Nibsy keeps its
own copy so the two services can evolve independently and so this service
doesn't need a hard dependency on the projects-api package.

The Chatter cookie (``access_token``) is set on the parent domain
``.kevsrobots.com``, so any subdomain on that domain receives it on
cross-origin requests when the browser sends them with credentials. We
verify the JWT locally and (optionally) cross-check with Chatter's
``/api/me``.

For #158 only ``get_current_user`` and ``require_admin`` are used by the
admin router; the rest of the helpers are kept for future endpoints.
"""

from __future__ import annotations

import logging
from typing import Any, Optional

import httpx
from fastapi import Cookie, Depends, Header, HTTPException, status
from jose import JWTError, jwt

from .config import get_settings

logger = logging.getLogger(__name__)


def _extract_token(
    access_token: Optional[str], authorization: Optional[str]
) -> Optional[str]:
    """Pull the raw JWT from either a cookie or the Authorization header.

    Chatter sets the cookie value as ``Bearer <jwt>`` (note the leading
    "Bearer "), so we strip the prefix in both forms.
    """

    if access_token and access_token.startswith("Bearer "):
        return access_token[7:]
    if access_token:
        # Some clients store the raw token in the cookie without prefix.
        return access_token
    if authorization and authorization.startswith("Bearer "):
        return authorization[7:]
    return None


def _decode_username(
    access_token: Optional[str],
    authorization: Optional[str],
) -> str:
    """Verify the JWT signature and pull ``sub`` (username)."""

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


async def fetch_chatter_me(token: str) -> Optional[dict[str, Any]]:
    """Fetch ``/api/me`` from Chatter for the user owning ``token``.

    Returns the JSON payload on success, or ``None`` on any network /
    parse / non-200 response. Tests monkeypatch this — keep the signature
    stable. ~3s timeout so we don't hang the request thread when Chatter
    is misbehaving.
    """

    settings = get_settings()
    url = f"{settings.chatter_base_url.rstrip('/')}/api/me"
    try:
        async with httpx.AsyncClient(timeout=3.0) as http:
            resp = await http.get(
                url,
                headers={"Authorization": f"Bearer {token}"},
            )
            if resp.status_code == 200:
                data = resp.json()
                if isinstance(data, dict):
                    return data
            else:
                logger.warning(
                    "Chatter /api/me returned %s for nibsy admin auth",
                    resp.status_code,
                )
    except Exception as exc:  # network / parsing — best-effort
        logger.warning("Chatter /api/me lookup failed: %s", exc)
    return None


async def get_current_user(
    access_token: Optional[str] = Cookie(default=None),
    authorization: Optional[str] = Header(default=None),
) -> str:
    """Resolve the current user from a cookie or bearer header.

    Strategy:
      1. Verify the JWT signature locally to extract ``sub`` (cheap,
         no network call) — gives a quick 401 on bad/expired tokens.
      2. Return the verified username. The local check is sufficient
         because Chatter signs every token with ``JWT_SECRET``; we trust
         our own signature.

    Notes:
      - Anonymous requests (no token at all) raise 401.
      - Bad signature or expired token raise 401.
      - We deliberately do *not* call ``fetch_chatter_me`` on every
        request — Chatter would become a single point of failure for
        every admin write. The local JWT verification is the same model
        projects-api uses.
    """

    return _decode_username(access_token, authorization)


def require_admin(user: str = Depends(get_current_user)) -> str:
    """FastAPI dependency: 401 if not logged in, 403 if not in admin list.

    Mirrors ``projects_api.auth.require_admin``. The ``ADMIN_USERNAMES``
    env var (comma-separated) is the source of truth — see
    ``config.Settings.admin_usernames_list``.
    """

    settings = get_settings()
    if user not in settings.admin_usernames_list:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Admin access required",
        )
    return user
