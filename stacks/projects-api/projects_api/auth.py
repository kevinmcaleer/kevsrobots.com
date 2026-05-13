"""Auth middleware — verifies Chatter JWT tokens."""

from __future__ import annotations

from typing import Optional

from fastapi import Cookie, Header, HTTPException, status
from jose import JWTError, jwt

from .config import get_settings


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
