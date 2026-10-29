"""Who's asking: the same Chatter JWT the Projects API accepts.

The token comes from the ``access_token`` cookie (set by Chatter for
``.kevsrobots.com``) or, for local development, an ``Authorization: Bearer``
header (``ProjectAuth.apiFetch`` sends the dev token that way). There's no
database here, so a disabled account is only stopped by the Projects API;
this service only proofreads text the writer already has.
"""

from __future__ import annotations

from typing import Optional

from fastapi import Cookie, Header, HTTPException, status
from jose import JWTError, jwt

from .config import get_settings


def _extract_token(access_token: Optional[str], authorization: Optional[str]) -> Optional[str]:
    if access_token:
        return access_token
    if authorization and authorization.lower().startswith("bearer "):
        return authorization[7:].strip() or None
    return None


async def get_current_user(
    access_token: Optional[str] = Cookie(default=None),
    authorization: Optional[str] = Header(default=None),
) -> str:
    """The username (``sub`` claim) of a valid session, or 401."""
    token = _extract_token(access_token, authorization)
    if not token:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Not authenticated")
    settings = get_settings()
    try:
        payload = jwt.decode(token, settings.jwt_secret, algorithms=[settings.jwt_algorithm])
    except JWTError:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Invalid or expired token")
    username = payload.get("sub")
    if not username:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Invalid token")
    return username
