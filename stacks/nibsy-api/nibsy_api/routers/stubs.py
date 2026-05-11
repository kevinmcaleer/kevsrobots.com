"""501 stubs for endpoints deferred to follow-up PRs.

Each route returns HTTP 501 Not Implemented and points to the GitHub issue
that will land it. Returning 501 (rather than letting FastAPI 404) means
the widget integration (#71) gets a clear signal during early integration.
"""

from __future__ import annotations

from fastapi import APIRouter, HTTPException, status

router = APIRouter(prefix="/api", tags=["stubs"])


def _not_implemented(issue: str) -> None:
    raise HTTPException(
        status_code=status.HTTP_501_NOT_IMPLEMENTED,
        detail=f"Not implemented — see {issue}",
    )




