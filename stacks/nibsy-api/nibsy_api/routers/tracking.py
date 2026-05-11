"""Click and impression tracking endpoints (#68)."""

from __future__ import annotations

import hashlib
from typing import Optional

from fastapi import APIRouter, Depends, Request
from pydantic import BaseModel, Field
from sqlalchemy.ext.asyncio import AsyncSession

from ..db import get_session
from ..models import NibsyClick, NibsyImpression

router = APIRouter(prefix="/api/track", tags=["tracking"])


class ClickRequest(BaseModel):
    content_url: str
    source_page: str
    content_id: Optional[int] = None
    session_id: Optional[str] = None


class ImpressionRequest(BaseModel):
    content_ids: list[int]
    source_page: str
    session_id: Optional[str] = None


def _anonymize_ip(ip: Optional[str]) -> Optional[str]:
    if not ip:
        return None
    return hashlib.sha256(ip.encode()).hexdigest()[:16]


@router.post("/click")
async def track_click(
    body: ClickRequest,
    request: Request,
    session: AsyncSession = Depends(get_session),
) -> dict:
    click = NibsyClick(
        content_id=body.content_id,
        content_url=body.content_url,
        source_page=body.source_page,
        session_id=body.session_id,
        ip_address=_anonymize_ip(request.client.host if request.client else None),
        user_agent=request.headers.get("user-agent"),
    )
    session.add(click)
    await session.commit()
    return {"status": "ok"}


@router.post("/impression")
async def track_impression(
    body: ImpressionRequest,
    request: Request,
    session: AsyncSession = Depends(get_session),
) -> dict:
    for cid in body.content_ids:
        session.add(
            NibsyImpression(
                content_id=cid,
                source_page=body.source_page,
                session_id=body.session_id,
            )
        )
    await session.commit()
    return {"status": "ok", "recorded": len(body.content_ids)}
