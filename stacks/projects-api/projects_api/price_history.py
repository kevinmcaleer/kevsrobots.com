"""Supplier price history (issues #202, #267).

``record_price`` writes a ``PartPriceSnapshot`` when a supplier's price
differs from its last recorded one, so calling it after every edit and
every daily check yields one row per actual change.
"""

from __future__ import annotations

from datetime import datetime, timezone
from typing import Optional

from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from .models import PartPriceSnapshot, PartSupplier


async def last_snapshot(session: AsyncSession, supplier_id: int) -> Optional[PartPriceSnapshot]:
    return await session.scalar(
        select(PartPriceSnapshot)
        .where(PartPriceSnapshot.supplier_id == supplier_id)
        .order_by(PartPriceSnapshot.recorded_at.desc(), PartPriceSnapshot.id.desc())
        .limit(1)
    )


async def record_price(session: AsyncSession, supplier: PartSupplier) -> bool:
    """Snapshot ``supplier``'s price if it has one and it changed. The
    supplier must have an id (flush first). Caller commits."""
    if supplier.unit_cost is None or supplier.id is None:
        return False
    last = await last_snapshot(session, supplier.id)
    if last is not None and last.unit_cost == supplier.unit_cost and last.currency_code == supplier.currency_code:
        return False
    session.add(PartPriceSnapshot(
        part_id=supplier.part_id, supplier_id=supplier.id,
        unit_cost=supplier.unit_cost, currency_code=supplier.currency_code,
        recorded_at=datetime.now(timezone.utc).replace(tzinfo=None),
    ))
    return True
