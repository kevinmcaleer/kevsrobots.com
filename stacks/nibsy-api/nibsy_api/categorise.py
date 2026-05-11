"""Manual AI categorisation helpers (#75).

Provides export/import functions for a manual categorisation workflow:
1. Export content as JSON for the user to categorise via Claude
2. Import structured categorisation results back into nibsy_content.metadata

The user runs categorisation manually using their Claude AI Max
subscription rather than automated API calls — this keeps costs at
zero and gives full control over the prompt and results.
"""

from __future__ import annotations

import logging
from datetime import datetime, timezone
from typing import Any

from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from .models import NibsyContent

logger = logging.getLogger(__name__)

CATEGORISATION_VERSION = "manual-v1"


async def export_for_categorisation(
    session: AsyncSession,
    force: bool = False,
) -> list[dict[str, Any]]:
    """Export content items that need categorisation.

    Skips items whose categorisation_version matches the current version
    unless force=True.
    """

    contents = (await session.scalars(select(NibsyContent))).all()
    items = []
    for c in contents:
        meta = c.content_metadata or {}
        cat = meta.get("categorisation", {})
        if not force and cat.get("categorisation_version") == CATEGORISATION_VERSION:
            continue
        items.append({
            "id": c.id,
            "content_type": c.content_type,
            "title": c.title,
            "url": c.url,
            "description": c.description,
            "tags": c.tags or [],
        })
    return items


async def import_categorisation(
    session: AsyncSession,
    results: list[dict[str, Any]],
) -> dict[str, int]:
    """Import categorisation results into nibsy_content.metadata.

    Each result should have:
    - id: int (content ID)
    - topics: list[str] (extracted topic tags)
    - difficulty: str (beginner/intermediate/advanced)
    - introduces: list[str] (concepts this page teaches)
    - assumes: list[str] (prerequisites)
    - pathway: dict (optional, {name, order, total} for course sequencing)
    """

    stats = {"updated": 0, "skipped": 0, "not_found": 0}
    now = datetime.now(timezone.utc).replace(tzinfo=None)

    for result in results:
        content_id = result.get("id")
        if content_id is None:
            stats["skipped"] += 1
            continue

        content = await session.get(NibsyContent, content_id)
        if content is None:
            stats["not_found"] += 1
            continue

        meta = dict(content.content_metadata or {})
        meta["categorisation"] = {
            "topics": result.get("topics", []),
            "difficulty": result.get("difficulty"),
            "introduces": result.get("introduces", []),
            "assumes": result.get("assumes", []),
            "categorisation_version": CATEGORISATION_VERSION,
            "categorised_at": now.isoformat(),
        }

        if result.get("pathway"):
            meta["pathway"] = result["pathway"]

        # Merge extracted topics into the content's tags for the scorer.
        existing_tags = set(content.tags or [])
        new_topics = set(result.get("topics", []))
        merged_tags = sorted(existing_tags | new_topics)
        content.tags = merged_tags

        content.content_metadata = meta
        content.updated_at = now
        stats["updated"] += 1

    await session.commit()
    logger.info("categorisation import: %s", stats)
    return stats
