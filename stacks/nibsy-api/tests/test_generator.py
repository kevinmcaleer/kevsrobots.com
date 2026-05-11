"""Tests for the heuristic recommendations generator (#74)."""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path

import pytest
from sqlalchemy import select

from nibsy_api.generator import GENERATOR_VERSION, generate_recommendations
from nibsy_api.ingest import ingest_from_data_dir
from nibsy_api.models import NibsyContent, NibsyRecommendation

FIXTURES = Path(__file__).resolve().parent / "fixtures" / "_data"


async def _seed(session) -> None:
    """Populate the content table from the bundled fixtures."""

    await ingest_from_data_dir(FIXTURES, session)


@pytest.mark.asyncio
async def test_generate_writes_recommendations(session) -> None:
    """Generation should produce a `nibsy_recommendations` row per content row."""

    await _seed(session)
    stats = await generate_recommendations(session)

    assert stats.generator_version == GENERATOR_VERSION
    assert stats.sources_processed > 0
    assert stats.recommendations_written > 0
    assert stats.duration_ms >= 0

    content_count = len((await session.scalars(select(NibsyContent))).all())
    rec_rows = (await session.scalars(select(NibsyRecommendation))).all()
    # Every source should have a row unless it was skipped because no
    # candidate scored above zero — with the fixtures, none should skip.
    assert len(rec_rows) == content_count - stats.sources_skipped


@pytest.mark.asyncio
async def test_generate_no_self_recommendation(session) -> None:
    """No source should recommend itself."""

    await _seed(session)
    await generate_recommendations(session)

    rows = (await session.scalars(select(NibsyRecommendation))).all()
    assert rows, "no recommendations written"
    for row in rows:
        for entry in row.recommendations:
            assert entry["url"] != row.source_url, (
                f"self-recommendation found for {row.source_url}"
            )


@pytest.mark.asyncio
async def test_generate_content_type_mixing(session) -> None:
    """For sources with diverse candidates, the output should mix types."""

    await _seed(session)
    await generate_recommendations(session)

    # Pick the source with the most recommendations and assert its types
    # aren't monocultural. With ~17 fixture rows across 6 types this
    # should always have at least 2 distinct types in the top-N.
    rows = (await session.scalars(select(NibsyRecommendation))).all()
    biggest = max(rows, key=lambda r: len(r.recommendations))
    distinct_types = {e["type"] for e in biggest.recommendations}
    assert len(distinct_types) >= 2, (
        f"expected ≥2 content types in top recs for {biggest.source_url}, "
        f"got {distinct_types}"
    )


@pytest.mark.asyncio
async def test_generate_removes_orphans(session) -> None:
    """Pre-existing recommendation rows for deleted content should be wiped."""

    await _seed(session)

    # Inject an orphan — a `nibsy_recommendations` row whose source_url
    # has no matching `nibsy_content` row.
    session.add(
        NibsyRecommendation(
            source_url="/orphan/page-that-no-longer-exists.html",
            recommendations=[
                {
                    "content_id": 9999,
                    "url": "/whatever",
                    "title": "x",
                    "type": "blog",
                    "reason": "stale",
                }
            ],
            generated_at=datetime.now(timezone.utc).replace(tzinfo=None),
            generator_version="ancient",
        )
    )
    await session.commit()

    stats = await generate_recommendations(session)
    assert stats.removed_orphans >= 1

    surviving = await session.scalar(
        select(NibsyRecommendation).where(
            NibsyRecommendation.source_url == "/orphan/page-that-no-longer-exists.html"
        )
    )
    assert surviving is None


@pytest.mark.asyncio
async def test_generate_is_idempotent(session) -> None:
    """Re-running generation should produce the same payload (modulo time)."""

    await _seed(session)
    await generate_recommendations(session)
    rows1 = (await session.scalars(select(NibsyRecommendation))).all()
    snapshot1 = {
        r.source_url: sorted(r.recommendations, key=lambda e: e["content_id"])
        for r in rows1
    }

    await generate_recommendations(session)
    rows2 = (await session.scalars(select(NibsyRecommendation))).all()
    snapshot2 = {
        r.source_url: sorted(r.recommendations, key=lambda e: e["content_id"])
        for r in rows2
    }

    assert snapshot1.keys() == snapshot2.keys()
    for url, entries1 in snapshot1.items():
        entries2 = snapshot2[url]
        # Compare the meaningful fields — generated_at will differ.
        assert [
            (e["content_id"], e["url"], e["type"], e["reason"]) for e in entries1
        ] == [
            (e["content_id"], e["url"], e["type"], e["reason"]) for e in entries2
        ], f"recommendations diverged on re-run for {url}"
