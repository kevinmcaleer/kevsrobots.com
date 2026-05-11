"""Tests for `GET /api/recommendations` (#74)."""

from __future__ import annotations

from pathlib import Path

import pytest
from sqlalchemy import select

from nibsy_api.generator import generate_recommendations
from nibsy_api.ingest import ingest_from_data_dir
from nibsy_api.models import NibsyContent, NibsyRecommendation

FIXTURES = Path(__file__).resolve().parent / "fixtures" / "_data"


async def _seed_and_generate(session) -> None:
    await ingest_from_data_dir(FIXTURES, session)
    await generate_recommendations(session)


async def _first_source_with_recs(session) -> str:
    rec = await session.scalar(select(NibsyRecommendation))
    assert rec is not None, "no recommendations were generated"
    return rec.source_url


@pytest.mark.asyncio
async def test_get_recommendations_returns_list(client, session) -> None:
    """A page that has precomputed recs should return them."""

    await _seed_and_generate(session)
    source_url = await _first_source_with_recs(session)

    response = await client.get("/api/recommendations", params={"page": source_url})
    assert response.status_code == 200
    body = response.json()
    assert body["source_url"] == source_url
    assert isinstance(body["recommendations"], list)
    assert len(body["recommendations"]) >= 1
    # Each item carries the agreed shape.
    item = body["recommendations"][0]
    for field in ("content_id", "url", "title", "type", "reason"):
        assert field in item, f"missing field {field} in recommendation item"


@pytest.mark.asyncio
async def test_get_recommendations_empty_for_unknown_page(client, session) -> None:
    """Unknown pages get 200 + empty list, not 404."""

    await _seed_and_generate(session)

    response = await client.get(
        "/api/recommendations", params={"page": "/totally/unknown/page.html"}
    )
    assert response.status_code == 200
    body = response.json()
    assert body["source_url"] == "/totally/unknown/page.html"
    assert body["recommendations"] == []
    assert body["generated_at"] is None
    assert body["generator_version"] is None


@pytest.mark.asyncio
async def test_get_recommendations_respects_limit(client, session) -> None:
    """`limit=2` caps the response at two items."""

    await _seed_and_generate(session)
    source_url = await _first_source_with_recs(session)

    response = await client.get(
        "/api/recommendations",
        params={"page": source_url, "limit": 2},
    )
    assert response.status_code == 200
    body = response.json()
    assert len(body["recommendations"]) <= 2


@pytest.mark.asyncio
async def test_get_recommendations_respects_exclude(client, session) -> None:
    """`exclude=<id>` filters that content_id out of the response."""

    await _seed_and_generate(session)
    # Find a source with at least 2 recs so excluding one still leaves something.
    rec = await session.scalar(
        select(NibsyRecommendation)
    )
    assert rec is not None
    assert len(rec.recommendations) >= 2, "need at least 2 recs for this test"
    excluded_id = rec.recommendations[0]["content_id"]

    response = await client.get(
        "/api/recommendations",
        params={"page": rec.source_url, "exclude": str(excluded_id)},
    )
    assert response.status_code == 200
    body = response.json()
    returned_ids = {e["content_id"] for e in body["recommendations"]}
    assert excluded_id not in returned_ids
