"""Heuristic recommendations generator (issue #74).

Walks `nibsy_content`, scores every (source, target) pair using a small set
of cheap signals — tag overlap, content-type affinity, recency — and writes
the top-6 per source URL into `nibsy_recommendations`.

No LLM is involved here by design. The richer AI categorisation pass that
swaps in better tags lives in #75 and is orthogonal: this generator will
benefit from it automatically once it lands, because it reads whatever is
in `nibsy_content.tags`.

Algorithm version is recorded on every row so consumers can invalidate
caches when the scoring changes. Bump `GENERATOR_VERSION` whenever the
output of `_score` could differ for the same inputs.
"""

from __future__ import annotations

import logging
import time
from datetime import datetime, timezone
from typing import Any, Iterable, Optional

from sqlalchemy import delete, select
from sqlalchemy.ext.asyncio import AsyncSession

from .models import NibsyContent, NibsyRecommendation
from .schemas import GenerationStats

logger = logging.getLogger(__name__)


GENERATOR_VERSION = "heuristic-v0"

# Up to 6 recommendations per source URL.
TOP_N = 6

# Soft penalty multiplier applied to a candidate's score for each item of
# the same content_type already chosen. 0.8 ** 1 = 0.8, ** 2 = 0.64, etc.
# Breaks monocultures without imposing a hard quota.
TYPE_REPEAT_PENALTY = 0.8

# Per-component weights in the linear combination below. Tuned by hand on
# the fixtures during development; expect to re-tune once #75 enriches
# `tags` with LLM-extracted topics.
W_TAG_OVERLAP = 1.0
W_AFFINITY = 0.5
W_RECENCY = 0.3

# Affinity matrix keyed by (source_type, target_type). Default 0.5 for any
# pair not listed — we lean optimistic so unlikely-seeming pairs still get
# considered if tags align.
_AFFINITY: dict[tuple[str, str], float] = {
    ("course", "course"): 0.5,
    ("course", "video"): 0.8,
    ("course", "blog"): 0.6,
    ("course", "post"): 0.6,
    ("blog", "blog"): 0.7,
    ("post", "post"): 0.7,
    ("blog", "course"): 0.5,
    ("post", "course"): 0.5,
    ("video", "video"): 0.4,
    ("video", "course"): 0.7,
    ("review", "robot"): 0.9,
    ("review", "review"): 0.4,
    ("robot", "course"): 0.8,
    ("robot", "robot"): 0.5,
    ("robot", "review"): 0.7,
    # Glossary terms are a stepping stone into deeper content — treat them
    # as a low-weight bridge to/from everything.
    ("glossary", "course"): 0.3,
    ("glossary", "blog"): 0.3,
    ("glossary", "post"): 0.3,
    ("glossary", "video"): 0.3,
    ("course", "glossary"): 0.3,
    ("blog", "glossary"): 0.3,
    ("post", "glossary"): 0.3,
    ("video", "glossary"): 0.3,
}
_AFFINITY_DEFAULT = 0.5


def _affinity(src_type: str, dst_type: str) -> float:
    return _AFFINITY.get((src_type, dst_type), _AFFINITY_DEFAULT)


def _jaccard(a: Optional[Iterable[str]], b: Optional[Iterable[str]]) -> float:
    """Jaccard index between two iterables of tags. Zero if both empty."""

    sa = set(a or [])
    sb = set(b or [])
    if not sa and not sb:
        return 0.0
    union = sa | sb
    if not union:
        return 0.0
    return len(sa & sb) / len(union)


def _recency(date_published: Optional[datetime], now: datetime) -> float:
    """Inverse-age bonus. New content gets ~1.0; older asymptotes to 0."""

    if date_published is None:
        return 0.0
    days = max(0, (now - date_published).days)
    return min(1.0, 1.0 / (1.0 + days / 365.0))


def _score(
    source: NibsyContent,
    target: NibsyContent,
    now: datetime,
) -> tuple[float, set[str]]:
    """Return (score, shared_tags) for the source→target pair."""

    sa = set(source.tags or [])
    sb = set(target.tags or [])
    shared = sa & sb
    tag_overlap = _jaccard(source.tags, target.tags)
    affinity = _affinity(source.content_type, target.content_type)
    recency = _recency(target.date_published, now)
    score = (
        W_TAG_OVERLAP * tag_overlap
        + W_AFFINITY * affinity
        + W_RECENCY * recency
    )
    return score, shared


def _reason(
    source: NibsyContent,
    target: NibsyContent,
    shared_tags: set[str],
) -> str:
    """Short human-readable reason. Two templates plus a fallback."""

    if shared_tags:
        # Cap at 3 tags so the reason stays short in the UI.
        sample = sorted(shared_tags)[:3]
        return f"Shares tags: {', '.join(sample)}"
    if target.date_published is not None:
        # No tag overlap but the target is fresh enough to surface.
        return f"Recent {target.content_type}"
    return f"Related {target.content_type}"


def _pick_top_with_mixing(
    scored: list[tuple[float, set[str], NibsyContent]],
) -> list[tuple[float, set[str], NibsyContent]]:
    """Greedy top-N with a soft same-type penalty.

    Multiplies each remaining candidate's score by TYPE_REPEAT_PENALTY ^ k
    where k is the count of already-chosen items with the same
    content_type. This breaks 6-of-a-kind monocultures while still
    letting strong same-type matches win when nothing else is close.
    """

    remaining = list(scored)
    chosen: list[tuple[float, set[str], NibsyContent]] = []
    type_counts: dict[str, int] = {}

    while remaining and len(chosen) < TOP_N:
        best_idx = -1
        best_adj = float("-inf")
        for i, (raw_score, _shared, target) in enumerate(remaining):
            penalty = TYPE_REPEAT_PENALTY ** type_counts.get(target.content_type, 0)
            adj = raw_score * penalty
            if adj > best_adj:
                best_adj = adj
                best_idx = i
        if best_idx < 0:
            break
        picked = remaining.pop(best_idx)
        chosen.append(picked)
        type_counts[picked[2].content_type] = (
            type_counts.get(picked[2].content_type, 0) + 1
        )

    return chosen


def _build_payload(
    source: NibsyContent,
    chosen: list[tuple[float, set[str], NibsyContent]],
) -> list[dict[str, Any]]:
    """Serialise the chosen targets into the JSONB shape we persist."""

    return [
        {
            "content_id": target.id,
            "url": target.url,
            "title": target.title,
            "type": target.content_type,
            "reason": _reason(source, target, shared),
            "score": round(score, 4),
        }
        for score, shared, target in chosen
    ]


async def generate_recommendations(session: AsyncSession) -> GenerationStats:
    """Regenerate `nibsy_recommendations` from scratch.

    Wipes orphan rows (recommendations whose source content was deleted),
    rescoreseach source's top-N targets, and upserts. Wrapped in a single
    commit so a mid-run failure can't half-update the table.
    """

    start = time.monotonic()
    stats = GenerationStats(generator_version=GENERATOR_VERSION)
    # Use a naive UTC datetime so it round-trips cleanly through both
    # PostgreSQL `TIMESTAMP` (without time zone) and SQLite's TEXT-coerced
    # storage — matches how `ingest.py` writes `updated_at`.
    now = datetime.now(timezone.utc).replace(tzinfo=None)

    # Load every content row once. The dataset is small (~200 rows expected
    # in production) so an in-memory scoring pass is fine and much simpler
    # than streaming. If we ever cross ~10k rows we'd switch to a smarter
    # index-based candidate-set approach.
    contents = (await session.scalars(select(NibsyContent))).all()
    if not contents:
        logger.info("generator: nibsy_content is empty — nothing to do")
        stats.duration_ms = int((time.monotonic() - start) * 1000)
        return stats

    content_urls = {c.url for c in contents}

    # --- Remove orphans first so they don't pollute the count -----------
    existing_recs = (
        await session.scalars(select(NibsyRecommendation.source_url))
    ).all()
    orphan_urls = [u for u in existing_recs if u not in content_urls]
    if orphan_urls:
        await session.execute(
            delete(NibsyRecommendation).where(
                NibsyRecommendation.source_url.in_(orphan_urls)
            )
        )
        stats.removed_orphans = len(orphan_urls)
        logger.info(
            "generator: removed %s orphan recommendation rows", len(orphan_urls)
        )

    # --- Score + persist top-N per source ------------------------------
    rec_count = 0
    for source in contents:
        scored: list[tuple[float, set[str], NibsyContent]] = []
        for target in contents:
            if target.url == source.url:
                continue
            score, shared = _score(source, target, now)
            if score <= 0:
                # Pure no-signal pair — skip to keep the candidate set small.
                continue
            scored.append((score, shared, target))

        if not scored:
            stats.sources_skipped += 1
            continue

        # Pre-sort by raw score desc so the mixing pass starts with a
        # reasonable order. The mixing pass then applies the penalty.
        scored.sort(key=lambda t: t[0], reverse=True)
        # Trim to a sensible candidate window — we never need more than
        # TOP_N * 4 to give the mixing pass room.
        scored = scored[: TOP_N * 4]
        chosen = _pick_top_with_mixing(scored)
        if not chosen:
            stats.sources_skipped += 1
            continue

        payload = _build_payload(source, chosen)
        existing = await session.scalar(
            select(NibsyRecommendation).where(
                NibsyRecommendation.source_url == source.url
            )
        )
        if existing is None:
            session.add(
                NibsyRecommendation(
                    source_url=source.url,
                    recommendations=payload,
                    generated_at=now,
                    generator_version=GENERATOR_VERSION,
                )
            )
        else:
            existing.recommendations = payload
            existing.generated_at = now
            existing.generator_version = GENERATOR_VERSION
        stats.sources_processed += 1
        rec_count += len(payload)

    stats.recommendations_written = rec_count
    await session.commit()
    stats.duration_ms = int((time.monotonic() - start) * 1000)
    logger.info(
        "generator: wrote %s recommendations across %s sources in %sms (skipped=%s, orphans=%s)",
        stats.recommendations_written,
        stats.sources_processed,
        stats.duration_ms,
        stats.sources_skipped,
        stats.removed_orphans,
    )
    return stats
