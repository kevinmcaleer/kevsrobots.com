"""Live, page-aware recommendations.

Nothing here is precomputed. Every request resolves the page the visitor is
on to a catalogue item, compares it with every other item and returns the
closest matches. The catalogue is small (around a thousand rows), so one
in-memory pass per request is cheap, and there is no stored table that can
go stale.

How two pages are compared
--------------------------
Each item becomes a weighted bag of terms built from its tags, title and
description. Rare terms count for more than common ones (IDF), tags count
for more than title words, and title words for more than description
words. Similarity is the cosine between two bags. A small content-type
affinity and a small recency bonus only break ties between equally
related items - they can never make an unrelated item win.

Logged-in readers
-----------------
When a viewing history is supplied, the pages in it are blended into an
interest profile (recent views weigh most) and every candidate is also
compared with that profile. Pages already read are not suggested again.
The profile is rebuilt from the history on each request and never stored.

Pages that are not in the catalogue
-----------------------------------
- A lesson (``/learn/<course>/<lesson>.html``) uses its course.
- Anything else (home page, section indexes) gets a pick from what is
  trending in that section, rotated daily so it is not always the same six.
"""

from __future__ import annotations

import hashlib
import math
import random
import re
from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional, Sequence
from urllib.parse import urlsplit

from sqlalchemy import func, select
from sqlalchemy.ext.asyncio import AsyncSession

from .generator import _affinity, _recency
from .models import NibsyContent, NibsyTrending

ALGORITHM_VERSION = "live-v1"

# Term weights by where the term was found.
W_TAG = 3.0
W_TITLE = 2.0
W_DESCRIPTION = 1.0

# Tie-breakers added to the similarity. Deliberately small: similarity for a
# good match is 0.2-0.8, so these reorder near-equal candidates only.
W_AFFINITY = 0.08
W_RECENCY = 0.05

# How much the reader's history counts next to the page they are on. Under
# 1.0 so the current page still leads; history steers and fills the gaps.
W_PROFILE = 0.6

# A view loses half its influence on the profile every this many days.
PROFILE_HALF_LIFE_DAYS = 30.0

# Below this similarity a candidate is not "related" - we would rather fall
# back to trending than pretend.
MIN_SIMILARITY = 0.04

# Each already-picked item of the same type multiplies the next one's score.
TYPE_REPEAT_PENALTY = 0.8

# How many trending items the daily rotation draws from.
FALLBACK_POOL = 24

_STOPWORDS = frozenset(
    """
    a about after all also an and any are as at be been before build building
    built but by can could do does for from get getting had has have how i if
    in into is it its just kev kevin kevs let like ll make making mcaleer me
    more most my new no not now of on one or our out over own part s so some
    than that the their them then there these they this to up use used using
    very want was we were what when where which who why will with you your
    video videos course courses lesson learn guide intro introduction project
    projects robot robots robotics
    """.split()
)

_WORD_RE = re.compile(r"[a-z0-9]+")


def _stem(token: str) -> str:
    """Fold simple plurals so "servos" and "servo" are the same term."""

    if len(token) > 3 and token.endswith("s") and not token.endswith("ss"):
        return token[:-1]
    return token


def _words(text: Optional[str]) -> list[str]:
    if not text:
        return []
    return [
        _stem(w)
        for w in _WORD_RE.findall(text.lower())
        if len(w) > 1 and not w.isdigit() and w not in _STOPWORDS
    ]


def _tag_term(tag: str) -> str:
    """Canonical form of a tag: "Raspberry Pi", "raspberry_pi" -> "raspberrypi"."""

    return _stem("".join(_WORD_RE.findall(str(tag).lower())))


def normalise_url(url: str) -> str:
    """Reduce any spelling of a site URL to one comparable key.

    ``https://www.kevsrobots.com/blog/x.html?a=1#b``, ``/blog/x.html`` and
    ``/blog/x`` all become ``/blog/x``. External URLs keep their host and
    query (a YouTube video id lives in the query string).
    """

    url = (url or "").strip()
    parts = urlsplit(url)
    host = parts.netloc.lower()
    if host and not host.endswith("kevsrobots.com"):
        return url
    path = parts.path or "/"
    if path.endswith("/index.html"):
        path = path[: -len("index.html")]
    if path.endswith(".html"):
        path = path[: -len(".html")]
    path = path.rstrip("/")
    return path or "/"


def _section(key: str) -> str:
    """First path segment of a normalised key ("/learn/x" -> "learn")."""

    return key.strip("/").split("/", 1)[0]


# Which content types a section's fallback prefers.
_SECTION_TYPES: dict[str, tuple[str, ...]] = {
    "learn": ("course",),
    "blog": ("post",),
    "reviews": ("review",),
    "robots": ("robot", "post"),
    "projects": ("robot", "post"),
    "all_videos": ("video",),
    "videos": ("video",),
}


@dataclass
class Item:
    """One catalogue row plus what the scorer needs."""

    id: int
    type: str
    title: str
    url: str
    key: str
    date: Optional[datetime]
    vector: dict[str, float] = field(default_factory=dict)
    norm: float = 0.0
    # The row's metadata: cover, level, duration, pins, pathways, ...
    meta: dict = field(default_factory=dict)
    description: Optional[str] = None

    @property
    def is_project(self) -> bool:
        """Something to build: a project, a robot, or a post on the projects list."""

        return self.type in ("project", "robot") or bool(self.meta.get("project"))


@dataclass
class Pick:
    item: Item
    score: float
    reason: str
    # True when the reader's history, not the current page, earned the slot.
    personal: bool = False


@dataclass
class View:
    """One row of a user's viewing history."""

    content_id: int
    last_viewed_at: datetime
    view_count: int = 1


class CatalogueIndex:
    """Term vectors for every catalogue item, built once per catalogue change."""

    def __init__(self, rows: Sequence[NibsyContent]) -> None:
        self.items: list[Item] = []
        self.by_id: dict[int, Item] = {}
        self.by_key: dict[str, Item] = {}
        self.course_by_slug: dict[str, Item] = {}
        # Pathway slug -> its courses in order (#632).
        self.pathways: dict[str, list[Item]] = {}
        # Canonical tag term -> the label to show a visitor ("raspberrypi" -> "raspberry pi").
        self.tag_labels: dict[str, str] = {}
        self.idf: dict[str, float] = {}
        self._build(rows)

    def _build(self, rows: Sequence[NibsyContent]) -> None:
        raw: list[tuple[Item, dict[str, float]]] = []

        # Pass 1 - collect every tag so title text can be matched against
        # multi-word tags ("Raspberry Pi" in a title -> the raspberrypi tag).
        row_tags: dict[int, list[str]] = {}
        for row in rows:
            meta = row.content_metadata or {}
            topics = (meta.get("categorisation") or {}).get("topics") or []
            tags = [str(t) for t in [*(row.tags or []), *topics] if t]
            row_tags[row.id] = tags
            for tag in tags:
                term = _tag_term(tag)
                if term:
                    self.tag_labels.setdefault(term, tag.replace("_", " ").replace("-", " "))

        # Pass 2 - a weighted term bag per item (max weight wins per term).
        for row in rows:
            terms: dict[str, float] = {}

            def add(term: str, weight: float) -> None:
                if term and terms.get(term, 0.0) < weight:
                    terms[term] = weight

            for tag in row_tags[row.id]:
                term = _tag_term(tag)
                if not term:
                    continue
                add(term, W_TAG)
                for word in _words(tag):
                    add(word, W_DESCRIPTION)

            for text, weight in ((row.title, W_TITLE), (row.description, W_DESCRIPTION)):
                words = _words(text)
                for word in words:
                    add(word, weight)
                # Adjacent words that spell a known tag ("raspberry pi").
                for first, second in zip(words, words[1:]):
                    joined = _stem(first + second)
                    if joined in self.tag_labels:
                        add(joined, weight)

            item = Item(
                id=row.id,
                type=row.content_type,
                title=row.title,
                url=row.url,
                key=normalise_url(row.url),
                date=row.date_published,
                meta=dict(row.content_metadata or {}),
                description=row.description,
            )
            raw.append((item, terms))

        # IDF over the whole catalogue, then final vectors and norms.
        total = len(raw) or 1
        df: dict[str, int] = {}
        for _item, terms in raw:
            for term in terms:
                df[term] = df.get(term, 0) + 1
        self.idf = {term: math.log(1.0 + total / count) for term, count in df.items()}

        for item, terms in raw:
            item.vector = {t: w * self.idf[t] for t, w in terms.items()}
            item.norm = math.sqrt(sum(v * v for v in item.vector.values()))
            self.items.append(item)
            self.by_id[item.id] = item
            self.by_key.setdefault(item.key, item)
            if item.type == "course":
                parts = item.key.strip("/").split("/")
                if len(parts) >= 2 and parts[0] == "learn":
                    self.course_by_slug.setdefault(parts[1], item)

        placed: dict[str, list[tuple[int, Item]]] = {}
        for item in self.items:
            for entry in item.meta.get("pathways") or []:
                if isinstance(entry, dict) and entry.get("slug"):
                    placed.setdefault(entry["slug"], []).append((int(entry.get("position", 0)), item))
        self.pathways = {
            slug: [item for _pos, item in sorted(entries, key=lambda e: e[0])]
            for slug, entries in placed.items()
        }

    # ---- lookups ---------------------------------------------------------

    def resolve(self, page: str) -> tuple[Optional[Item], str]:
        """Find the catalogue item for a page path.

        Returns ``(item, basis)`` where basis is ``"page"`` for an exact
        match, ``"course"`` when a lesson was mapped to its course, or
        ``"fallback"`` when the page is not in the catalogue.
        """

        key = normalise_url(page)
        item = self.by_key.get(key)
        if item is not None:
            return item, "page"
        parts = key.strip("/").split("/")
        if len(parts) >= 2 and parts[0] == "learn":
            course = self.course_by_slug.get(parts[1])
            if course is not None:
                return course, "course"
        return None, "fallback"

    def similarity(self, a: dict[str, float], a_norm: float, b: Item) -> float:
        if not a_norm or not b.norm:
            return 0.0
        if len(a) > len(b.vector):
            small, large = b.vector, a
        else:
            small, large = a, b.vector
        dot = sum(v * large[t] for t, v in small.items() if t in large)
        return dot / (a_norm * b.norm)

    def reason(self, source_vector: dict[str, float], target: Item) -> str:
        """Short, human reason shown in Nibsy's speech bubble.

        Names the tag that contributes most to the match, whether it was
        an explicit tag or spelled out in a title.
        """

        shared = [
            t for t in target.vector if t in source_vector and t in self.tag_labels
        ]
        if shared:
            best = max(shared, key=lambda t: source_vector[t] * target.vector[t])
            return f"More on {self.tag_labels[best]}"
        if target.type == "video":
            return "Watch next"
        return f"Related {target.type}"


def _mix(candidates: list[Pick], limit: int) -> list[Pick]:
    """Greedy top-N with a soft same-type penalty, keeping one related video.

    The site's job is to send readers to the videos, so when a related
    video exists it always gets one of the slots.
    """

    remaining = sorted(candidates, key=lambda p: p.score, reverse=True)[: limit * 6]
    chosen: list[Pick] = []
    counts: dict[str, int] = {}
    while remaining and len(chosen) < limit:
        best = max(
            remaining,
            key=lambda p: p.score * TYPE_REPEAT_PENALTY ** counts.get(p.item.type, 0),
        )
        remaining.remove(best)
        chosen.append(best)
        counts[best.item.type] = counts.get(best.item.type, 0) + 1

    if chosen and len(chosen) == limit and not counts.get("video"):
        video = next((p for p in remaining if p.item.type == "video"), None)
        if video is not None:
            chosen[-1] = video
    return chosen


def build_profile(
    index: CatalogueIndex, history: Sequence[View], now: datetime
) -> dict[str, float]:
    """Blend the pages a user has read into one interest vector.

    Each viewed page contributes its (unit-length) term vector, weighted
    so that recent views count most (half-life ``PROFILE_HALF_LIFE_DAYS``)
    and repeat views count a little more. Nothing about the result is
    stored; it is rebuilt from the history on every request.
    """

    profile: dict[str, float] = {}
    for view in history:
        item = index.by_id.get(view.content_id)
        if item is None or not item.norm:
            continue
        age_days = max(0.0, (now - view.last_viewed_at).total_seconds() / 86400)
        weight = 0.5 ** (age_days / PROFILE_HALF_LIFE_DAYS) * (
            1.0 + math.log(max(1, view.view_count))
        )
        scale = weight / item.norm
        for term, value in item.vector.items():
            profile[term] = profile.get(term, 0.0) + value * scale
    return profile


def views_from_pages(
    index: CatalogueIndex,
    seen: Sequence[tuple[str, datetime]],
) -> list[View]:
    """Turn the pages a browser says it read into history rows.

    ``seen`` is ``(page path, when)`` pairs. Pages outside the catalogue
    are dropped and a lesson counts as its course, exactly as for stored
    views. Nothing here is written anywhere.
    """

    views: dict[int, View] = {}
    for page, when in seen:
        item, _basis = index.resolve(page)
        if item is None:
            continue
        view = views.get(item.id)
        if view is None:
            views[item.id] = View(item.id, when, 1)
        else:
            view.view_count += 1
            view.last_viewed_at = max(view.last_viewed_at, when)
    return list(views.values())


def merge_views(*histories: Sequence[View]) -> list[View]:
    """Combine histories, one row per page: latest view, summed counts."""

    merged: dict[int, View] = {}
    for history in histories:
        for view in history:
            have = merged.get(view.content_id)
            if have is None:
                merged[view.content_id] = View(
                    view.content_id, view.last_viewed_at, view.view_count
                )
            else:
                have.view_count += view.view_count
                have.last_viewed_at = max(have.last_viewed_at, view.last_viewed_at)
    return list(merged.values())


def _related(
    index: CatalogueIndex,
    source: Optional[Item],
    profile: dict[str, float],
    excluded: set[int],
    now: datetime,
    keep=None,
) -> list[Pick]:
    """Every item related to the current page and/or the reader's profile.

    relevance = similarity to the page + W_PROFILE x similarity to the
    profile. With no page (home, section indexes) the profile stands alone.
    """

    profile_norm = math.sqrt(sum(v * v for v in profile.values()))
    profile_weight = W_PROFILE if source is not None else 1.0
    source_type = source.type if source is not None else "profile"
    picks: list[Pick] = []
    for target in index.items:
        if target.id in excluded or (keep is not None and not keep(target)):
            continue
        page_sim = (
            index.similarity(source.vector, source.norm, target) if source else 0.0
        )
        personal_sim = (
            profile_weight * index.similarity(profile, profile_norm, target)
            if profile_norm
            else 0.0
        )
        if max(page_sim, personal_sim) < MIN_SIMILARITY:
            continue
        score = (
            page_sim
            + personal_sim
            + W_AFFINITY * _affinity(source_type, target.type)
            + W_RECENCY * _recency(target.date, now)
        )
        personal = personal_sim > page_sim
        reason = "" if personal else index.reason(source.vector, target)
        picks.append(Pick(target, score, reason, personal))
    return picks


def _because_you_read(
    index: CatalogueIndex, history: Sequence[View], target: Item
) -> str:
    """Name the viewed page that best explains a personal pick."""

    best: Optional[Item] = None
    best_sim = 0.0
    for view in history:
        viewed = index.by_id.get(view.content_id)
        if viewed is None:
            continue
        sim = index.similarity(viewed.vector, viewed.norm, target)
        if sim > best_sim:
            best, best_sim = viewed, sim
    if best is None:
        return "Picked for you"
    title = best.title if len(best.title) <= 40 else best.title[:39].rstrip() + "…"
    return f"Because you read {title}"


def fallback_picks(
    index: CatalogueIndex,
    page: str,
    limit: int,
    excluded: set[int],
    trending: dict[int, float],
    now: datetime,
    types: Optional[set[str]] = None,
) -> list[Pick]:
    """Trending (or newest) items for the page's section, rotated daily.

    ``types`` limits the pool to those content types (the read-next
    section on blog posts asks for posts only).
    """

    preferred = _SECTION_TYPES.get(_section(normalise_url(page)))

    def popularity(item: Item) -> float:
        if trending:
            return trending.get(item.id, 0.0)
        return _recency(item.date, now)

    pool = [
        i for i in index.items
        if i.id not in excluded and (not types or i.type in types)
    ]
    if preferred:
        in_section = [i for i in pool if i.type in preferred]
        if len(in_section) >= limit:
            pool = in_section
    pool.sort(key=popularity, reverse=True)
    pool = pool[:FALLBACK_POOL]

    # Same page + same day -> same picks; a new day reshuffles.
    seed = hashlib.sha256(f"{normalise_url(page)}|{now:%Y-%m-%d}".encode()).hexdigest()
    rng = random.Random(seed)
    sample = sorted(
        rng.sample(pool, min(limit, len(pool))), key=popularity, reverse=True
    )
    label = "Popular right now" if trending else "New on the site"
    return [Pick(item, popularity(item), label) for item in sample]


def recommend_for_page(
    index: CatalogueIndex,
    page: str,
    limit: int,
    excluded_ids: set[int],
    trending: dict[int, float],
    now: datetime,
    history: Sequence[View] = (),
    types: Optional[set[str]] = None,
) -> tuple[list[Pick], str]:
    """Recommendations for one page. Returns ``(picks, basis)``.

    ``history`` is the reader's viewing history, if any: the logged-in
    reader's stored views, and/or what an anonymous reader's browser says
    they read. Pages they have already read are never suggested back.
    ``types`` limits the picks to those content types.
    """

    source, basis = index.resolve(page)
    excluded = set(excluded_ids)
    if source is not None:
        excluded.add(source.id)
    profile = build_profile(index, history, now)
    if profile:
        excluded |= {v.content_id for v in history}

    picks: list[Pick] = []
    if source is not None or profile:
        keep = (lambda item: item.type in types) if types else None
        picks = _mix(_related(index, source, profile, excluded, now, keep), limit)
        for pick in picks:
            if pick.personal:
                pick.reason = _because_you_read(index, history, pick.item)

    if len(picks) < limit:
        # Not enough genuinely related items - top up from trending.
        excluded |= {p.item.id for p in picks}
        picks += fallback_picks(
            index, page, limit - len(picks), excluded, trending, now, types
        )
    return picks, basis


# ---- learning (#632) -------------------------------------------------------


def resolve_pins(index: CatalogueIndex, urls, excluded: set[int]) -> list[Item]:
    """The catalogue items a page pins, in the order given, skipping unknowns."""

    found: list[Item] = []
    for url in urls or []:
        item, _basis = index.resolve(str(url))
        if item is not None and item.id not in excluded and item not in found:
            found.append(item)
    return found


def next_in_pathways(index: CatalogueIndex, course: Item) -> list[tuple[dict, Item]]:
    """For each pathway the course is in, the course that comes after it.

    Returns ``[(pathway entry, next course)]`` in the order the course's
    pathways were listed. The last course of a pathway has no next.
    """

    out: list[tuple[dict, Item]] = []
    for entry in course.meta.get("pathways") or []:
        courses = index.pathways.get(entry.get("slug"), [])
        if course not in courses:
            continue
        position = courses.index(course)
        if position + 1 < len(courses):
            out.append((entry, courses[position + 1]))
    return out


def related_of_kind(
    index: CatalogueIndex,
    source: Optional[Item],
    keep,
    limit: int,
    excluded: set[int],
    now: datetime,
    pins=(),
    profile: Optional[dict[str, float]] = None,
) -> list[Pick]:
    """Pinned items first, then the items ``keep(item)`` accepts, most related first.

    Used for "Learn the skills" (courses for a post or project) and
    "Build something next" (projects for a course). Unlike the widget, no
    trending top-up: a weak match is left out rather than padded.
    """

    picks: list[Pick] = []
    taken = set(excluded)
    if source is not None:
        taken.add(source.id)
    for item in resolve_pins(index, pins, taken):
        if len(picks) >= limit:
            break
        picks.append(Pick(item, 1.0, "Recommended by Kev"))
        taken.add(item.id)

    if len(picks) < limit and (source is not None or profile):
        candidates = [
            p
            for p in _related(index, source, profile or {}, taken, now)
            if keep(p.item)
        ]
        candidates.sort(key=lambda p: p.score, reverse=True)
        for pick in candidates[: limit - len(picks)]:
            if not pick.reason:
                pick.reason = "Picked for you"
            picks.append(pick)
    return picks


# ---- cached index ----------------------------------------------------------

_cache: tuple[tuple, CatalogueIndex] | None = None


async def get_index(session: AsyncSession) -> CatalogueIndex:
    """Return the catalogue index, rebuilding it only when content changed.

    The signature query is one cheap aggregate; ingest and categorisation
    both bump ``updated_at`` (or the row count), which invalidates it.
    """

    global _cache
    row = (
        await session.execute(
            select(
                func.count(NibsyContent.id),
                func.max(NibsyContent.id),
                func.max(NibsyContent.updated_at),
            )
        )
    ).one()
    signature = (id(session.bind), *row)
    if _cache is not None and _cache[0] == signature:
        return _cache[1]
    rows = (await session.scalars(select(NibsyContent))).all()
    index = CatalogueIndex(rows)
    _cache = (signature, index)
    return index


async def load_trending(session: AsyncSession) -> dict[int, float]:
    rows = (
        await session.execute(
            select(NibsyTrending.content_id, NibsyTrending.trending_score)
        )
    ).all()
    return {r.content_id: r.trending_score for r in rows}
