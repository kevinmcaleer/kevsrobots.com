"""YAML data ingestion.

Reads the canonical YAML files under `web/_data/` (or a fixtures directory in
tests) and upserts them into `nibsy_content`. Each YAML file has its own
quirks — see the per-file handlers below — but the output schema is uniform.

Upsert key is `nibsy_content.url`. Rows are compared field-by-field; identical
rows are counted as "unchanged" so we can prove idempotency.
"""

from __future__ import annotations

import logging
import re
from datetime import date, datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Optional

import httpx
import yaml
from sqlalchemy import delete, select
from sqlalchemy.ext.asyncio import AsyncSession

from .models import NibsyContent
from .schemas import IngestStats

logger = logging.getLogger(__name__)


# --- Normalisation helpers -------------------------------------------------


def _slugify(text: str) -> str:
    """Turn arbitrary text into a stable URL slug."""

    text = text.strip().lower()
    text = re.sub(r"[^a-z0-9]+", "-", text)
    return text.strip("-")


def _parse_date(value: Any) -> Optional[datetime]:
    """Best-effort coercion of a YAML date/string into a datetime."""

    if value is None:
        return None
    if isinstance(value, datetime):
        return value
    if isinstance(value, date):
        return datetime(value.year, value.month, value.day)
    if isinstance(value, str):
        for fmt in ("%Y-%m-%d", "%Y/%m/%d", "%Y-%m-%dT%H:%M:%S"):
            try:
                return datetime.strptime(value, fmt)
            except ValueError:
                continue
    return None


def _load_yaml(path: Path) -> Any:
    """Read a YAML file, returning `None` if missing."""

    if not path.exists():
        logger.info("ingest: %s not found — skipping", path.name)
        return None
    with path.open("r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


# --- Per-file row builders -------------------------------------------------


def _rows_from_courses(data: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for entry in data or []:
        link = entry.get("link")
        name = entry.get("name")
        if not link or not name:
            continue
        rows.append(
            {
                "content_type": "course",
                "title": name,
                "url": link,
                "description": entry.get("description"),
                "tags": entry.get("groups") or [],
                "date_published": _parse_date(entry.get("date_published")),
                "content_metadata": {
                    "author": entry.get("author"),
                    "cover": entry.get("cover"),
                    "duration": entry.get("duration"),
                },
            }
        )
    return rows


def _post_url_to_blog_path(url: str) -> str:
    """Convert a posts.yaml URL like /2026-02-22-slug.md to /blog/slug.html."""

    name = url.strip("/")
    if name.endswith(".md"):
        name = name[:-3]
    # Strip the YYYY-MM-DD- date prefix.
    parts = name.split("-", 3)
    if len(parts) >= 4 and len(parts[0]) == 4:
        slug = parts[3]
    else:
        slug = name
    return f"/blog/{slug}.html"


def _rows_from_posts(data: Any) -> list[dict[str, Any]]:
    # posts.yaml is dict-wrapped: {posts: [...]} — issue body missed this.
    if isinstance(data, dict):
        items = data.get("posts") or []
    else:
        items = data or []
    rows: list[dict[str, Any]] = []
    for entry in items:
        url = entry.get("url")
        title = entry.get("title")
        if not url or not title:
            continue
        rows.append(
            {
                "content_type": "post",
                "title": title,
                "url": _post_url_to_blog_path(url),
                "description": None,
                "tags": [],
                "date_published": _parse_date(entry.get("date")),
                "content_metadata": {},
            }
        )
    return rows


def _rows_from_youtube(data: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for entry in data or []:
        video_id = entry.get("video_id")
        title = entry.get("title")
        if not video_id or not title:
            continue
        # youtube.yml has no native URL — synthesise one for the upsert key.
        url = f"https://youtube.com/watch?v={video_id}"
        rows.append(
            {
                "content_type": "video",
                "title": title,
                "url": url,
                "description": None,
                "tags": [t for t in (entry.get("type"), entry.get("strategy")) if t],
                "date_published": _parse_date(entry.get("published")),
                "content_metadata": {
                    "video_id": video_id,
                    "type": entry.get("type"),
                    "strategy": entry.get("strategy"),
                },
            }
        )
    return rows


def _rows_from_robots(data: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for entry in data or []:
        link = entry.get("link")
        name = entry.get("name")
        if not link or not name:
            continue
        rows.append(
            {
                "content_type": "robot",
                "title": name,
                "url": link,
                "description": entry.get("description"),
                "tags": [],
                "date_published": _parse_date(entry.get("date")),
                "content_metadata": {
                    "cover": entry.get("cover"),
                    "hero": entry.get("hero"),
                },
            }
        )
    return rows


def _rows_from_reviews(data: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for entry in data or []:
        title = entry.get("title")
        excerpt = entry.get("excerpt")
        link = entry.get("link")
        # Skip placeholder rows per brief: `excerpt: null` or `title: title`.
        if not title or excerpt is None or title == "title" or not link:
            continue
        rows.append(
            {
                "content_type": "review",
                "title": title,
                "url": link,
                "description": excerpt,
                "tags": [],
                "date_published": _parse_date(entry.get("date")),
                "content_metadata": {
                    "author": entry.get("author"),
                    "cover": entry.get("cover"),
                    "rating": entry.get("rating"),
                    "transparency": entry.get("transparency"),
                },
            }
        )
    return rows


def _rows_from_glossary(data: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for entry in data or []:
        term = entry.get("term")
        if not term:
            continue
        # `link` is external — synthesise a site-relative URL from the term
        # slug so the upsert key is stable across edits.
        url = f"/glossary/{_slugify(term)}"
        rows.append(
            {
                "content_type": "glossary",
                "title": term,
                "url": url,
                "description": entry.get("definition") or entry.get("full"),
                "tags": [],
                "date_published": None,
                "content_metadata": {
                    "full": entry.get("full"),
                    "external_link": entry.get("link"),
                    "course": entry.get("course"),
                    "article": entry.get("article"),
                },
            }
        )
    return rows


# --- Upsert --------------------------------------------------------------


_TRACKED_FIELDS = (
    "content_type",
    "title",
    "description",
    "tags",
    "date_published",
    "content_metadata",
)


async def _upsert_row(
    session: AsyncSession, row: dict[str, Any], stats: IngestStats
) -> Optional[NibsyContent]:
    """Insert/update a single row keyed by `url`. Returns the persisted row."""

    existing = await session.scalar(
        select(NibsyContent).where(NibsyContent.url == row["url"])
    )
    if existing is None:
        obj = NibsyContent(**row)
        session.add(obj)
        stats.added += 1
        return obj

    changed = False
    for field in _TRACKED_FIELDS:
        new_value = row.get(field)
        if field == "content_metadata":
            # Merge rather than replace so per-file augmentations (e.g.
            # popular_videos.yaml stats merged into youtube rows) survive
            # subsequent re-ingests of the primary file.
            existing_meta = dict(getattr(existing, field) or {})
            incoming_meta = dict(new_value or {})
            merged = {**existing_meta, **incoming_meta}
            if merged != existing_meta:
                setattr(existing, field, merged)
                changed = True
            continue
        if getattr(existing, field) != new_value:
            setattr(existing, field, new_value)
            changed = True
    if changed:
        existing.updated_at = datetime.now(timezone.utc).replace(tzinfo=None)
        stats.updated += 1
    else:
        stats.unchanged += 1
    return existing


async def _merge_popular_videos(
    data: Iterable[dict[str, Any]], session: AsyncSession, stats: IngestStats
) -> None:
    """Merge popular-video stats into existing video rows.

    Per brief: do NOT create new content rows. Look up by synthesised
    YouTube URL and merge `{views, popular}` into `metadata`.
    """

    for entry in data or []:
        video_id = entry.get("Content")
        if not video_id:
            continue
        url = f"https://youtube.com/watch?v={video_id}"
        existing = await session.scalar(
            select(NibsyContent).where(NibsyContent.url == url)
        )
        if existing is None:
            # No matching video — skip rather than invent a content row.
            stats.skipped += 1
            continue
        before = dict(existing.content_metadata or {})
        meta = dict(before)
        meta["views"] = entry.get("Views")
        meta["views_str"] = entry.get("views_str")
        meta["popular"] = bool(entry.get("popular"))
        if meta == before:
            stats.unchanged += 1
            continue
        existing.content_metadata = meta
        existing.updated_at = datetime.now(timezone.utc).replace(tzinfo=None)
        stats.updated += 1


# --- Public entrypoint ---------------------------------------------------


_REMOTE_FILES = [
    "courses.yml",
    "posts.yaml",
    "youtube.yml",
    "robots.yml",
    "reviews.yml",
    "glossary.yml",
    "popular_videos.yaml",
]


_FILE_HANDLERS: dict[str, Any] = {
    "courses.yml": _rows_from_courses,
    "posts.yaml": _rows_from_posts,
    "youtube.yml": _rows_from_youtube,
    "robots.yml": _rows_from_robots,
    "reviews.yml": _rows_from_reviews,
    "glossary.yml": _rows_from_glossary,
}


async def ingest_from_data_dir(
    path: Path, session: AsyncSession
) -> IngestStats:
    """Read all known YAML files in `path`, upsert into `nibsy_content`.

    `popular_videos.yaml` is handled separately: it merges into existing
    video rows rather than creating its own content rows.
    """

    stats = IngestStats()
    path = Path(path)

    if not path.exists() or not path.is_dir():
        stats.errors += 1
        stats.error_details.append(f"data dir not found: {path}")
        return stats

    # Pass 1 — the row-producing files.
    for filename, handler in _FILE_HANDLERS.items():
        file_path = path / filename
        try:
            data = _load_yaml(file_path)
        except yaml.YAMLError as exc:  # malformed YAML
            stats.errors += 1
            stats.error_details.append(f"{filename}: {exc}")
            continue
        if data is None:
            continue
        stats.files_processed.append(filename)
        rows = handler(data)
        for row in rows:
            try:
                await _upsert_row(session, row, stats)
            except Exception as exc:  # noqa: BLE001 — log + continue
                stats.errors += 1
                stats.error_details.append(f"{filename} {row.get('url')}: {exc}")

    # Flush so popular_videos merging sees freshly-inserted youtube rows.
    await session.flush()

    # Pass 2 — popular_videos.yaml merges metadata only.
    pop_path = path / "popular_videos.yaml"
    try:
        pop_data = _load_yaml(pop_path)
    except yaml.YAMLError as exc:
        stats.errors += 1
        stats.error_details.append(f"popular_videos.yaml: {exc}")
        pop_data = None
    if pop_data is not None:
        stats.files_processed.append("popular_videos.yaml")
        await _merge_popular_videos(pop_data, session, stats)

    # Clean up stale .md post URLs left over from before the URL fix.
    stale = await session.execute(
        delete(NibsyContent).where(
            NibsyContent.content_type == "post",
            NibsyContent.url.like("%.md"),
        )
    )
    if stale.rowcount:
        logger.info("ingest: removed %s stale .md post rows", stale.rowcount)

    await session.commit()
    return stats


async def ingest_from_remote(
    base_url: str, session: AsyncSession
) -> IngestStats:
    """Fetch YAML files over HTTP from the live site and ingest them (#69)."""

    stats = IngestStats()
    base_url = base_url.rstrip("/")

    async with httpx.AsyncClient(timeout=30) as client:
        # Pass 1 — row-producing files.
        for filename in _REMOTE_FILES:
            if filename == "popular_videos.yaml":
                continue
            handler = _FILE_HANDLERS.get(filename)
            if handler is None:
                continue
            url = f"{base_url}/assets/data/{filename}"
            try:
                resp = await client.get(url)
                resp.raise_for_status()
            except httpx.HTTPError as exc:
                stats.errors += 1
                stats.error_details.append(f"{filename}: {exc}")
                continue
            try:
                data = yaml.safe_load(resp.text)
            except yaml.YAMLError as exc:
                stats.errors += 1
                stats.error_details.append(f"{filename}: {exc}")
                continue
            if data is None:
                continue
            stats.files_processed.append(filename)
            rows = handler(data)
            for row in rows:
                try:
                    await _upsert_row(session, row, stats)
                except Exception as exc:  # noqa: BLE001
                    stats.errors += 1
                    stats.error_details.append(f"{filename} {row.get('url')}: {exc}")

        await session.flush()

        # Pass 2 — popular_videos.yaml merges metadata only.
        pop_url = f"{base_url}/assets/data/popular_videos.yaml"
        try:
            resp = await client.get(pop_url)
            resp.raise_for_status()
            pop_data = yaml.safe_load(resp.text)
        except (httpx.HTTPError, yaml.YAMLError) as exc:
            stats.errors += 1
            stats.error_details.append(f"popular_videos.yaml: {exc}")
            pop_data = None
        if pop_data is not None:
            stats.files_processed.append("popular_videos.yaml")
            await _merge_popular_videos(pop_data, session, stats)

    await session.commit()
    return stats
