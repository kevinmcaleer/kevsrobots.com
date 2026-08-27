"""Read the built Jekyll site and turn each page into a Page object.

The old indexer treated a page as one blob of `soup.get_text()`. This module
keeps the document *structure* - headings and the text under them - because
that structure is what the chunker splits on, and it enriches each page with
metadata pulled from Jekyll's _data YAML files so that a lesson knows which
course it belongs to and a post knows its tags.
"""

import hashlib
import os
import re
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path

import yaml
from bs4 import BeautifulSoup

from .config import (
    BASE_URL,
    CONTENT_SELECTORS,
    DATA_ROOT,
    EXCLUDED_PAGE_TYPES,
    HEADING_TAGS,
    SITE_ROOT,
    SKIP_DIRS,
    STRIP_SELECTORS,
)

# The dev server writes localhost URLs into og:image during a local build.
_DEV_HOSTS = (
    "http://0.0.0.0:4000",
    "https://0.0.0.0:4000",
    "http://localhost:4000",
    "https://localhost:4000",
    "http://127.0.0.1:4000",
    "https://127.0.0.1:4000",
)


@dataclass
class Section:
    """A heading and the text beneath it, before any size-based splitting."""

    trail: str  # breadcrumb of enclosing headings, e.g. "Setup > Wiring"
    text: str


@dataclass
class Page:
    """One indexable HTML page, parsed and enriched."""

    url: str  # site-relative, e.g. "learn/docker/03_images.html"
    title: str
    description: str
    page_type: str
    date: str
    author: str
    cover_image: str
    sections: list[Section] = field(default_factory=list)
    tags: list[str] = field(default_factory=list)
    course: str = ""
    course_slug: str = ""
    groups: list[str] = field(default_factory=list)
    rating: float | None = None

    @property
    def full_url(self) -> str:
        return f"{BASE_URL}/{self.url}"

    @property
    def body_text(self) -> str:
        return "\n\n".join(s.text for s in self.sections)

    @property
    def content_hash(self) -> str:
        """Fingerprint of the page's text - changes when the content changes.

        Deliberately hashes the extracted text rather than the raw HTML: a
        Jekyll rebuild rewrites every file's timestamp and can reshuffle
        whitespace, and we do not want that to trigger a re-embed.
        """
        payload = f"{self.title}\n{self.description}\n{self.body_text}"
        return hashlib.sha256(payload.encode("utf-8")).hexdigest()[:16]


# ---------------------------------------------------------------------------
# Metadata from Jekyll's _data directory
# ---------------------------------------------------------------------------


@lru_cache(maxsize=1)
def _site_metadata() -> dict:
    """Metadata Jekyll rendered into the built site.

    Preferred over reading _data directly, because _data does not exist inside
    the search container - the index is built from the site image, which
    carries this file but not the Jekyll sources.
    """
    path = SITE_ROOT / "search-metadata.json"
    if not path.is_file():
        return {}
    try:
        import json

        return json.loads(path.read_text(encoding="utf-8"))
    except Exception as error:
        print(f"Warning: could not read search-metadata.json: {error}")
        return {}


# Map our internal names onto the keys used in search-metadata.json.
_METADATA_KEYS = {
    "courses.yml": "courses",
    "projects.yml": "projects",
    "robots.yml": "robots",
    "reviews.yml": "reviews",
    "glossary.yml": "glossary",
}


def _load_yaml(name: str):
    """Load a data file, preferring the site-embedded JSON over raw _data.

    Both shapes are lists of dicts with the same field names, so callers do not
    care which source answered.
    """
    key = _METADATA_KEYS.get(name)
    if key:
        embedded = _site_metadata().get(key)
        if embedded:
            return embedded

    path = DATA_ROOT / name
    if not path.is_file():
        return None
    try:
        with open(path, "r", encoding="utf-8") as handle:
            return yaml.safe_load(handle)
    except Exception as error:  # a malformed data file must not kill indexing
        print(f"Warning: could not read {name}: {error}")
        return None


@lru_cache(maxsize=1)
def course_lookup() -> dict:
    """Map a course slug to its name, groups and cover.

    A lesson at /learn/docker/03_images.html belongs to the course whose link
    starts /learn/docker/ - so the slug is the path segment after "learn".
    """
    courses = _load_yaml("courses.yml") or []
    lookup = {}
    for course in courses:
        link = course.get("link") or ""
        parts = [p for p in link.split("/") if p]
        if len(parts) >= 2 and parts[0] == "learn":
            slug = parts[1]
            lookup[slug] = {
                "name": course.get("name", ""),
                "groups": course.get("groups") or [],
                "cover": course.get("cover", ""),
                "author": course.get("author", ""),
            }
    return lookup


def _normalise_link(link: str) -> str:
    """Turn a _data `link` value into a site-relative key we can match on.

    The data files are inconsistent: some links carry a leading slash, some
    omit the .html, some point at a directory. Normalising both sides of the
    join is cheaper than trying to fix 200 YAML entries.
    """
    key = (link or "").strip().lstrip("/")
    key = key.split("#")[0].split("?")[0]
    if key.endswith("/"):
        key += "index.html"
    return key


@lru_cache(maxsize=1)
def page_metadata_lookup() -> dict:
    """Map a page URL to extra metadata from the _data YAML files.

    Blog posts carry no tags of their own (only unpublished drafts do), so the
    site's real categorisation comes from these curated data files: projects
    have tags, reviews have ratings, robots and glossary terms have names.
    """
    lookup: dict[str, dict] = {}

    def add(link, **fields):
        key = _normalise_link(link)
        if not key:
            return
        entry = lookup.setdefault(key, {"tags": []})
        for name, value in fields.items():
            if name == "tags":
                for tag in value or []:
                    tag = str(tag).strip().lower()
                    if tag and tag not in entry["tags"]:
                        entry["tags"].append(tag)
            elif value:
                entry[name] = value

    for project in _load_yaml("projects.yml") or []:
        if isinstance(project, dict):
            add(project.get("link"), tags=project.get("tags") or [], collection="project")

    for robot in _load_yaml("robots.yml") or []:
        if isinstance(robot, dict):
            add(robot.get("link"), collection="robot", tags=["robot"])

    for review in _load_yaml("reviews.yml") or []:
        if isinstance(review, dict):
            add(review.get("link"), collection="review", tags=["review"],
                rating=review.get("rating"))

    for term in _load_yaml("glossary.yml") or []:
        if isinstance(term, dict) and term.get("link"):
            add(term.get("link"), tags=["glossary"])

    return lookup


def _tags_from_meta(soup) -> list[str]:
    """Pull tags from whichever meta tag the layout happened to use."""
    tags: list[str] = []
    for attrs in ({"property": "tags"}, {"name": "keywords"}, {"property": "article:tag"}):
        for tag_el in soup.find_all("meta", attrs=attrs):
            content = tag_el.get("content", "")
            tags.extend(t.strip().lower() for t in content.replace(",", " ").split() if t.strip())
    seen, unique = set(), []
    for tag in tags:
        if tag not in seen:
            seen.add(tag)
            unique.append(tag)
    return unique


# ---------------------------------------------------------------------------
# HTML parsing
# ---------------------------------------------------------------------------


def _meta(soup, prop: str, default: str = "") -> str:
    el = soup.find("meta", {"property": prop}) or soup.find("meta", {"name": prop})
    if el and el.get("content"):
        return el["content"].strip()
    return default


def _clean_image_url(url: str) -> str:
    if not url:
        return ""
    for host in _DEV_HOSTS:
        url = url.replace(host, BASE_URL)
    if url.startswith("http://"):
        url = "https://" + url[len("http://") :]
    if not url.startswith(("http://", "https://")):
        url = BASE_URL + ("" if url.startswith("/") else "/") + url
    return url.replace(".com//", ".com/")


def _normalise(text: str) -> str:
    return re.sub(r"[^a-z0-9 ]", "", text.lower()).strip()


def _is_subtitle(heading: str, description: str) -> bool:
    """True if this heading is really the page's subtitle rather than a section.

    Two signals: it matches the page description, or it is long enough that no
    one would write it as a section heading.
    """
    if len(heading) > 90:
        return True
    if not description:
        return False
    h, d = _normalise(heading), _normalise(description)
    return bool(h) and (h == d or (len(h) > 25 and (h in d or d.startswith(h))))


def _extract_sections(soup, description: str = "") -> list[Section]:
    """Walk the main content, splitting on headings and tracking the trail.

    The trail is the enclosing heading hierarchy, so a paragraph deep in a
    lesson still knows it lives under "Docker > Images > Layer caching". That
    context gets prepended to the chunk text before embedding, which is the
    single biggest lever on retrieval quality.
    """
    main = None
    for selector in CONTENT_SELECTORS:
        main = soup.select_one(selector)
        if main is not None:
            break
    if main is None:
        main = soup.find("body") or soup

    sections: list[Section] = []
    trail: list[str] = []
    current: list[str] = []

    def flush():
        text = "\n".join(line for line in current if line.strip())
        text = re.sub(r"\n{3,}", "\n\n", text).strip()
        if text:
            sections.append(Section(trail=" > ".join(t for t in trail if t), text=text))
        current.clear()

    for element in main.find_all(
        [*HEADING_TAGS, "p", "li", "td", "th", "pre", "blockquote", "dd", "dt"]
    ):
        # Skip anything nested inside another element we already captured,
        # or BeautifulSoup hands us the same text several times over.
        if element.find_parent(["pre", "blockquote", "li", "td"]) is not None:
            continue

        if element.name in HEADING_TAGS:
            flush()
            level = int(element.name[1])
            heading = element.get_text(" ", strip=True)
            if not heading:
                continue
            # Several layouts render the page description as an <h2> subtitle.
            # That is prose, not structure - it belongs in the text, not the
            # breadcrumb, where it would swamp every chunk from the page.
            if _is_subtitle(heading, description):
                current.append(heading)
                continue
            trail[: level - 1] = trail[: level - 1]
            del trail[level - 1 :]
            while len(trail) < level - 1:
                trail.append("")
            trail.append(heading)
        else:
            text = element.get_text(" ", strip=True)
            if text:
                current.append(text)

    flush()
    return sections


def parse_page(file_path: Path) -> Page | None:
    """Parse one HTML file into a Page, or None if it should not be indexed."""
    try:
        raw = file_path.read_text(encoding="utf-8", errors="replace")
    except Exception as error:
        print(f"Warning: could not read {file_path}: {error}")
        return None

    try:
        soup = BeautifulSoup(raw, "lxml")
    except Exception as error:
        print(f"Warning: could not parse {file_path}: {error}")
        return None

    page_type = _meta(soup, "page-type", "page") or "page"
    if page_type in EXCLUDED_PAGE_TYPES:
        return None

    # Drop navigation and boilerplate before extracting any text.
    for selector in STRIP_SELECTORS:
        for element in soup.select(selector):
            element.decompose()

    url = os.path.relpath(file_path, SITE_ROOT)
    title = soup.title.get_text(strip=True) if soup.title else ""
    if not title:
        h1 = soup.find("h1")
        title = h1.get_text(" ", strip=True) if h1 else Path(url).stem

    date = _meta(soup, "date")
    if not date:
        from datetime import datetime, timezone

        mtime = file_path.stat().st_mtime
        date = datetime.fromtimestamp(mtime, tz=timezone.utc).strftime("%Y-%m-%dT%H:%M:%S+00:00")

    description = _meta(soup, "description") or _meta(soup, "og:description")

    page = Page(
        url=url,
        title=title,
        description=description,
        page_type=page_type,
        date=date,
        author=_meta(soup, "author", "Kevin McAleer"),
        cover_image=_clean_image_url(_meta(soup, "og:image")),
        sections=_extract_sections(soup, description),
        tags=_tags_from_meta(soup),
    )

    _enrich(page)

    # A page with no extractable prose is not worth an embedding.
    if not page.body_text.strip():
        return None

    return page


def _enrich(page: Page) -> None:
    """Attach course and tag metadata from the _data YAML files."""
    parts = [p for p in page.url.split("/") if p]

    if parts and parts[0] == "learn" and len(parts) >= 2:
        slug = parts[1]
        course = course_lookup().get(slug)
        page.course_slug = slug
        if course:
            page.course = course["name"]
            page.groups = [str(g).lower() for g in course["groups"]]
            if not page.author and course.get("author"):
                page.author = course["author"]

    extra = page_metadata_lookup().get(_normalise_link(page.url))
    if extra:
        for tag in extra.get("tags", []):
            if tag not in page.tags:
                page.tags.append(tag)
        if extra.get("rating") is not None:
            page.rating = extra["rating"]

    # Course groups are useful as tags too - they are how the site categorises.
    for group in page.groups:
        if group not in page.tags:
            page.tags.append(group)


def iter_pages(site_root: Path = SITE_ROOT):
    """Yield every indexable page in the built site, in a stable order."""
    site_root = Path(site_root)
    if not site_root.is_dir():
        raise FileNotFoundError(
            f"Built site not found at {site_root}. Build the Jekyll site first."
        )

    for path in sorted(site_root.rglob("*.html")):
        relative_parts = set(path.relative_to(site_root).parts[:-1])
        if relative_parts & SKIP_DIRS:
            continue
        page = parse_page(path)
        if page is not None:
            yield page
