"""Turn the built site into chunks - the shared front half of both indexers."""

import json
from collections import Counter
from pathlib import Path

from .chunker import chunk_page
from .config import SITE_ROOT
from .content import iter_pages


def _preference(url: str) -> tuple:
    """Sort key for choosing which of two identical pages to keep.

    Courses publish their intro twice - once as `01_intro.html` and again as
    the folder's `index.html`. Both are real URLs, but indexing both means one
    page occupies two result slots. Prefer the shallower, shorter URL: a reader
    sent to /learn/scrawly_wally/ is better served than one sent to
    /learn/scrawly_wally/01_intro.html.
    """
    return (url.count("/"), len(url), url)


def iter_chunks(site_root: Path = SITE_ROOT, verbose: bool = False):
    """Yield (page, chunks) for every indexable page, skipping duplicates.

    Deduplication is by content hash, so it catches the intro/index pairs
    above and any other page published at two URLs, without needing a list of
    special cases.
    """
    by_hash: dict[str, object] = {}
    for page in iter_pages(site_root):
        existing = by_hash.get(page.content_hash)
        if existing is None or _preference(page.url) < _preference(existing.url):
            by_hash[page.content_hash] = page

    for page in sorted(by_hash.values(), key=lambda p: p.url):
        chunks = chunk_page(page)
        if not chunks:
            continue
        if verbose:
            print(f"  {page.url} ({len(chunks)} chunks)")
        yield page, chunks


def summarise(site_root: Path = SITE_ROOT) -> dict:
    """Walk the whole site and report what the pipeline produced."""
    pages = 0
    chunk_count = 0
    sizes: list[int] = []
    types: Counter = Counter()
    tagged = 0
    with_course = 0

    for page, chunks in iter_chunks(site_root):
        pages += 1
        chunk_count += len(chunks)
        sizes.extend(len(c.text) for c in chunks)
        types[page.page_type] += 1
        if page.tags and page.tags != ["untagged"]:
            tagged += 1
        if page.course:
            with_course += 1

    sizes.sort()
    return {
        "pages": pages,
        "chunks": chunk_count,
        "median_chunk_chars": sizes[len(sizes) // 2] if sizes else 0,
        "largest_chunk_chars": sizes[-1] if sizes else 0,
        "pages_with_tags": tagged,
        "pages_with_course": with_course,
        "page_types": dict(types.most_common()),
    }


def dump(output: Path, site_root: Path = SITE_ROOT, limit: int | None = None) -> int:
    """Write chunks to JSON so you can eyeball what is actually being indexed."""
    records = []
    for _, chunks in iter_chunks(site_root):
        for chunk in chunks:
            records.append({"id": chunk.id, "text": chunk.text, "metadata": chunk.metadata})
            if limit and len(records) >= limit:
                break
        if limit and len(records) >= limit:
            break

    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(records, indent=2), encoding="utf-8")
    return len(records)
