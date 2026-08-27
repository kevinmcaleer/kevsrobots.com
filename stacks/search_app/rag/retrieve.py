"""Search the index and pick the best chunks to return.

Two arms, merged:

* **Semantic** - ChromaDB vector search, good at meaning ("robot that draws
  on walls" finds the plotter project without sharing a single keyword).
* **Exact** - a regex pass over chunk text for identifier-shaped terms, because
  embeddings are genuinely bad at part numbers. Search for `PCA9685` and a
  pure vector search returns "things about servo drivers" with the one page
  that actually says PCA9685 nowhere near the top.
"""

import re
from dataclasses import dataclass, field
from pathlib import Path

from .config import (
    CHROMA_PATH,
    MAX_CHUNKS_PER_PAGE,
    OVERSAMPLE,
    STRAGGLER_RATIO,
    TOP_K,
)
from .vector_index import get_collection


@dataclass
class Hit:
    """One retrieved chunk, with its score and where it came from."""

    text: str
    url: str
    title: str
    section: str
    distance: float
    metadata: dict = field(default_factory=dict)
    exact: bool = False

    @property
    def score(self) -> float:
        """Cosine distance as a 0-1 similarity, for humans.

        Display only - never rank or threshold on this. Chroma already returned
        results in distance order.
        """
        return max(0.0, min(1.0, 1.0 - self.distance))

    @property
    def label(self) -> str:
        return self.section or self.title

    def as_result(self) -> dict:
        """Shape a hit like the existing FTS API's result rows."""
        meta = self.metadata
        return {
            "url": self.url,
            "cover_image": meta.get("cover_image", ""),
            "page_title": self.title,
            "description": meta.get("description", ""),
            "date": meta.get("date", ""),
            "author": meta.get("author", "Kevin McAleer"),
            "page_type": meta.get("page_type", "page"),
            "snippet": _snippet(self.text),
            "section": self.section,
            "course": meta.get("course", ""),
            "tags": meta.get("tags", []),
            "score": round(self.score, 4),
            "exact_match": self.exact,
        }


def _snippet(text: str, length: int = 240) -> str:
    """The chunk text minus its breadcrumb line, trimmed for display."""
    body = text.split("\n\n", 1)[-1] if "\n\n" in text else text
    body = " ".join(body.split())
    return body[:length] + ("..." if len(body) > length else "")


def _hit_from(document: str, metadata: dict, distance: float, exact: bool = False) -> Hit:
    metadata = metadata or {}
    return Hit(
        text=document or "",
        url=metadata.get("url", ""),
        title=metadata.get("title", metadata.get("url", "")),
        section=metadata.get("section", ""),
        distance=distance,
        metadata=metadata,
        exact=exact,
    )


# ---------------------------------------------------------------------------
# Filters
# ---------------------------------------------------------------------------


def build_filter(
    page_types: list[str] | None = None,
    tag: str | None = None,
    course: str | None = None,
    since: float | None = None,
) -> dict | None:
    """Turn query options into a Chroma `where` clause.

    Chroma wants `where=None` for "no filter" - an empty dict is an error. And
    `$and` needs at least two operands, so a single clause is returned bare.
    """
    clauses: list[dict] = []

    if page_types:
        types = [t.strip() for t in page_types if t and t.strip()]
        if len(types) == 1:
            clauses.append({"page_type": {"$eq": types[0]}})
        elif types:
            clauses.append({"page_type": {"$in": types}})

    if tag:
        # tags is a list field, so $contains - $in would compare the whole list
        # against the string and silently match nothing.
        clauses.append({"tags": {"$contains": tag.strip().lower()}})

    if course:
        clauses.append({"course_slug": {"$eq": course.strip().lower()}})

    if since:
        clauses.append({"date": {"$gte": str(since)}})

    if not clauses:
        return None
    return clauses[0] if len(clauses) == 1 else {"$and": clauses}


# ---------------------------------------------------------------------------
# Ranking helpers
# ---------------------------------------------------------------------------


def diversify(hits: list[Hit], limit: int, per_page: int = MAX_CHUNKS_PER_PAGE) -> list[Hit]:
    """Stop one long page filling every slot.

    A detailed course page has a dozen chunks that all match a question about
    it. Without a cap they crowd out the blog post that actually answers it.
    """
    kept: list[Hit] = []
    counts: dict[str, int] = {}
    for hit in hits:
        if counts.get(hit.url, 0) >= per_page:
            continue
        counts[hit.url] = counts.get(hit.url, 0) + 1
        kept.append(hit)
        if len(kept) >= limit:
            break
    return kept


def drop_stragglers(hits: list[Hit], ratio: float | None = STRAGGLER_RATIO) -> list[Hit]:
    """Keep hits at least `ratio` as good as the best one.

    Relative, not absolute: a top hit of 52% is a genuinely good match on this
    corpus, and a fixed "reject under 60%" rule would throw away every correct
    answer.
    """
    if not ratio or not hits:
        return hits
    floor = hits[0].score * ratio
    return [h for h in hits if h.score >= floor] or hits[:1]


def looks_like_identifier(term: str) -> bool:
    """Part numbers, board names and error codes - digits mixed with letters."""
    return (
        len(term) >= 4
        and any(c.isdigit() for c in term)
        and any(c.isalpha() for c in term)
    )


# ---------------------------------------------------------------------------
# The two arms
# ---------------------------------------------------------------------------


def semantic_search(
    query: str,
    top_k: int = TOP_K,
    where: dict | None = None,
    db_path: Path = CHROMA_PATH,
    oversample: int = OVERSAMPLE,
) -> list[Hit]:
    """Vector search, oversampled so diversify has material to work with."""
    collection = get_collection(db_path)
    count = collection.count()
    if count == 0:
        return []

    n_results = max(1, min(top_k * oversample, count))
    result = collection.query(
        query_texts=[query],
        n_results=n_results,
        where=where,
        include=["documents", "metadatas", "distances"],
    )

    documents = (result.get("documents") or [[]])[0]
    metadatas = (result.get("metadatas") or [[]])[0]
    distances = (result.get("distances") or [[]])[0]

    return [
        _hit_from(document, metadata, distance)
        for document, metadata, distance in zip(documents, metadatas, distances)
    ]


def exact_search(
    query: str,
    limit: int = 5,
    where: dict | None = None,
    db_path: Path = CHROMA_PATH,
) -> list[Hit]:
    """Find chunks that literally contain an identifier from the query."""
    terms = [t.strip(".,?!()[]\"'") for t in query.split()]
    identifiers = [t for t in terms if looks_like_identifier(t)]
    if not identifiers:
        return []

    collection = get_collection(db_path)
    if collection.count() == 0:
        return []

    found: list[Hit] = []
    for identifier in identifiers:
        try:
            result = collection.get(
                # $contains on document text is case sensitive, so use $regex
                # with an inline flag. re.escape because a part number can
                # contain "." or "-", which mean something else in a regex.
                where_document={"$regex": f"(?i){re.escape(identifier)}"},
                where=where,
                include=["documents", "metadatas"],
                limit=limit,
            )
        except Exception as error:
            print(f"Exact search failed for {identifier!r}: {error}")
            continue

        for document, metadata in zip(
            result.get("documents") or [], result.get("metadatas") or []
        ):
            # distance 0.0 is a fiction - there was no vector comparison - but
            # it encodes the right priority when merged.
            found.append(_hit_from(document, metadata, 0.0, exact=True))

    return found


def merge_hits(exact: list[Hit], semantic: list[Hit], limit: int) -> list[Hit]:
    """Exact matches first, then semantic ones, no duplicates."""
    merged: list[Hit] = []
    seen: set[str] = set()
    for hit in list(exact) + list(semantic):
        key = f"{hit.url}::{hit.metadata.get('chunk_index', hit.text[:60])}"
        if key in seen:
            continue
        seen.add(key)
        merged.append(hit)
        if len(merged) >= limit:
            break
    return merged


# ---------------------------------------------------------------------------
# The public entry point
# ---------------------------------------------------------------------------


def search(
    query: str,
    top_k: int = TOP_K,
    page_types: list[str] | None = None,
    tag: str | None = None,
    course: str | None = None,
    hybrid: bool = True,
    db_path: Path = CHROMA_PATH,
    apply_straggler_filter: bool = True,
) -> list[Hit]:
    """Hybrid search over the vector index.

    Returns page-diverse hits, exact identifier matches first.
    """
    if not query or not query.strip():
        return []

    where = build_filter(page_types=page_types, tag=tag, course=course)

    semantic = semantic_search(query, top_k=top_k, where=where, db_path=db_path)
    if apply_straggler_filter:
        semantic = drop_stragglers(semantic)

    exact = exact_search(query, where=where, db_path=db_path) if hybrid else []

    merged = merge_hits(exact, semantic, limit=top_k * OVERSAMPLE)
    return diversify(merged, top_k)


def get_page_chunks(url: str, db_path: Path = CHROMA_PATH) -> list[dict]:
    """Every indexed chunk for one page, in document order.

    Used by the MCP `get_page` tool so an LLM can read a whole page after a
    search surfaces one paragraph of it.
    """
    collection = get_collection(db_path)
    result = collection.get(
        where={"url": url}, include=["documents", "metadatas"]
    )
    rows = list(
        zip(result.get("documents") or [], result.get("metadatas") or [])
    )
    rows.sort(key=lambda row: (row[1] or {}).get("chunk_index", 0))
    return [{"text": document, "metadata": metadata} for document, metadata in rows]
