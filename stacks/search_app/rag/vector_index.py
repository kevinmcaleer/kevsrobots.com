"""Build and maintain the ChromaDB index of the site.

Indexing is incremental: a page is only re-embedded when its content hash
changes. That keeps a rebuild after a typo fix down to seconds instead of
re-embedding ten thousand chunks.
"""

from dataclasses import dataclass
from pathlib import Path

import chromadb
from chromadb.config import Settings

from .config import CHROMA_PATH, COLLECTION_NAME, SITE_ROOT
from .pipeline import iter_chunks

BATCH_SIZE = 200


def get_collection(db_path: Path = CHROMA_PATH, name: str = COLLECTION_NAME):
    """Open (or create) the persistent collection.

    Cosine space is set at creation time and cannot be changed later without a
    rebuild - Chroma silently ignores it on an existing collection.
    """
    db_path = Path(db_path).expanduser()
    db_path.mkdir(parents=True, exist_ok=True)

    client = chromadb.PersistentClient(
        path=str(db_path),
        settings=Settings(anonymized_telemetry=False),
    )
    return client.get_or_create_collection(
        name=name,
        configuration={"hnsw": {"space": "cosine"}},
    )


@dataclass
class IndexStats:
    """What the last indexing run actually did."""

    added: int = 0
    updated: int = 0
    unchanged: int = 0
    removed: int = 0
    chunks: int = 0

    def summary(self) -> str:
        return (
            f"{self.added} new, {self.updated} changed, "
            f"{self.unchanged} unchanged, {self.removed} removed "
            f"({self.chunks} chunks written)"
        )


def write_chunks(collection, chunks) -> int:
    """Upsert chunks in batches.

    Always upsert, never add: `add()` silently keeps the *old* document when it
    meets an id it already has, so an edited page would quietly keep its stale
    text in the index with no error to tell you.
    """
    for start in range(0, len(chunks), BATCH_SIZE):
        batch = chunks[start : start + BATCH_SIZE]
        collection.upsert(
            ids=[c.id for c in batch],
            documents=[c.text for c in batch],
            metadatas=[c.metadata for c in batch],
        )
    return len(chunks)


def indexed_page_hashes(collection) -> dict[str, str]:
    """Map every indexed page URL to the content hash it was indexed at."""
    stored = collection.get(include=["metadatas"])
    hashes: dict[str, str] = {}
    for metadata in stored.get("metadatas") or []:
        if metadata and metadata.get("url"):
            hashes[metadata["url"]] = metadata.get("content_hash", "")
    return hashes


def build_index(
    site_root: Path = SITE_ROOT,
    db_path: Path = CHROMA_PATH,
    full: bool = False,
    verbose: bool = True,
) -> IndexStats:
    """Index the site, skipping pages whose content has not changed."""
    collection = get_collection(db_path)
    stats = IndexStats()

    if full:
        existing: dict[str, str] = {}
        ids = collection.get()["ids"]
        if ids:
            for start in range(0, len(ids), 5000):
                collection.delete(ids=ids[start : start + 5000])
    else:
        existing = indexed_page_hashes(collection)

    seen: set[str] = set()

    for page, chunks in iter_chunks(site_root):
        seen.add(page.url)
        previous = existing.get(page.url)

        if previous == page.content_hash:
            stats.unchanged += 1
            continue

        if previous is not None:
            # Delete before writing: a page that shrinks from five chunks to
            # two would otherwise leave ::2, ::3, ::4 orphaned in the index,
            # still searchable, still quoting text that no longer exists.
            collection.delete(where={"url": page.url})
            stats.updated += 1
        else:
            stats.added += 1

        stats.chunks += write_chunks(collection, chunks)
        if verbose:
            print(f"  indexed {page.url} ({len(chunks)} chunks)")

    for stale in set(existing) - seen:
        collection.delete(where={"url": stale})
        stats.removed += 1
        if verbose:
            print(f"  removed {stale}")

    return stats


def collection_stats(db_path: Path = CHROMA_PATH) -> dict:
    """Health check on the vector index."""
    try:
        collection = get_collection(db_path)
    except Exception as error:
        return {"chunks": 0, "pages": 0, "error": str(error), "path": str(db_path)}

    stored = collection.get(include=["metadatas"])
    metadatas = [m for m in (stored.get("metadatas") or []) if m]
    pages = {m.get("url") for m in metadatas}
    types: dict[str, int] = {}
    for m in metadatas:
        types[m.get("page_type", "unknown")] = types.get(m.get("page_type", "unknown"), 0) + 1

    return {
        "chunks": collection.count(),
        "pages": len(pages),
        "page_types": dict(sorted(types.items(), key=lambda kv: -kv[1])),
        "path": str(Path(db_path).expanduser()),
    }
