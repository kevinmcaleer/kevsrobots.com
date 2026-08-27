#!/usr/bin/env python3
"""Build the search indexes for kevsrobots.com.

Builds both the SQLite FTS index (keyword arm) and the ChromaDB vector index
(semantic arm) from the built Jekyll site. Run at Docker build time so each
node ships with its own index and needs no shared database.

    python3 build_index.py              # incremental vector, rebuild FTS
    python3 build_index.py --full       # wipe and rebuild everything
    python3 build_index.py --stats      # report on the existing indexes
    python3 build_index.py --dump out.json --limit 50
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from rag import config
from rag.pipeline import dump as dump_chunks
from rag.pipeline import summarise


def cmd_stats() -> int:
    from rag.vector_index import collection_stats

    print(f"Site root:   {config.SITE_ROOT}")
    print(f"Chroma path: {config.CHROMA_PATH}")
    print()
    info = collection_stats()
    if info.get("error"):
        print(f"Vector index: ERROR - {info['error']}")
        return 1
    print(f"Vector index: {info['chunks']} chunks across {info['pages']} pages")
    for page_type, count in list(info.get("page_types", {}).items())[:12]:
        print(f"    {page_type:>16}: {count}")
    return 0 if info["chunks"] else 1


def cmd_summary() -> int:
    start = time.time()
    info = summarise()
    print(f"Pipeline summary (took {time.time() - start:.1f}s)")
    for key, value in info.items():
        print(f"  {key}: {value}")
    return 0


def cmd_build(full: bool, verbose: bool, skip_fts: bool, skip_vector: bool) -> int:
    if not config.SITE_ROOT.is_dir():
        print(f"ERROR: built site not found at {config.SITE_ROOT}", file=sys.stderr)
        print("Build the Jekyll site first (cd stacks && docker-compose up -d)", file=sys.stderr)
        return 1

    if not skip_fts:
        print("=== Building SQLite FTS index ===")
        start = time.time()
        import index as fts_indexer

        fts_indexer.main()
        print(f"FTS index built in {time.time() - start:.1f}s\n")

    if not skip_vector:
        print("=== Building ChromaDB vector index ===")
        start = time.time()
        from rag.vector_index import build_index, collection_stats

        stats = build_index(full=full, verbose=verbose)
        elapsed = time.time() - start
        print(f"Done in {elapsed:.1f}s: {stats.summary()}")

        info = collection_stats()
        print(f"Index now holds {info['chunks']} chunks across {info['pages']} pages")
        if info["chunks"] == 0:
            print("ERROR: vector index is empty after build", file=sys.stderr)
            return 1

    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Build kevsrobots search indexes.")
    parser.add_argument("--full", action="store_true",
                        help="Wipe and rebuild the vector index from scratch")
    parser.add_argument("--quiet", dest="verbose", action="store_false",
                        help="Only print the summary line")
    parser.add_argument("--stats", action="store_true", help="Report on existing indexes and exit")
    parser.add_argument("--summary", action="store_true",
                        help="Report what the chunk pipeline produces, without indexing")
    parser.add_argument("--dump", metavar="PATH", help="Write chunks to JSON for inspection")
    parser.add_argument("--limit", type=int, default=None, help="Cap --dump output")
    parser.add_argument("--skip-fts", action="store_true", help="Do not rebuild the FTS index")
    parser.add_argument("--skip-vector", action="store_true", help="Do not rebuild the vector index")
    args = parser.parse_args(argv)

    if args.stats:
        return cmd_stats()
    if args.summary:
        return cmd_summary()
    if args.dump:
        count = dump_chunks(Path(args.dump), limit=args.limit)
        print(f"Wrote {count} chunks to {args.dump}")
        return 0

    return cmd_build(args.full, args.verbose, args.skip_fts, args.skip_vector)


if __name__ == "__main__":
    raise SystemExit(main())
