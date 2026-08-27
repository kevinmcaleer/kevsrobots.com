#!/usr/bin/env python3
"""Measure retrieval quality against the golden question set.

Judging generated prose is subjective. Judging retrieval is not: did the right
page come back, and how high up? That is the metric worth building on - if the
right page is retrieved a decent prompt almost always produces a good answer,
and if it is not, nothing downstream can save you.

Two numbers:

* **Hit rate @k** - the fraction of questions where the expected page appeared
  anywhere in the top k. Blunt, but it tracks how the tool feels.
* **MRR** - 1/rank of the right answer, averaged. Rewards ranking the right
  page *higher*, which hit rate cannot see.

Compares the semantic/hybrid engine against the FTS baseline so a change can be
shown to have helped rather than just felt different.

    python3 eval/evaluate.py                 # both engines, side by side
    python3 eval/evaluate.py --engine rag    # just the new one
    python3 eval/evaluate.py --top-k 5
"""

import argparse
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

DEFAULT_EVAL = Path(__file__).resolve().parent / "eval.yml"


def _rag_urls(question: str, top_k: int) -> list[str]:
    from rag.retrieve import search

    return [hit.url for hit in search(question, top_k=top_k)]


def _fts_urls(question: str, top_k: int) -> list[str]:
    from search.database import query_documents

    rows = query_documents(question, offset=0, limit=top_k)
    return [row["url"] for row in rows]


ENGINES = {"rag": _rag_urls, "fts": _fts_urls}


def evaluate(engine: str, cases: list[dict], top_k: int) -> dict:
    get_urls = ENGINES[engine]

    hits = 0
    reciprocal_ranks: list[float] = []
    misses: list[tuple[str, str]] = []
    scored = 0

    for case in cases:
        question = case["question"]
        expected = case.get("expect")
        try:
            urls = get_urls(question, top_k)
        except Exception as error:
            misses.append((question, f"engine error: {error}"))
            reciprocal_ranks.append(0.0)
            scored += 1
            continue

        if expected is None:
            # "Should find nothing useful" is a generation-side property, not a
            # retrieval one - our search always returns the nearest chunks
            # however far away they are. Report these separately rather than
            # scoring them as retrieval failures.
            continue

        scored += 1
        rank = next((i for i, url in enumerate(urls, 1) if url.startswith(expected)), None)
        if rank:
            hits += 1
            reciprocal_ranks.append(1 / rank)
        else:
            reciprocal_ranks.append(0.0)
            misses.append((question, f"wanted {expected}, got {urls[:3]}"))

    return {
        "engine": engine,
        "scored_cases": scored,
        "hit_rate": hits / scored if scored else 0.0,
        "mrr": sum(reciprocal_ranks) / scored if scored else 0.0,
        "misses": misses,
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Measure retrieval quality.")
    parser.add_argument("--eval", type=Path, default=DEFAULT_EVAL)
    parser.add_argument("--top-k", type=int, default=10)
    parser.add_argument("--engine", choices=[*ENGINES, "both"], default="both")
    parser.add_argument("--fail-under", type=float, default=None,
                        help="Exit non-zero if the RAG hit rate is below this (0-1)")
    args = parser.parse_args(argv)

    if not args.eval.is_file():
        print(f"ERROR: no evaluation set at {args.eval}", file=sys.stderr)
        return 1

    cases = yaml.safe_load(args.eval.read_text(encoding="utf-8")) or []
    engines = list(ENGINES) if args.engine == "both" else [args.engine]

    print(f"Evaluating {len(cases)} questions at k={args.top_k}\n")

    results = {}
    for engine in engines:
        results[engine] = evaluate(engine, cases, args.top_k)

    print(f"{'engine':<8} {'scored':>7} {'hit rate':>10} {'MRR':>8}")
    print("-" * 36)
    for engine in engines:
        r = results[engine]
        print(f"{engine:<8} {r['scored_cases']:>7} {r['hit_rate']:>9.0%} {r['mrr']:>8.3f}")

    if "rag" in results and "fts" in results:
        delta_hit = results["rag"]["hit_rate"] - results["fts"]["hit_rate"]
        delta_mrr = results["rag"]["mrr"] - results["fts"]["mrr"]
        print(f"\nRAG vs FTS: hit rate {delta_hit:+.0%}, MRR {delta_mrr:+.3f}")

    for engine in engines:
        misses = results[engine]["misses"]
        if misses:
            print(f"\n{engine} misses ({len(misses)}) - the to-do list:")
            for question, detail in misses:
                print(f"  {question}\n      {detail}")

    if args.fail_under is not None and "rag" in results:
        if results["rag"]["hit_rate"] < args.fail_under:
            print(f"\nFAIL: RAG hit rate {results['rag']['hit_rate']:.0%} "
                  f"is below the {args.fail_under:.0%} threshold", file=sys.stderr)
            return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
