---
layout: lesson
title: Querying the Index
author: Kevin McAleer
type: page
cover: /learn/obsidian_rag/assets/banners/10_querying_the_index.jpg
date: 2026-08-23
previous: 09_incremental_indexing.html
next: 11_metadata_filters.html
description: Turn a question into ranked hits, understand what the distance means,
  and stop one note hogging every slot
percent: 55
duration: 8
date_updated: 2026-08-23
navigation:
- name: Build a RAG for Your Obsidian Vault
- content:
  - section: Overview
    content:
    - name: Introduction
      link: 00_intro.html
    - name: What RAG Actually Is
      link: 01_what_is_rag.html
    - name: Embeddings and Vectors
      link: 02_embeddings_and_vectors.html
  - section: Setting Up
    content:
    - name: Setting Up the Project
      link: 03_project_setup.html
    - name: Meet ChromaDB
      link: 04_meet_chromadb.html
  - section: Reading Your Vault
    content:
    - name: Walking the Vault
      link: 05_reading_the_vault.html
    - name: Frontmatter, Tags and Wikilinks
      link: 06_parsing_notes.html
    - name: Chunking Notes
      link: 07_chunking_notes.html
  - section: Building the Index
    content:
    - name: Building the Index
      link: 08_building_the_index.html
    - name: Incremental Re-indexing
      link: 09_incremental_indexing.html
  - section: Retrieval
    content:
    - name: Querying the Index
      link: 10_querying_the_index.html
    - name: Filtering by Tag, Folder and Date
      link: 11_metadata_filters.html
    - name: Making Retrieval Better
      link: 12_improving_retrieval.html
  - section: Generating Answers
    content:
    - name: Asking Claude
      link: 13_generating_answers.html
    - name: Citations and Staying Grounded
      link: 14_citations_and_grounding.html
  - section: The Finished Tool
    content:
    - name: Building the Ask My Vault CLI
      link: 15_the_cli.html
    - name: Keeping the Index Fresh
      link: 16_keeping_it_fresh.html
    - name: Is It Actually Any Good
      link: 17_evaluating_your_rag.html
  - section: Wrapping Up
    content:
    - name: Troubleshooting
      link: 18_troubleshooting.html
    - name: Summary and Next Steps
      link: 19_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

The index is built. Now let's ask it things.

## A hit, not a dictionary

Chroma hands back parallel lists. Working with those directly gets ugly fast, so the first thing we do is turn them into objects:

```python
# vault_rag/retrieve.py
"""Search the index and pick the best chunks to hand to the model."""

from dataclasses import dataclass
from pathlib import Path

from .config import DB_PATH, MAX_CHUNKS_PER_NOTE, TOP_K
from .index import get_collection


@dataclass
class Hit:
    """One retrieved chunk, with its score and where it came from."""

    text: str
    path: str
    title: str
    section: str
    distance: float

    @property
    def score(self) -> float:
        """Cosine distance turned into a 0-1 similarity, easier to read."""
        return max(0.0, 1.0 - self.distance)

    @property
    def label(self) -> str:
        return self.section or self.title
```

`score` exists purely for humans. Everyone reads "72%" faster than "distance 0.28", and `max(0.0, ...)` stops a distance above 1.0 - which happens for genuinely unrelated text - from displaying as a negative percentage.

Do **not** rank by `score` and do not filter on it. Chroma already returned the results in distance order. This is a display convenience, nothing more.

---

## The search function

```python
# vault_rag/retrieve.py  (add below the Hit class)
def search(question: str, top_k: int = TOP_K, db_path: Path = DB_PATH,
           where: dict | None = None, oversample: int = 3) -> list[Hit]:
    """Find the chunks most similar to the question."""
    collection = get_collection(db_path)
    if collection.count() == 0:
        return []

    result = collection.query(
        query_texts=[question],
        n_results=min(top_k * oversample, collection.count()),
        where=where,
        include=["documents", "metadatas", "distances"],
    )

    hits = [
        Hit(
            text=document,
            path=metadata["path"],
            title=metadata.get("title", metadata["path"]),
            section=metadata.get("section", ""),
            distance=distance,
        )
        for document, metadata, distance in zip(
            result["documents"][0], result["metadatas"][0], result["distances"][0]
        )
    ]

    return diversify(hits, top_k)
```

A few things doing quiet work.

**The empty check.** It is guarding the line below it. On an empty collection, `min(top_k * oversample, collection.count())` evaluates to `0`, and Chroma rejects that outright:

```
TypeError: Number of requested results 0, cannot be negative, or zero.
```

Returning `[]` early lets the CLI print "have you run index yet?" instead of a traceback. (Querying an empty collection with a sensible `n_results` is fine and just returns nothing - it is the zero that breaks.)

**`zip` over the three `[0]` lists.** That is the nested-result unwrapping from lesson 4. One question in, one inner list out.

**`metadata.get("title", metadata["path"])`** falls back to the path if a chunk somehow has no title. Defensive, but the alternative is a `KeyError` in the middle of a search.

**Oversampling.** We ask Chroma for `top_k * 3` results and then narrow them down ourselves. That is the next section.

---

## Oversampling and diversity

Ask "how do the SMARS motors work" against a vault with a long, detailed SMARS note and you get this:

```
[1] SMARS Robot > Motor wiring      (52%)  robots/SMARS.md
[2] SMARS Robot > Motors            (49%)  robots/SMARS.md
[3] SMARS Robot > Motor upgrades    (47%)  robots/SMARS.md
[4] SMARS Robot > Gearbox notes     (44%)  robots/SMARS.md
[5] SMARS Robot > Chassis           (41%)  robots/SMARS.md
[6] SMARS Robot > Parts list        (38%)  robots/SMARS.md
```

Six slots, one note. Technically these *are* the six closest chunks. But your build log from last March that says "swapped the DRV8833 for a TB6612FNG because of the heat" never gets a look in, and that is the note that actually answers the question.

The fix is a cap per note:

```python
# vault_rag/retrieve.py  (add below search)
def diversify(hits: list[Hit], limit: int, per_note: int = MAX_CHUNKS_PER_NOTE) -> list[Hit]:
    """Stop one chatty note from filling every slot."""
    kept: list[Hit] = []
    counts: dict[str, int] = {}
    for hit in hits:
        if counts.get(hit.path, 0) >= per_note:
            continue
        counts[hit.path] = counts.get(hit.path, 0) + 1
        kept.append(hit)
        if len(kept) == limit:
            break
    return kept
```

Walk the hits in rank order, keep each one unless we already have `per_note` from that note, stop at `limit`.

This is why we oversample. If we had asked Chroma for exactly 6 and then thrown 4 of them away, we would return 2 results. Asking for 18 and filtering down to 6 means there is always enough material to fill the slots from different notes.

Same query, with diversity on:

```
[1] SMARS Robot > Motor wiring      (52%)  robots/SMARS.md
[2] SMARS Robot > Motors            (49%)  robots/SMARS.md
[3] Build log 2026-03 > Motors      (44%)  daily/2026-03-14.md
[4] Parts drawer > Motor drivers    (39%)  reference/parts.md
[5] SMARS Quad > Motor selection    (37%)  robots/SMARS Quad.md
[6] Battery notes > Current draw    (31%)  electronics/battery.md
```

Six slots, five notes, and the build log made it in. Much better material for the model to work from.

---

## Trying it out

```python
# try_it.py - a scratch script in the project root
from vault_rag.retrieve import search

for number, hit in enumerate(search("which GPIO pins drive the motors?", db_path="testdb"), 1):
    print(f"[{number}] {hit.label}  ({hit.score:.0%})")
    print(f"    {hit.path}")
    print(f"    {hit.text[:100].replace(chr(10), ' ')}...")
```

```
[1] SMARS Robot > SMARS > Motor wiring  (52%)
    robots/SMARS.md
    SMARS Robot > SMARS > Motor wiring  The DRV8833 takes four GPIO pins - two per motor...
[2] SMARS Robot > SMARS  (45%)
    robots/SMARS.md
    SMARS Robot > SMARS  My SMARS uses two N20 geared motors and a DRV8833 motor driver...
[3] Daily > 2026-08-20  (20%)
    Daily.md
    Daily > 2026-08-20  Ordered more PETG. Tested the SMARS on carpet - the N20 motors stall...
```

Look at those percentages. The best hit is **52%** - and it is unambiguously the correct answer. Result three at 20% is loosely related. Result six, on a bigger vault, would be under 5% and pure noise.

This is why absolute thresholds do not work. A rule like "reject anything under 60%" would have thrown away every single correct answer here.

---

## Should you cut off low scoring hits at all?

Sometimes, yes - but do it *relative* to the best hit, not against a fixed number:

```python
# vault_rag/retrieve.py  (optional - add below diversify)
def drop_stragglers(hits: list[Hit], ratio: float = 0.5) -> list[Hit]:
    """Keep hits that are at least `ratio` as good as the best one."""
    if not hits:
        return hits
    floor = hits[0].score * ratio
    return [h for h in hits if h.score >= floor]
```

With a top hit of 52%, that keeps everything down to 26%. With a top hit of 90%, it demands 45%. The bar moves with the quality of the match, which is the behaviour you actually want.

It is optional. Extra weak context costs a few tokens and a well-prompted model ignores it. But if your answers keep wandering off topic, this is a good knob to reach for.

---

## Try it Yourself

1. Search your real vault for something you know is in there. Did it come first? If not, look at what *did* win and ask yourself why the embedding thought that was closer.
2. Set `oversample=1` and search a topic you have written about a lot. Watch one note swallow every slot.
3. Set `MAX_CHUNKS_PER_NOTE = 1`. More variety, but a long note can no longer contribute two adjacent paragraphs that together hold the answer. Which do you prefer on your own vault?
4. Print the *worst* hit from a `top_k=20` search. Absolute garbage, or vaguely related? That tells you how much room `drop_stragglers` has to work with.
5. Search for a single word like "motor", then for a full question like "why did the motors stall on carpet". The full question usually wins - embeddings have more to work with.

---

## Common Issues

- **Problem**: Every result is from the same note despite `diversify`.
- **Solution**: Check the `path` metadata is actually different. If you index the same vault under two different absolute paths, `relative_path` saves you - but only if you index relative to the vault root.
- **Why**: `diversify` groups by `hit.path`, so identical paths count as one note.

- **Problem**: `IndexError: list index out of range` on `result["documents"][0]`.
- **Solution**: The collection is empty, or `include` did not request documents.
- **Why**: Chroma returns `None` rather than an empty list for anything you did not include, and `None[0]` fails in a way that does not obviously point at `include`.

- **Problem**: Scores are all above 100% or come out negative.
- **Solution**: The collection was not created with cosine space.
- **Why**: With squared L2, distances are unbounded, so `1 - distance` is meaningless. Rebuild with `configuration={"hnsw": {"space": "cosine"}}`.

- **Problem**: Search is slow on the first call and fast afterwards.
- **Solution**: Normal - the embedding model is loading into memory.
- **Why**: The model loads lazily on first use. For a CLI that exits between commands you pay it every time, which is one reason lesson 15 offers an interactive mode.
