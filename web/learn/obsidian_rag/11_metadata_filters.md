---
layout: lesson
title: Filtering by Tag, Folder and Date
author: Kevin McAleer
type: page
cover: /learn/obsidian_rag/assets/banners/11_metadata_filters.jpg
date: 2026-08-23
previous: 10_querying_the_index.html
next: 12_improving_retrieval.html
description: Narrow the search before it happens using Chroma metadata filters, and
  understand what each operator really does
percent: 60
duration: 7
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

Semantic search finds things that *mean* the same. Metadata filters find things that *are* a certain kind. Together they are much stronger than either alone.

"What did I write about power consumption" searches everything. "What did I write about power consumption, in notes tagged `pico`, since June" searches the twelve notes that could possibly be relevant - and the results are dramatically better, because there is far less to be wrong about.

---

## Filtering happens before ranking

This is the bit worth internalising. A `where` clause is not a post-filter on the results. Chroma applies it *first*, then finds the nearest neighbours **within** what survives.

That means a filtered search does not return fewer results - it returns *different* ones. Ask for 6 hits with a tag filter and you get the 6 best chunks from that tag, even if none of them would have made the unfiltered top 50.

Which is exactly what you want, and occasionally surprising: filter hard enough and you will get low-scoring results, because the best available match in a small pool is not very good. That is the filter telling you something true.

---

## Building the clause

```python
# vault_rag/retrieve.py  (add below diversify)
def build_filter(tag: str | None = None, folder: str | None = None,
                 since: float | None = None) -> dict | None:
    """Turn CLI options into a Chroma `where` clause."""
    clauses = []
    if tag:
        clauses.append({"tags": {"$contains": tag.lower()}})
    if folder:
        clauses.append({"folder": {"$eq": folder}})
    if since:
        clauses.append({"mtime": {"$gte": since}})

    if not clauses:
        return None
    return clauses[0] if len(clauses) == 1 else {"$and": clauses}
```

Three details that are not obvious:

**Returning `None` when there are no clauses.** Chroma wants `where=None` for "no filter". An empty dict `{}` is not the same thing and produces an error.

**Unwrapping a single clause.** `{"$and": [one_clause]}` is rejected - `$and` needs at least two operands. Since one filter is the common case, handle it explicitly rather than discovering it at runtime.

**`tag.lower()`.** We lowercased tags at index time in lesson 6, so we must lowercase at query time too. Any normalisation you apply to stored data has to be applied to the query as well - forget one side and the filter silently matches nothing.

---

## Tags are a list, so use $contains

Our `tags` metadata is a list like `["pico", "micropython", "wifi"]`. For list valued metadata, `$contains` asks "is this value one of the entries":

```python
where={"tags": {"$contains": "pico"}}     # matches
```

The trap is that `$in` looks like it should do the same thing, and does not:

```python
where={"tags": {"$in": ["pico"]}}         # matches NOTHING
```

`$in` asks "is the field's value *equal to* one of these" - it compares the whole list against each candidate. A list is never equal to a string, so nothing matches, and Chroma raises no error because the query is perfectly valid. It just returns an empty result and lets you wonder why.

Use `$contains` for list fields. Use `$in` for scalar fields where you want to accept several values:

```python
where={"folder": {"$in": ["robots", "electronics"]}}     # correct use of $in
```

---

## Combining clauses

```python
where = {
    "$and": [
        {"tags": {"$contains": "robots"}},
        {"mtime": {"$gte": 1750000000}},
    ]
}
```

`$or` works the same way. You can nest them, though in practice more than two levels means your query is doing something a human should probably decide instead.

Here is a friendlier "since" than a raw Unix timestamp:

```python
# vault_rag/retrieve.py  (import time at the top, days_ago below build_filter)
import time

def days_ago(days: int) -> float:
    """Unix timestamp for N days back, for use with the mtime filter."""
    return time.time() - days * 86400

where = build_filter(tag="robots", since=days_ago(90))
```

---

## Seeing it work

```python
# try_it.py - a scratch script in the project root
from vault_rag.retrieve import build_filter, search

# Unfiltered - the whole vault competes
for hit in search("how do I save power", db_path="testdb"):
    print(f"{hit.score:>4.0%}  {hit.path}")

print()

# Only notes tagged pico
where = build_filter(tag="pico")
for hit in search("how do I save power", db_path="testdb", where=where):
    print(f"{hit.score:>4.0%}  {hit.path}")
```

```
 22%  electronics/Pico W.md
  6%  electronics/Pico W.md
```

Filtered to the `pico` tag, only chunks from that note can compete - and the top score of 22% is honest about the fact that this small test vault does not have a great answer.

Folder filtering behaves the same way:

```python
where = build_filter(folder="kitchen")
# -> [1] Sourdough > Sourdough starter  (45%)
```

---

## What you cannot filter on

Chroma's metadata operators are `$eq`, `$ne`, `$gt`, `$gte`, `$lt`, `$lte`, `$in`, `$nin`, `$contains` and `$not_contains`. Notably missing:

- **Substring match on a string field.** `{"folder": {"$contains": "electr"}}` matches nothing - there is no `LIKE`. If you want prefix matching on folders, store the top-level folder as its own metadata field at index time.
- **Regex on metadata.** `$regex` exists for `where_document` but not for `where`.
- **Case insensitive comparison.** Normalise at index time, as we do with tags.

The general principle: anything you want to filter on has to be **computed at index time and stored as its own field**. Filters are cheap lookups, not computations.

That is worth planning for. If you think you might one day want "only notes with more than 500 words", add a `word_count` to the metadata now - re-indexing to add a field is a full rebuild.

---

## Try it Yourself

1. Add a `--tag` filter to a search on your real vault, using your most-used tag. Compare the top result with and without it.
2. Add a `word_count` field to `chunk_note`'s metadata, re-index with `full=True`, then filter with `{"word_count": {"$gte": 100}}`. Do longer chunks give better answers?
3. Store the top-level folder as a separate `top_folder` field: `note.relative_path.split("/")[0]`. Now you can filter a whole area of your vault without knowing the exact subfolder.
4. Filter to a tag that only appears on one note, then search for something completely unrelated. Watch the score collapse - proof that the filter runs first.
5. Try `{"tags": {"$not_contains": "archive"}}` to exclude a whole category rather than include one. Often more useful than you would think.

---

## Common Issues

- **Problem**: A tag filter returns nothing, but you can see the tag in your notes.
- **Solution**: Check the case. We store lowercase; `build_filter` lowercases the query for you, but a hand-written `where` clause will not.
- **Why**: `$contains` on a list is an exact string comparison per entry. `"Robots"` and `"robots"` are different strings.

- **Problem**: `Expected where to have exactly one operator, got {}` or a similar complaint about an empty dict.
- **Solution**: Return `None` rather than `{}` when there are no filters.
- **Why**: Chroma validates the shape of `where` strictly and an empty dict is not a valid filter.

- **Problem**: `$and` with a single clause errors.
- **Solution**: Unwrap it, as `build_filter` does.
- **Why**: `$and` and `$or` are defined as taking two or more operands.

- **Problem**: The `mtime` filter never matches.
- **Solution**: Print an actual stored `mtime` and compare magnitudes with your filter value.
- **Why**: `mtime` is Unix seconds, around 1.7 billion. Comparing against a value in milliseconds - or against a `datetime` - matches nothing, and Chroma will not warn you that you are comparing incomparable numbers.
