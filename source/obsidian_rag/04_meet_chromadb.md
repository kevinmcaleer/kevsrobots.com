---
title: Meet ChromaDB
description: Collections, documents, metadata and queries - everything ChromaDB does, in one lesson
layout: lesson
type: page
cover: /learn/obsidian_rag/assets/banners/04_meet_chromadb.jpg
date_updated: 2026-08-23
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Before we point ChromaDB at a real vault, let's spend a lesson just playing with it. It has a small API and you can learn nearly all of it in twenty minutes.

## The mental model

ChromaDB stores **documents**. A document is:

- A chunk of **text** - which Chroma embeds for you
- A unique **id** - a string you choose
- Some **metadata** - a flat dictionary of extra facts you want to filter on later

Documents live in a **collection**, which is a bit like a table. We will use exactly one collection for the whole vault.

That is genuinely it.

---

## Your first collection

```python
import chromadb

client = chromadb.PersistentClient(path="./scratch_db")

collection = client.get_or_create_collection(
    name="notes",
    configuration={"hnsw": {"space": "cosine"}},
)

print(collection.count())
```

Three things worth pausing on:

**`PersistentClient`** writes to disk, so the index survives between runs. There is also `chromadb.EphemeralClient()` which keeps everything in memory - great for tests, useless for our tool.

**`get_or_create_collection`** does what it says, which makes it safe to call on every run. `create_collection` raises if the collection already exists.

**`configuration={"hnsw": {"space": "cosine"}}`** sets the distance metric. Chroma's default is squared L2, which works but produces distances that are harder to reason about. Cosine is the right choice for text, and you must set it **when the collection is created** - you cannot change it later without rebuilding.

---

## Adding documents

```python
collection.upsert(
    ids=["note1-0", "note2-0", "note3-0"],
    documents=[
        "The Pico W has a CYW43439 wireless chip and runs MicroPython.",
        "My SMARS robot uses two N20 motors and a DRV8833 driver.",
        "Sourdough starter needs feeding every 12 hours in warm weather.",
    ],
    metadatas=[
        {"note": "Pico.md", "folder": "electronics"},
        {"note": "SMARS.md", "folder": "robots"},
        {"note": "Bread.md", "folder": "kitchen"},
    ],
)

print(collection.count())   # 3
```

Notice we never mentioned embeddings. Chroma spotted that we passed `documents` rather than `embeddings` and ran the default model over them for us.

### Always use upsert, never add

Chroma has an `add()` method too. Use `upsert()` instead, always.

The difference bites in a way you will not notice until it has cost you an hour:

```python
collection.add(ids=["note1-0"], documents=["Completely different text"])
print(collection.get(ids=["note1-0"], include=["documents"])["documents"])
# ['The Pico W has a CYW43439 wireless chip and runs MicroPython.']
```

`add()` saw an id it already had and **silently kept the old document**. No exception, no warning in your output. Edit a note, re-index with `add()`, and your index quietly still holds the old version.

`upsert()` overwrites. That is what you want every single time.

---

## Querying

```python
results = collection.query(
    query_texts=["which board has wifi?"],
    n_results=2,
    include=["documents", "metadatas", "distances"],
)

print(results["ids"])
print(results["distances"])
```

```
[['note1-0', 'note2-0']]
[[0.5076934695243835, 0.8984732627868652]]
```

The Pico note wins, even though the question said "wifi" and the note said "wireless chip".

### Why everything is nested one level deep

Those double brackets trip up everyone. `query_texts` is a **list** - you can ask several questions in one call - so every result is a list-of-lists, one inner list per query.

We only ever ask one thing at a time, so you will see `[0]` all over our code:

```python
for document, metadata, distance in zip(
    results["documents"][0],
    results["metadatas"][0],
    results["distances"][0],
):
    print(f"{distance:.3f}  {metadata['note']}  {document[:40]}...")
```

### Ask for only what you need

`include=["documents", "metadatas", "distances"]` is not optional politeness - it is a real optimisation. Leave it off and older Chroma versions hand back the full 384 number embedding for every hit, which you then throw away.

---

## Metadata filtering

This is where Chroma earns its keep. You can constrain the search *before* the similarity comparison happens:

```python
results = collection.query(
    query_texts=["motors"],
    n_results=5,
    where={"folder": "robots"},
)
print(results["ids"])       # [['note2-0']]
```

The operators available on metadata are:

| Operator | Meaning | Example |
|---|---|---|
| `$eq` / `$ne` | Equals, not equals | `{"folder": {"$eq": "robots"}}` |
| `$gt` `$gte` `$lt` `$lte` | Numeric comparison | `{"mtime": {"$gte": 1700000000}}` |
| `$in` / `$nin` | Value is (not) in a list | `{"folder": {"$in": ["robots", "electronics"]}}` |
| `$contains` / `$not_contains` | List field does (not) contain a value | `{"tags": {"$contains": "pico"}}` |
| `$and` / `$or` | Combine clauses | `{"$and": [clause1, clause2]}` |
{:class="table table-single"}

A bare `{"folder": "robots"}` is shorthand for `{"folder": {"$eq": "robots"}}`.

**`$contains` is for list values, not substrings.** If `tags` is `["pico", "micropython"]` then `{"tags": {"$contains": "pico"}}` matches. If `folder` is the string `"electronics"` then `{"folder": {"$contains": "electr"}}` matches **nothing** - there is no substring operator for metadata.

---

## Filtering on the document text

Separately from metadata, you can filter on the chunk text itself:

```python
collection.query(
    query_texts=["anything"],
    n_results=5,
    where_document={"$contains": "MicroPython"},
)
```

`where_document` supports `$contains`, `$not_contains` and `$regex`. Here `$contains` **is** a substring match, and it **is case sensitive** - `"micropython"` finds nothing. Use `$regex` with an inline flag when you need to ignore case:

```python
where_document={"$regex": "(?i)micropython"}
```

We will use this in lesson 12 to rescue searches for exact part numbers, which is the thing embeddings are worst at.

---

## Reading and deleting

`get()` is the non-semantic sibling of `query()` - no question, no ranking, just "give me the rows matching this filter":

```python
collection.get(where={"note": "Pico.md"}, include=["metadatas"])
collection.get(ids=["note1-0"])
collection.get()                       # everything
```

`delete()` takes the same arguments:

```python
collection.delete(where={"note": "Bread.md"})
collection.delete(ids=["note1-0"])
```

Deleting by `where` is exactly how we will handle an edited note in lesson 9 - drop all of that note's chunks, then write the new ones.

---

## Try it Yourself

1. Add three documents about a topic you know well, then query with a question that uses none of the same words. Did the right one come back?
2. Query with `n_results=100` on a collection holding 3 documents. Chroma returns 3, not an error - handy, because it means you never have to clamp `n_results` yourself.
3. Add a document with metadata `{"tags": ["a", "b"]}` and filter with `{"tags": {"$contains": "a"}}`. Then try `{"tags": {"$in": ["a"]}}` and watch it return nothing. That distinction matters in lesson 11.
4. Delete the whole `scratch_db` folder and re-run your script. Everything rebuilds from nothing - the database really is just that directory.

---

## Common Issues

- **Problem**: `Expected a name containing 3-512 characters from [a-zA-Z0-9._-], starting and ending with a character in [a-zA-Z0-9]`
- **Solution**: Your collection name is too short or has an illegal character. `"v"` fails; `"vault_notes"` is fine.
- **Why**: Chroma validates collection names because they become directory names on disk.

- **Problem**: `Expected metadata to be a non-empty dict, got 0 metadata attributes`
- **Solution**: Never pass `{}` as a metadata entry. Pass at least one key, or leave the whole `metadatas` argument off.
- **Why**: An empty dict is almost always a bug in your code rather than a deliberate choice, so Chroma refuses it.

- **Problem**: Metadata comes back as `None` for a document you definitely gave metadata to.
- **Solution**: Check for a `None` *value* in the dict - something like `{"title": None}` when a note had no title.
- **Why**: A `None` value causes the entire metadata dictionary to be stored as `None`. Coerce your values to strings, or drop the key, before writing. We handle this properly in lesson 8.

- **Problem**: Distances look wrong, or all results seem equally close.
- **Solution**: Check you passed `configuration={"hnsw": {"space": "cosine"}}` when the collection was **created**.
- **Why**: Setting it on an existing collection is ignored silently. Delete the database directory and rebuild.
