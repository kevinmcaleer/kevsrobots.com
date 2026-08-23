---
title: Chunking Notes
description: Split notes into retrieval sized pieces along heading boundaries, and keep the context that makes each piece make sense
layout: lesson
type: page
cover: /learn/obsidian_rag/assets/banners/07_chunking_notes.jpg
date_updated: 2026-08-23
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Chunking is the single highest leverage decision in a RAG pipeline. Get it right and mediocre everything else still works. Get it wrong and no amount of clever prompting rescues it.

## Why not just embed whole notes

Because an embedding is an *average* of meaning, and averages of long documents are mush.

Picture a 2,000 word note called "Robot build log". It covers motor selection, a 3D printing disaster, battery wiring, and a bug in the PID loop. Embed the whole thing as one vector and you get a point that means "vaguely about robot building" - close to nothing in particular.

Ask "what motor driver did I use" and that note might not even make the top five, because its one vector is diluted by everything else in it.

Split it into four chunks and the motor paragraph gets its own vector, pointing squarely at "motor driver". Now it wins.

There is a second reason. When you hand context to the model, you want to spend your tokens on relevant text. Sending 2,000 words to answer a question that needed 80 of them is waste - and the other 1,920 words are actively distracting.

---

## Splitting on headings first

Notes already have structure. Someone - you - decided where the topic changed and put a heading there. Use it.

```python
# vault_rag/chunker.py
"""Split notes into retrieval sized chunks."""

import re
from dataclasses import dataclass

from .config import CHUNK_OVERLAP, CHUNK_SIZE, MIN_CHUNK_SIZE
from .vault import Note

HEADING_RE = re.compile(r"^(#{1,6})\s+(.*)$")


@dataclass
class Chunk:
    """A slice of a note, ready to be embedded."""

    id: str
    text: str
    metadata: dict


def split_by_headings(body: str) -> list[tuple[str, str]]:
    """Break a note into (heading trail, text) sections.

    The heading trail is a breadcrumb like "Setup > Wiring" so a chunk
    still knows where it came from once it is pulled out of the note.
    """
    sections: list[tuple[str, str]] = []
    trail: list[str] = []
    current: list[str] = []

    def flush():
        text = "\n".join(current).strip()
        if text:
            sections.append((" > ".join(trail), text))
        current.clear()

    for line in body.splitlines():
        match = HEADING_RE.match(line)
        if match:
            flush()
            level = len(match.group(1))
            trail[:] = trail[: level - 1]
            while len(trail) < level - 1:
                trail.append("")
            trail.append(match.group(2).strip())
        else:
            current.append(line)
    flush()

    return sections or [("", body.strip())]
```

The `trail` list is the clever part. It tracks the heading hierarchy as we walk down the note:

- Hit an `##` (level 2)? Truncate the trail to 1 entry, then append. So `["Hardware"]` becomes `["Hardware", "Motor wiring"]`.
- Hit a `#` (level 1)? Truncate to 0 and append. The trail resets.
- The `while` loop pads with empty strings for the case where a note jumps from `#` straight to `###`, skipping a level. People do this constantly.

The `or [("", body.strip())]` at the end handles a note with no headings at all - one section, empty trail.

---

## Splitting sections that are still too big

A section under one heading can still run to thousands of characters. Window it:

```python
def split_long_text(text: str, size: int, overlap: int) -> list[str]:
    """Window a long section into overlapping pieces, breaking on paragraphs."""
    if len(text) <= size:
        return [text]

    paragraphs = [p for p in re.split(r"\n\s*\n", text) if p.strip()]
    pieces, current = [], ""

    for paragraph in paragraphs:
        if current and len(current) + len(paragraph) + 2 > size:
            pieces.append(current.strip())
            tail = current[-overlap:] if overlap else ""
            current = f"{tail}\n\n{paragraph}"
        else:
            current = f"{current}\n\n{paragraph}" if current else paragraph

    if current.strip():
        pieces.append(current.strip())

    # A single paragraph can still be longer than the window - hard split it
    final: list[str] = []
    for piece in pieces:
        while len(piece) > size * 1.5:
            final.append(piece[:size])
            piece = piece[size - overlap:]
        final.append(piece)
    return final
```

Two ideas here.

**Break on paragraphs, not on characters.** Cutting mid-sentence produces chunks that start with half a thought, and the embedding of half a thought is not much use. Accumulating whole paragraphs until the next one would overflow keeps every chunk readable.

**Overlap.** Each new piece starts with the last `CHUNK_OVERLAP` characters of the previous one. That is deliberate redundancy: if the answer to a question straddles a boundary - the question is set up at the end of one paragraph and answered at the start of the next - overlap means at least one chunk contains both halves.

The final `while` loop is the escape hatch. Someone somewhere has a 5,000 character paragraph with no blank lines in it (a pasted log file, usually), and without a hard split it would become one enormous chunk.

---

## Why 1200 characters

`CHUNK_SIZE = 1200` is roughly 200 words, or two to three solid paragraphs. It is a starting point, not a law:

| Chunk size | What you get |
|---|---|
| 300-500 | Precise hits, but chunks often lack the context to be understandable alone |
| 800-1500 | The sweet spot for personal notes. Big enough to hold a complete thought |
| 3000+ | Back to the mush problem. Retrieval gets vague and you waste tokens |
{:class="table table-single"}

Lesson 17 shows how to measure this properly on your own vault rather than trusting a table.

---

## Keeping the context: the breadcrumb

Here is a chunk pulled straight out of a note:

> The DRV8833 takes four GPIO pins - two per motor. I used GP6, GP7 for the left motor and GP8, GP9 for the right.

Perfectly clear to you. But which robot? The chunk does not say, and neither does its embedding. Ask "which pins drive the SMARS motors" and the word SMARS appears nowhere in the text, so this chunk scores worse than it should.

The fix is to prepend the breadcrumb to the chunk text *before* embedding it:

> SMARS Robot > SMARS > Motor wiring
>
> The DRV8833 takes four GPIO pins - two per motor...

Now "SMARS" is in the embedded text and the chunk ranks properly. This one change makes a bigger difference to retrieval quality than almost anything else in this course.

There is a wrinkle. Most Obsidian notes open with an H1 that repeats the filename, so the naive breadcrumb reads `SMARS Robot > SMARS > Motor wiring` - the same thing twice. Collapse it:

```python
def breadcrumb(title: str, heading: str) -> str:
    """Join the note title and heading trail, without repeating anything.

    An Obsidian note usually opens with an H1 that repeats its own filename,
    so "SMARS > SMARS > Motor wiring" is common. Collapse it.
    """
    parts = [title, *(h for h in heading.split(" > ") if h)] if heading else [title]
    trail: list[str] = []
    for part in parts:
        if not trail or trail[-1].lower() != part.lower():
            trail.append(part)
    return " > ".join(trail)
```

---

## Putting it together

```python
def chunk_note(note: Note, size: int = CHUNK_SIZE, overlap: int = CHUNK_OVERLAP) -> list[Chunk]:
    """Turn one note into a list of chunks with full metadata."""
    chunks: list[Chunk] = []
    index = 0

    for heading, section_text in split_by_headings(note.body):
        for piece in split_long_text(section_text, size, overlap):
            if len(piece.strip()) < MIN_CHUNK_SIZE:
                continue

            # The heading trail rides along with the text so the embedding
            # captures the context, not just the bare sentence.
            section = breadcrumb(note.title, heading)
            text = f"{section}\n\n{piece.strip()}"

            chunks.append(
                Chunk(
                    id=f"{note.relative_path}::{index}",
                    text=text,
                    metadata={
                        "path": note.relative_path,
                        "title": note.title,
                        "heading": heading,
                        "section": section,
                        "folder": note.folder,
                        "tags": note.tags or ["untagged"],
                        "links": note.links or ["none"],
                        "mtime": note.mtime,
                        "note_hash": note.content_hash,
                        "chunk_index": index,
                    },
                )
            )
            index += 1

    return chunks
```

The chunk id is `robots/smars.md::0`, `robots/smars.md::1` and so on. Deterministic, readable in a debug dump, and derived from the note path - which means re-chunking the same note produces the same ids and `upsert` overwrites cleanly.

`MIN_CHUNK_SIZE` throws away the scraps: a heading with one line under it, a stray "TODO", the two words left over after a split. They match everything weakly and nothing well.

The `or ["untagged"]` and `or ["none"]` matter more than they look. Chroma rejects an empty metadata dict, and a `None` value silently nulls the *entire* metadata entry for that chunk. A placeholder is ugly but keeps every chunk filterable.

---

## Try it Yourself

```python
from vault_rag.config import VAULT_PATH
from vault_rag.vault import iter_notes
from vault_rag.chunker import chunk_note

total, sizes = 0, []
for note in iter_notes(VAULT_PATH):
    chunks = chunk_note(note)
    total += len(chunks)
    sizes.extend(len(c.text) for c in chunks)

sizes.sort()
print(f"{total} chunks")
print(f"median {sizes[len(sizes) // 2]} chars, largest {sizes[-1]}")
```

1. Run it. A healthy median is somewhere around half your `CHUNK_SIZE`, because most sections are short and never need splitting.
2. Print the largest chunk. Is it a pasted log file? That is a candidate for a skip rule.
3. Set `CHUNK_SIZE = 400` and re-run. Chunk count roughly doubles. Set it to `3000` and watch it halve. Note both numbers - lesson 17 uses them.
4. Pick a note with nested headings and print every chunk's `section`. Does the breadcrumb read the way you would describe that section out loud?

---

## Common Issues

- **Problem**: Chunk count is enormous - five times the number of notes.
- **Solution**: Check your daily notes. A vault with 800 dailies, each with three headings, produces a lot of tiny chunks.
- **Why**: Nothing is broken, but consider raising `MIN_CHUNK_SIZE` or skipping the dailies folder if those notes are mostly logistics.

- **Problem**: A chunk's `section` reads `SMARS >  > Motor wiring` with a gap.
- **Solution**: That is the padding for a skipped heading level - the note went from `#` to `###`.
- **Why**: The empty entries are filtered out in `breadcrumb`, so the stored `section` is clean. Only the raw `heading` metadata keeps the gap.

- **Problem**: Retrieval keeps returning the wrong section of the right note.
- **Solution**: Confirm the breadcrumb is actually in the chunk text - print `chunk.text[:80]`.
- **Why**: Without the heading in the embedded text, all chunks from one note look nearly identical to the embedding model.

- **Problem**: Code blocks get split down the middle.
- **Solution**: Accept it, or add a guard that treats a fenced block as an unsplittable unit.
- **Why**: `split_long_text` breaks on blank lines, and code often contains them. For a notes vault the damage is usually small; for a vault full of long code samples it is worth fixing.
