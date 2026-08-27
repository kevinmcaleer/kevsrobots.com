"""Split pages into retrieval-sized chunks that carry their own context.

Two ideas do most of the work here:

1. Split on headings first, because the author already decided where the topic
   changed and put a heading there.
2. Prepend a breadcrumb to each chunk's text *before* embedding it, so a
   paragraph about wiring still knows it belongs to "SMARS > Motor wiring".
   This matters more to retrieval quality than anything else in the pipeline.
"""

import re
from dataclasses import dataclass

from .config import CHUNK_OVERLAP, CHUNK_SIZE, INCLUDE_COURSE_IN_TEXT, MIN_CHUNK_SIZE
from .content import Page


@dataclass
class Chunk:
    """A slice of a page, ready to be embedded."""

    id: str
    text: str
    metadata: dict


def split_long_text(text: str, size: int = CHUNK_SIZE, overlap: int = CHUNK_OVERLAP) -> list[str]:
    """Window a long section into overlapping pieces, breaking on paragraphs.

    Cutting mid-sentence produces chunks that start with half a thought, and
    the embedding of half a thought is not much use. Accumulating whole
    paragraphs until the next one would overflow keeps every chunk readable.
    """
    if len(text) <= size:
        return [text]

    paragraphs = [p for p in re.split(r"\n\s*\n", text) if p.strip()]
    pieces: list[str] = []
    current = ""

    for paragraph in paragraphs:
        if current and len(current) + len(paragraph) + 2 > size:
            pieces.append(current.strip())
            tail = current[-overlap:] if overlap else ""
            current = f"{tail}\n\n{paragraph}"
        else:
            current = f"{current}\n\n{paragraph}" if current else paragraph

    if current.strip():
        pieces.append(current.strip())

    # A single paragraph can still exceed the window - a pasted log, a big
    # table. Hard split those rather than emitting one enormous chunk.
    final: list[str] = []
    for piece in pieces:
        while len(piece) > size * 1.5:
            final.append(piece[:size])
            piece = piece[size - overlap :]
        if piece.strip():
            final.append(piece)
    return final


def breadcrumb(title: str, trail: str) -> str:
    """Join the page title and heading trail without repeating anything.

    Most pages open with an H1 that repeats the title, so the naive breadcrumb
    reads "Walking the Vault > Walking the Vault > What to skip". Collapse it.
    """
    parts = [title, *(h for h in trail.split(" > ") if h)] if trail else [title]
    collapsed: list[str] = []
    for part in parts:
        part = part.strip()
        if not part:
            continue
        if not collapsed or collapsed[-1].lower() != part.lower():
            collapsed.append(part)
    return " > ".join(collapsed)


def _clean_metadata(metadata: dict) -> dict:
    """Coerce values into what Chroma accepts.

    Chroma stores str, int, float, bool or list-of-str. A None value does not
    raise - it silently nulls the *entire* metadata dict for that chunk, so you
    lose every other field too. Placeholders are ugly but keep chunks filterable.
    """
    clean = {}
    for key, value in metadata.items():
        if value is None:
            continue
        if isinstance(value, (list, tuple, set)):
            items = [str(v).strip() for v in value if str(v).strip()]
            clean[key] = items or ["none"]
        elif isinstance(value, bool):
            clean[key] = value
        elif isinstance(value, (int, float)):
            clean[key] = value
        else:
            text = str(value).strip()
            clean[key] = text if text else "none"
    return clean


def _embedded_prefix(page: Page, crumb: str) -> str:
    """The context line that gets embedded along with the chunk body.

    The breadcrumb alone leaves a chunk unable to be found by its own subject:
    a lesson on the Q-learning update rule never writes "reinforcement
    learning", so a query using the course's own topic words misses it
    entirely. Naming the course closes that gap.
    """
    if INCLUDE_COURSE_IN_TEXT and page.course and page.course.lower() not in crumb.lower():
        return f"{page.course} > {crumb}"
    return crumb


def chunk_page(
    page: Page, size: int = CHUNK_SIZE, overlap: int = CHUNK_OVERLAP
) -> list[Chunk]:
    """Turn one page into a list of chunks with full metadata."""
    chunks: list[Chunk] = []
    index = 0

    for section in page.sections:
        for piece in split_long_text(section.text, size, overlap):
            piece = piece.strip()
            if not piece:
                continue

            crumb = breadcrumb(page.title, section.trail)
            prefix = _embedded_prefix(page, crumb)

            if len(piece) < MIN_CHUNK_SIZE:
                # Too small to stand alone as a chunk - a one-line section, a
                # stray "TODO", the tail of a split. Rather than dropping it
                # (which loses real facts: "Left motor: GP6 and GP7" is short
                # and is exactly what someone searches for), fold it into the
                # previous chunk, keeping its heading so the context survives.
                if chunks and len(chunks[-1].text) + len(piece) < size * 1.4:
                    chunks[-1].text += f"\n\n{crumb.rsplit(' > ', 1)[-1]}: {piece}"
                continue

            text = f"{prefix}\n\n{piece}"

            metadata = _clean_metadata(
                {
                    "url": page.url,
                    "title": page.title,
                    "section": crumb,
                    "heading": section.trail,
                    "description": page.description,
                    "page_type": page.page_type,
                    "date": page.date,
                    "author": page.author,
                    "cover_image": page.cover_image,
                    "course": page.course,
                    "course_slug": page.course_slug,
                    "tags": page.tags or ["untagged"],
                    "rating": page.rating,
                    "content_hash": page.content_hash,
                    "chunk_index": index,
                }
            )

            chunks.append(Chunk(id=f"{page.url}::{index}", text=text, metadata=metadata))
            index += 1

    return chunks
