---
layout: lesson
title: Walking the Vault
author: Kevin McAleer
type: page
cover: /learn/obsidian_rag/assets/banners/05_reading_the_vault.jpg
date: 2026-08-23
previous: 04_meet_chromadb.html
next: 06_parsing_notes.html
description: Find every markdown note in an Obsidian vault, skip the folders you do
  not want, and load them into Note objects
percent: 30
duration: 6
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

An Obsidian vault is just a folder of markdown files. No database, no proprietary format - which is the whole reason Obsidian is worth building tools for.

That does mean a few things are lurking in there that you do not want in your index.

## What to skip

| Folder | Why skip it |
|---|---|
| `.obsidian` | Plugin config, themes, workspace layout - thousands of lines of JSON and the odd stray markdown file |
| `.trash` | Notes you deleted. Retrieving one would be genuinely confusing |
| `.git` | If you version control your vault |
| `.smart-env` | Cache folder left by some AI plugins |
| `templates` | Placeholder text like `{% raw %}{{title}}{% endraw %}` that matches everything and means nothing |
{:class="table table-single"}

Those are already in `SKIP_DIRS` in `config.py`. Add your own as you find them.

---

## The Note object

Everything downstream works with `Note` objects rather than raw file paths, so the parsing happens exactly once:

```python
# vault_rag/vault.py
"""Read an Obsidian vault from disk and turn it into Note objects."""

import hashlib
from dataclasses import dataclass, field
from pathlib import Path

from .config import SKIP_DIRS


@dataclass
class Note:
    """One markdown file from the vault, already parsed."""

    path: Path
    relative_path: str
    title: str
    body: str
    frontmatter: dict = field(default_factory=dict)
    tags: list[str] = field(default_factory=list)
    links: list[str] = field(default_factory=list)
    mtime: float = 0.0

    @property
    def folder(self) -> str:
        parent = Path(self.relative_path).parent
        return "" if str(parent) == "." else str(parent)

    @property
    def content_hash(self) -> str:
        """Fingerprint of the note's text - changes when the note changes."""
        return hashlib.sha256(self.body.encode("utf-8")).hexdigest()[:16]
```

`frontmatter`, `tags` and `links` are declared but stay empty until the next lesson. The two properties are worth a closer look.

**`folder`** turns `robots/smars.md` into `robots`, and a note at the vault root into `""`. Obsidian users organise by folder constantly, so this makes a great filter.

**`content_hash`** is the key to fast re-indexing. Hash the body, keep the first 16 hex characters, and you have a fingerprint that changes whenever the note's text changes. In lesson 9 we compare that against what is stored in the index and skip anything unchanged. Note that it hashes the **body**, not the file - so touching a file without editing it does not trigger a re-index.

---

## Loading one note

```python
def load_note(path: Path, vault_path: Path) -> Note:
    """Read and parse one markdown file."""
    text = path.read_text(encoding="utf-8", errors="replace")
    relative_path = str(path.relative_to(vault_path))
    return Note(
        path=path,
        relative_path=relative_path,
        title=path.stem,
        body=text.strip(),
        mtime=path.stat().st_mtime,
    )
```

Two small details doing real work:

**`errors="replace"`** stops one badly encoded file from killing the whole indexing run. A note pasted from a Windows machine years ago can contain bytes that are not valid UTF-8; `replace` swaps them for a placeholder character and carries on.

**`relative_path`** is what we store in the index, not the absolute path. Move your vault to a different machine and the index still makes sense.

---

## Walking the whole vault

```python
def iter_notes(vault_path: Path):
    """Yield every indexable note in the vault, in a stable order."""
    vault_path = Path(vault_path).expanduser()
    if not vault_path.is_dir():
        raise FileNotFoundError(f"Vault not found: {vault_path}")

    for path in sorted(vault_path.rglob("*.md")):
        parts = set(path.relative_to(vault_path).parts)
        if parts & SKIP_DIRS:
            continue
        note = load_note(path, vault_path)
        if note.body:
            yield note
```

Four decisions in nine lines:

**It is a generator.** `yield`, not `return [...]`. A big vault has thousands of notes and there is no reason to hold them all in memory at once - the indexer processes them one at a time and lets each go.

**`sorted()`** makes the order deterministic. Without it, `rglob` returns whatever order the filesystem feels like, which differs between macOS and Linux and makes debugging miserable.

**The skip check uses a set intersection.** `parts` is every path component - `("robots", "old", "smars.md")` - so `parts & SKIP_DIRS` catches a skipped folder at *any* depth, not just the top level. A `templates` folder nested three levels down still gets skipped.

**Empty notes are dropped.** An empty file produces an empty embedding and pollutes results. Obsidian creates these constantly when you click "new note" and then wander off.

---

## Try it Yourself

Point it at your real vault:

```python
from vault_rag.config import VAULT_PATH
from vault_rag.vault import iter_notes

notes = list(iter_notes(VAULT_PATH))
print(f"{len(notes)} notes")

# The five biggest - these are the ones chunking has to handle well
for note in sorted(notes, key=lambda n: len(n.body), reverse=True)[:5]:
    print(f"{len(note.body):7,} chars  {note.relative_path}")
```

Then try these:

1. Compare `len(notes)` against `find ~/YourVault -name "*.md" | wc -l`. The difference is your skipped and empty notes. Is it the number you expected?
2. Print the ten *smallest* notes. Anything under about 100 characters is probably not worth indexing - a useful thing to know before you tune `MIN_CHUNK_SIZE`.
3. Add `"attachments"` to `SKIP_DIRS` and see whether your count changes.
4. Print the same note's `content_hash` twice, then edit the note and print it again. Watch it change.

---

## Common Issues

- **Problem**: `FileNotFoundError: Vault not found: ~/Obsidian/MyVault`
- **Solution**: The tilde was not expanded. Our `iter_notes` calls `.expanduser()` for you, so this means the path genuinely does not exist - check for a typo or a space in the folder name.
- **Why**: `Path("~/x")` is a literal path with a `~` directory in it, which almost certainly does not exist.

- **Problem**: Notes from `.obsidian` still appear.
- **Solution**: Confirm you are intersecting against `path.relative_to(vault_path).parts` and not `path.parts`.
- **Why**: `path.parts` includes every component of the absolute path. If your vault happens to live under a folder called `templates`, you would skip the entire vault.

- **Problem**: Indexing is slow just to *walk* the vault, before any embedding.
- **Solution**: Check for an `attachments` folder full of PDFs or images. `rglob("*.md")` still has to stat everything it walks past.
- **Why**: `rglob` walks the entire directory tree. Adding heavy asset folders to `SKIP_DIRS` does not help here because the walk happens first - if it is a real problem, use `os.walk` and prune directories as you go.

- **Problem**: `UnicodeDecodeError` on one specific file.
- **Solution**: You dropped `errors="replace"` from `read_text`.
- **Why**: Not every file in a long-lived vault is clean UTF-8.
