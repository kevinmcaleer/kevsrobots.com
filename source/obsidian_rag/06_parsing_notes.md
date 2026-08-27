---
title: Frontmatter, Tags and Wikilinks
description: Parse the YAML frontmatter, collect inline tags and extract wikilinks so you can filter on them later
layout: lesson
type: page
cover: /learn/obsidian_rag/assets/banners/06_parsing_notes.jpg
date_updated: 2026-08-23
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Right now every note's `body` includes its frontmatter block, its `tags` list is empty, and its `title` is just the filename. Let's fix all three.

Why bother? Because metadata is what turns a search into a *useful* search. "What did I write about motors" is fine. "What did I write about motors, in notes tagged `robots`, since March" is much better - and that only works if the tags and dates are in the index.

---

## Stripping the frontmatter

An Obsidian note often opens with a YAML block between two `---` lines:

```markdown
---
title: Raspberry Pi Pico W
tags: [pico, micropython, wifi]
created: 2026-03-14
---

# Raspberry Pi Pico W

The Pico W adds a CYW43439 wireless chip...
```

That block must come out of the body before we embed anything. Leave it in and every chunk from that note carries `created: 2026-03-14` along with it, which is noise the embedding model will happily average into the meaning.

```python
# vault_rag/vault.py  (the two imports go at the top of the file,
#                      the rest below the Note class)
import re
import yaml

FRONTMATTER_RE = re.compile(
    r"\A---[ \t]*\r?\n(.*?)^---[ \t]*\r?$\r?\n?",
    re.DOTALL | re.MULTILINE,
)


def split_frontmatter(text: str) -> tuple[dict, str]:
    """Pull the YAML frontmatter block off the front of a note."""
    match = FRONTMATTER_RE.match(text)
    if not match:
        return {}, text
    try:
        data = yaml.safe_load(match.group(1)) or {}
    except yaml.YAMLError:
        data = {}
    if not isinstance(data, dict):
        data = {}
    return data, text[match.end():]
```

That regex looks fiddly, so here is what each piece is defending against:

- **`\A`** anchors to the very start of the string. A `---` horizontal rule halfway down a note is not frontmatter.
- **`.*?`** is non-greedy, so it stops at the *first* closing `---`, not the last one in the file.
- **`^---...$` with `re.MULTILINE`** requires the closing marker to be on a line of its own. It also means an *empty* frontmatter block - just `---` then `---` - is matched and stripped. The simpler pattern you will see in most tutorials silently leaves those two lines sitting in your note body.
- **`\r?`** in three places handles CRLF line endings, which any vault that has ever touched a Windows machine will contain.
- **`[ \t]*`** tolerates trailing spaces after the markers, which some editors add.
- **`re.DOTALL`** lets `.` match newlines, which it does not by default.

And three guards around the YAML, all of which come from real vaults:

- `yaml.safe_load` on an empty block returns `None`, hence `or {}`.
- Broken YAML - a stray colon in an unquoted title is the classic - raises `YAMLError`. One bad note should not stop the indexing run.
- A frontmatter block containing only a list parses to a `list`, not a `dict`, and then `data.get(...)` explodes later. The `isinstance` check catches it.

Never use `yaml.load()` here. `safe_load` cannot construct arbitrary Python objects, and your vault may well contain notes you pasted from the internet.

---

## Collecting tags

Obsidian has two kinds of tag and people use both, often in the same note:

```markdown
---
tags: [robots, 3dprinting]
---

Printed the new chassis today. #petg #worked-first-time
```

Frontmatter tags can be a list, or a single string, or a comma separated string, depending on which Obsidian version wrote them and whether a human hand-edited it. Inline tags are `#word` anywhere in the body.

```python
# vault_rag/vault.py  (add below split_frontmatter)
INLINE_TAG_RE = re.compile(r"(?:^|\s)#([A-Za-z][\w/-]*)")
CODE_FENCE_RE = re.compile(r"```.*?```", re.DOTALL)


def extract_tags(frontmatter: dict, body: str) -> list[str]:
    """Collect tags from frontmatter and from #inline-tags in the body."""
    tags: list[str] = []

    raw = frontmatter.get("tags") or frontmatter.get("tag") or []
    if isinstance(raw, str):
        raw = [t.strip() for t in raw.replace(",", " ").split()]
    if isinstance(raw, list):
        tags.extend(str(t).lstrip("#") for t in raw if t)

    # Ignore anything inside a code fence, or "#" would match every comment
    prose = CODE_FENCE_RE.sub(" ", body)
    tags.extend(INLINE_TAG_RE.findall(prose))

    seen, unique = set(), []
    for tag in tags:
        tag = tag.strip().lower()
        if tag and tag not in seen:
            seen.add(tag)
            unique.append(tag)
    return unique
```

The details that matter:

**`[A-Za-z]` as the first character** means a tag must start with a letter. Without it, every markdown heading in your vault becomes a tag - `## Setup` would give you a tag called `Setup`. (A heading is safe anyway because of the space after the `#`, but `#2026` in a date would otherwise become a tag, and Obsidian does not allow purely numeric tags either.)

**Code fences are stripped first.** A note with a Python snippet in it is full of `# comments`. Without this you end up with tags called `initialise`, `TODO` and `set`.

**`[\w/-]` in the tail** allows nested tags like `#project/smars` and hyphenated ones like `#worked-first-time`, both of which Obsidian supports.

**Lowercasing and de-duplicating** at the end means `#Robots` in one note and `#robots` in another are the same tag, and the order stays stable for a clean hash.

---

## Extracting wikilinks

```python
# vault_rag/vault.py  (add below extract_tags)
WIKILINK_RE = re.compile(r"\[\[([^\]|#]+)")


def extract_links(body: str) -> list[str]:
    """Find the [[wikilinks]] in a note."""
    return sorted({link.strip() for link in WIKILINK_RE.findall(body) if link.strip()})
```

The character class `[^\]|#]` stops at the first `]`, `|` or `#`, which handles all three Obsidian link forms in one go:

| Written in the note | Extracted |
|---|---|
| `[[SMARS]]` | `SMARS` |
| `[[SMARS\|my robot]]` | `SMARS` |
| `[[SMARS#Motor wiring]]` | `SMARS` |
{:class="table table-single"}

We are storing links mainly so they are there when you want them - a natural extension is "also retrieve the notes that this note links to". The `sorted(set(...))` keeps the list stable, which matters because unstable metadata makes diffing an index harder than it needs to be.

---

## Wiring it into load_note

```python
# vault_rag/vault.py  (replaces the load_note you wrote in lesson 5)
def load_note(path: Path, vault_path: Path) -> Note:
    """Read and parse one markdown file."""
    text = path.read_text(encoding="utf-8", errors="replace")
    frontmatter, body = split_frontmatter(text)
    relative_path = str(path.relative_to(vault_path))
    return Note(
        path=path,
        relative_path=relative_path,
        title=str(frontmatter.get("title") or path.stem),
        body=body.strip(),
        frontmatter=frontmatter,
        tags=extract_tags(frontmatter, body),
        links=extract_links(body),
        mtime=path.stat().st_mtime,
    )
```

`str(frontmatter.get("title") or path.stem)` prefers an explicit frontmatter title and falls back to the filename. The `str()` wrapper is not decoration - a note titled `2026-03-14` parses out of YAML as a `datetime.date` object, and passing that to Chroma later gives you an unhelpful error a long way from the cause.

---

## Try it Yourself

```python
# try_it.py - a scratch script in the project root
from vault_rag.config import VAULT_PATH
from vault_rag.vault import iter_notes

from collections import Counter

tag_counts = Counter()
for note in iter_notes(VAULT_PATH):
    tag_counts.update(note.tags)

for tag, count in tag_counts.most_common(20):
    print(f"{count:5}  #{tag}")
```

1. Run that against your vault. Are there tags in there you did not know you used? Anything obviously junk from a code block that slipped through?
2. Find your most-linked-to note: build a `Counter` over `note.links` instead. That is usually a hub note, and hub notes are worth retrieving well.
3. Deliberately break a note's frontmatter with an unquoted colon (`title: Pico: the good one`) and confirm indexing carries on rather than crashing.
4. Print `note.frontmatter` for a few notes. Anything else in there worth indexing? A `status` or `project` field makes an excellent filter.

---

## Common Issues

- **Problem**: Tags include things like `include`, `define` or `TODO`.
- **Solution**: Your code fence stripping is not running, or the note uses indented code blocks rather than triple backticks.
- **Why**: `CODE_FENCE_RE` only matches triple-backtick fences. For four-space-indented code, also strip lines matching `^ {4,}\S`.

- **Problem**: `yaml.scanner.ScannerError` escaping from a note.
- **Solution**: Widen the `except` to `yaml.YAMLError`, which is the base class for all of PyYAML's parse errors.
- **Why**: `ScannerError`, `ParserError` and `ComposerError` are all subclasses. Catching only one lets the others through.

- **Problem**: Frontmatter is not being stripped and shows up in your chunk text.
- **Solution**: Print `repr(text[:80])` for that note and look at the line endings and the exact marker.
- **Why**: The usual culprits are CRLF endings, a marker written as `----`, or a leading blank line before the opening `---` - and `\A` means a leading blank line is fatal. The regex above handles the first two.

- **Problem**: One note has 200 tags.
- **Solution**: Look at the note. It is almost certainly a MOC or index note that lists every tag in your vault.
- **Why**: Nothing is broken, but that note will match far too many tag filters. Consider skipping notes with an absurd tag count.
