---
layout: lesson
title: Building the Ask My Vault CLI
author: Kevin McAleer
type: page
cover: /learn/obsidian_rag/assets/banners/15_the_cli.jpg
date: 2026-08-23
previous: 14_citations_and_grounding.html
next: 16_keeping_it_fresh.html
description: Wire every piece together into a real command line tool with index, search,
  ask and stats commands
percent: 80
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

Every piece works. Time to put a handle on it.

## Four commands

| Command | What it does |
|---|---|
| `amv index` | Build or update the index |
| `amv search "question"` | Show the raw chunks that match - no model, no cost |
| `amv ask "question"` | Get a grounded answer with citations |
| `amv stats` | How much is indexed, and where |
{:class="table table-single"}

`search` and `ask` being separate commands is the most useful design decision in the whole tool. When an answer is wrong, `search` tells you *immediately* whether the retriever found the right material. Half your debugging time disappears.

---

## The skeleton

```python
# vault_rag/cli.py
"""Ask My Vault - command line interface."""

import argparse
import sys
import time
from pathlib import Path

from .config import DB_PATH, TOP_K, VAULT_PATH
from .index import collection_stats, index_vault
from .retrieve import build_filter, search
```

Notice `answer` is **not** imported here. It gets imported inside `cmd_ask`, because importing it pulls in the `anthropic` package, and there is no reason for `amv search` to pay that cost.

---

## The index command

```python
def cmd_index(args) -> int:
    print(f"Indexing {args.vault}")
    start = time.time()
    stats = index_vault(args.vault, args.db, full=args.full, verbose=args.verbose)
    print(f"Done in {time.time() - start:.1f}s: {stats.summary()}")
    return 0
```

Each command returns an exit code. `0` for success, non-zero for failure - so `amv index && amv ask "..."` behaves the way a shell user expects.

---

## The search command

```python
def cmd_search(args) -> int:
    where = build_filter(tag=args.tag, folder=args.folder)
    hits = search(args.question, top_k=args.top_k, db_path=args.db, where=where)
    if not hits:
        print("No matches. Have you run `amv index` yet?")
        return 1
    for number, hit in enumerate(hits, start=1):
        print(f"\n[{number}] {hit.label}  ({hit.score:.0%})")
        print(f"    {hit.path}")
        snippet = hit.text.replace("\n", " ")[:180]
        print(f"    {snippet}...")
    return 0
```

`hit.text.replace("\n", " ")` flattens the chunk to one line. Markdown chunks are full of newlines and without this the output turns into a mess that is impossible to scan.

---

## The ask command

```python
def cmd_ask(args) -> int:
    from .answer import check_citations, stream_answer

    where = build_filter(tag=args.tag, folder=args.folder)
    hits = search(args.question, top_k=args.top_k, db_path=args.db, where=where)
    if not hits:
        print("Nothing in your vault matches that. Try `amv index` first.")
        return 1

    pieces = []
    for piece in stream_answer(args.question, hits):
        pieces.append(piece)
        print(piece, end="", flush=True)

    cited, invalid = check_citations("".join(pieces), len(hits))

    print("\n\nSources:")
    for number, hit in enumerate(hits, start=1):
        marker = " " if number in cited else "-"
        print(f" {marker}[{number}] {hit.path}  ({hit.score:.0%})")

    if invalid:
        print(f"\nWarning: cited sources that do not exist: {sorted(invalid)}")
    return 0
```

We collect the streamed pieces into a list as well as printing them, so the citation check from lesson 14 has the full text to work with. The `-` marker in the source list flags anything that was retrieved but never cited.

---

## Stats, and the parser

```python
def cmd_stats(args) -> int:
    info = collection_stats(args.db)
    print(f"Database: {info['path']}")
    print(f"Notes indexed: {info['notes']}")
    print(f"Chunks stored: {info['chunks']}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="amv", description="Ask questions about your Obsidian vault."
    )
    parser.add_argument("--db", type=Path, default=DB_PATH,
                        help="Where the Chroma database lives")
    subparsers = parser.add_subparsers(dest="command", required=True)

    index_parser = subparsers.add_parser("index", help="Index or re-index the vault")
    index_parser.add_argument("--vault", type=Path, default=VAULT_PATH)
    index_parser.add_argument("--full", action="store_true",
                              help="Wipe and rebuild from scratch")
    index_parser.add_argument("--quiet", dest="verbose", action="store_false")
    index_parser.set_defaults(func=cmd_index)

    for name, handler, help_text in [
        ("search", cmd_search, "Show the raw chunks that match"),
        ("ask", cmd_ask, "Get an answer with citations"),
    ]:
        sub = subparsers.add_parser(name, help=help_text)
        sub.add_argument("question")
        sub.add_argument("--top-k", type=int, default=TOP_K)
        sub.add_argument("--tag", help="Only search notes with this tag")
        sub.add_argument("--folder", help="Only search notes in this folder")
        sub.set_defaults(func=handler)

    stats_parser = subparsers.add_parser("stats", help="Show index statistics")
    stats_parser.set_defaults(func=cmd_stats)

    return parser
```

`search` and `ask` take identical arguments, so they are built in a loop. Two blocks of copy-pasted `add_argument` calls drift apart the first time you add an option to one and forget the other.

`set_defaults(func=...)` is the argparse pattern worth knowing. Each subparser attaches its own handler to the parsed args, so dispatch is one line rather than a chain of `if args.command == ...`.

---

## Main

```python
def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return args.func(args)
    except FileNotFoundError as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
```

`main(argv=None)` taking an argument list means you can test the whole CLI without a subprocess: `assert main(["stats"]) == 0`.

`FileNotFoundError` gets a friendly message because "your vault path is wrong" is a user error, not a bug. Everything else keeps its traceback - if the tool breaks you want to see why.

`130` is the conventional exit code for Ctrl-C, and catching `KeyboardInterrupt` means interrupting a long index run prints nothing ugly.

---

## Making it a real command

Right now you run it as `python -m vault_rag.cli`. Let's shorten that. Create `pyproject.toml` in the project root:

```toml
[project]
name = "ask-my-vault"
version = "1.0.0"
requires-python = ">=3.10"
dependencies = [
    "chromadb>=1.5.0",
    "anthropic>=1.0.0",
    "pyyaml>=6.0",
]

[project.scripts]
amv = "vault_rag.cli:main"

[build-system]
requires = ["setuptools>=61"]
build-backend = "setuptools.build_meta"

[tool.setuptools]
packages = ["vault_rag"]
```

That last block is not optional, even though most tutorials leave it out. Without it setuptools tries to guess which folders are packages, sees `testvault/` sitting next to `vault_rag/`, cannot decide, and fails the install with a wall of text about "package discovery". Naming the one package you actually ship settles it.

Then install it in editable mode:

```bash
pip install -e .
```

Now `amv` is a command. Editable mode means your source directory *is* the installed package, so edits take effect immediately with no reinstall.

---

## Using it

```bash
amv index
```

```
Indexing /Users/kev/Obsidian/MyVault
  indexed Daily.md (1 chunks)
  indexed electronics/Pico W.md (3 chunks)
  indexed kitchen/Sourdough.md (1 chunks)
  indexed robots/SMARS.md (3 chunks)
Done in 0.4s: 4 new, 0 changed, 0 unchanged, 0 removed (8 chunks written)
```

```bash
amv search "which GPIO pins drive the motors?"
```

```
[1] SMARS Robot > SMARS > Motor wiring  (52%)
    robots/SMARS.md
    SMARS Robot > SMARS > Motor wiring  The DRV8833 takes four GPIO pins - two per motor. I used GP6, GP7 for the left motor and GP8, GP9 for the right...

[2] SMARS Robot > SMARS  (45%)
    robots/SMARS.md
    SMARS Robot > SMARS  My SMARS uses two N20 geared motors and a DRV8833 motor driver...
```

```bash
amv ask "which GPIO pins drive the motors?" --tag robots
```

```
The SMARS uses a DRV8833, which needs four GPIO pins - two per motor. The left
motor is on GP6 and GP7, the right on GP8 and GP9 [1].

Sources:
  [1] robots/SMARS.md  (52%)
 -[2] robots/SMARS.md  (45%)
```

That is the tool finished.

---

## Try it Yourself

1. Index your real vault and ask it three questions you genuinely wanted answered. How did it do?
2. Add a `--json` flag to `search` that dumps hits as JSON. Now you can pipe it into `jq` or another script.
3. Add an `open` command that takes a result number and opens that note: `subprocess.run(["open", str(VAULT_PATH / hit.path)])` on macOS, `xdg-open` on Linux.
4. Add an interactive mode - a `while True` loop reading questions with `input()`. The embedding model stays loaded between questions, so the second question comes back much faster than the first.
5. Add a `--no-stream` flag for when you want to pipe the answer into another program.

---

## Common Issues

- **Problem**: `pip install -e .` fails with `error: Multiple top-level packages discovered in a flat-layout`.
- **Solution**: Add the `[tool.setuptools] packages = ["vault_rag"]` block shown above.
- **Why**: Auto-discovery only works when exactly one candidate package sits in the project root. A `testvault/` folder is enough to break it.

- **Problem**: `amv: command not found` after `pip install -e .`
- **Solution**: Check your virtualenv is active. The `amv` script is installed into `.venv/bin/`, which is only on your PATH while the venv is activated.
- **Why**: Editable installs put the entry point in the environment's `bin`, not in a global location.

- **Problem**: `error: the following arguments are required: command`
- **Solution**: You ran bare `amv`. That is `required=True` on the subparsers doing its job.
- **Why**: Without it, bare `amv` would silently do nothing, which is a worse experience than an error.

- **Problem**: The question gets split across several arguments.
- **Solution**: Quote it: `amv ask "why did the motors stall"`.
- **Why**: The shell splits on spaces before argparse ever sees it. You could use `nargs="+"` and join, but requiring quotes is clearer.

- **Problem**: `amv ask` is slow to start even before the answer streams.
- **Solution**: That is the embedding model loading, which happens once per process.
- **Why**: A CLI exits between invocations so you pay it every time. Interactive mode - Try it Yourself number 4 - is the fix.

- **Problem**: The `--db` flag has no effect.
- **Solution**: It is a top-level argument, so it goes *before* the subcommand: `amv --db ./testdb search "..."`.
- **Why**: argparse binds arguments to the parser they were added to, and subcommand arguments come after the subcommand name.
