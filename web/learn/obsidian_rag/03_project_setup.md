---
layout: lesson
title: Setting Up the Project
author: Kevin McAleer
type: page
cover: /learn/obsidian_rag/assets/banners/03_project_setup.jpg
date: 2026-08-23
previous: 02_embeddings_and_vectors.html
next: 04_meet_chromadb.html
description: Create the virtual environment, install the dependencies and lay out
  the Ask My Vault package
percent: 20
duration: 5
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

Time to build something. In this lesson we set up the project skeleton that every later lesson fills in.

## The virtual environment

ChromaDB pulls in a fair number of dependencies, so keep it out of your system Python:

```bash
mkdir ask-my-vault
cd ask-my-vault
python3 -m venv .venv
source .venv/bin/activate      # Windows: .venv\Scripts\activate
```

Your prompt should now show `(.venv)`. If it does not, the activate step did not take.

---

## Dependencies

Three packages, and that is the lot:

```
chromadb>=1.5.0
anthropic>=1.0.0
pyyaml>=6.0
```

Save that as `requirements.txt` and install it:

```bash
pip install -r requirements.txt
```

What each one is for:

| Package | Job |
|---|---|
| `chromadb` | The vector database, plus the local embedding model |
| `anthropic` | Talking to Claude for the answer generation step |
| `pyyaml` | Parsing the YAML frontmatter at the top of your notes |
{:class="table table-single"}

The install takes a minute or two - `chromadb` brings along a Rust extension and an ONNX runtime. Check it landed:

```bash
python -c "import chromadb; print(chromadb.__version__)"
```

---

## The package layout

We are building a real tool, not a script, so it gets a proper package. Create this structure:

```
ask-my-vault/
├── .venv/
├── requirements.txt
└── vault_rag/
    ├── __init__.py      # version, and nothing else
    ├── config.py        # every setting in one place
    ├── vault.py         # read and parse notes
    ├── chunker.py       # split notes into chunks
    ├── index.py         # build the ChromaDB index
    ├── retrieve.py      # search the index
    ├── answer.py        # ask Claude
    └── cli.py           # the command line interface
```

One file per stage of the pipeline. When retrieval misbehaves you know exactly which file to open.

```bash
mkdir vault_rag
touch vault_rag/{__init__,config,vault,chunker,index,retrieve,answer,cli}.py
```

---

## Two files to write now

`__init__.py` is tiny:

```python
# vault_rag/__init__.py
"""Ask My Vault - a retrieval augmented generation tool for Obsidian."""

__version__ = "1.0.0"
```

`config.py` is the one that matters. Every tunable value in the whole project lives here, so that when you want to experiment with chunk sizes you change one number in one place:

```python
# vault_rag/config.py
"""All the knobs in one place."""

import os
from pathlib import Path

# Where your Obsidian vault lives
VAULT_PATH = Path(os.environ.get("VAULT_PATH", "~/Obsidian/MyVault")).expanduser()

# Where the Chroma database is stored
DB_PATH = Path(os.environ.get("VAULT_DB_PATH", "~/.ask-my-vault/chroma")).expanduser()

COLLECTION_NAME = "vault_notes"

# Chunking, in characters
CHUNK_SIZE = 1200
CHUNK_OVERLAP = 200
MIN_CHUNK_SIZE = 60

# Retrieval
TOP_K = 6
MAX_CHUNKS_PER_NOTE = 2

# Generation
ANSWER_MODEL = "claude-opus-5"
MAX_ANSWER_TOKENS = 2000

# Folders we never want to index
SKIP_DIRS = {".obsidian", ".trash", ".git", ".smart-env", "node_modules", "templates"}
```

A few notes on the choices there:

- **`os.environ.get` with a default** means you can point the tool at a different vault without editing code - handy when you want to test against a scratch vault.
- **`.expanduser()`** turns `~/Obsidian` into a real absolute path. Chroma will not expand the tilde for you, and the error you get if you forget is genuinely baffling.
- **The database lives outside the vault**, in `~/.ask-my-vault/`. Put it *inside* the vault and Obsidian will try to sync a binary database across your devices, which ends badly.
- **`templates` is in `SKIP_DIRS`** because template notes are full of placeholder text that pollutes retrieval with junk.

---

## Point it at your vault

Rather than editing `config.py`, set the environment variable:

```bash
export VAULT_PATH="$HOME/Documents/Obsidian/MyVault"
```

Add it to your `~/.zshrc` or `~/.bashrc` so it survives a new terminal. Check it worked:

```bash
python -c "from vault_rag.config import VAULT_PATH; print(VAULT_PATH, VAULT_PATH.is_dir())"
```

You want to see your vault path followed by `True`.

---

## A safety net before you point at real notes

We are going to be reading files and writing a database. Reading is harmless, but while you are learning it is more pleasant to work against a small vault where you know exactly what should come back.

Make one:

```bash
mkdir -p testvault/electronics testvault/robots
cat > testvault/electronics/pico.md <<'EOF'
---
title: Raspberry Pi Pico W
tags: [pico, micropython]
---

# Raspberry Pi Pico W

The Pico W adds a CYW43439 wireless chip for 2.4GHz Wi-Fi.
The onboard LED is on the wireless chip, not GPIO 25.
EOF
cat > testvault/robots/smars.md <<'EOF'
---
title: SMARS Robot
tags: [robots, 3dprinting]
---

# SMARS

Two N20 geared motors driven by a DRV8833.
Left motor on GP6 and GP7, right motor on GP8 and GP9.
EOF
```

Two notes, and you know precisely what is in them. When retrieval returns something odd you will be able to tell immediately.

---

## Try it Yourself

1. Run `pip list | wc -l` after installing. ChromaDB is not a small dependency - worth knowing what you signed up for.
2. Add a `PROJECT_ROOT` to `config.py` using `Path(__file__).parent.parent` and print it. Useful later for finding data files relative to the package.
3. Set `VAULT_PATH` to a folder that does not exist and print `VAULT_PATH.is_dir()`. It returns `False` rather than raising - we will turn that into a proper error message in lesson 5.

---

## Common Issues

- **Problem**: `pip install chromadb` fails to build a wheel.
- **Solution**: Upgrade pip first with `pip install --upgrade pip`, then retry.
- **Why**: Chroma ships pre-built wheels for common platforms. An old pip does not know how to select them and falls back to compiling from source, which needs a Rust toolchain you probably do not have.

- **Problem**: `ModuleNotFoundError: No module named 'vault_rag'` when running your code.
- **Solution**: Run from the project root - the directory *containing* `vault_rag`, not from inside it. Use `python -m vault_rag.cli`, not `python vault_rag/cli.py`.
- **Why**: Relative imports like `from .config import ...` only resolve when Python is treating `vault_rag` as a package. The `-m` flag does that; running the file directly does not.

- **Problem**: `VAULT_PATH` prints as `~/Obsidian/MyVault` with a literal tilde.
- **Solution**: You dropped the `.expanduser()`.
- **Why**: The shell expands `~`, but a string in Python is just a string. `Path("~/x").is_dir()` is always `False`.
