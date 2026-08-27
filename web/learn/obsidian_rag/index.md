---
layout: lesson
title: Introduction
author: Kevin McAleer
type: page
cover: /learn/obsidian_rag/assets/banners/00_intro.jpg
date: 2026-08-23
next: 01_what_is_rag.html
description: Build Ask My Vault, a Python tool that answers questions about your Obsidian
  notes using ChromaDB and Claude
percent: 5
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

Ahoy there makers! If you have been using Obsidian for a while you have probably hit the same wall I did. You know you wrote something down about that motor driver. You know it is in there somewhere. But search gives you forty hits for "motor" and none of them are the one you want.

Search looks for words. You want to search for *meaning*.

That is what we are going to build. By the end of this course you will have a tool called **Ask My Vault** - a Python command line app that reads your entire vault, understands what each note is actually about, and answers questions like *"what motor driver did I use on the SMARS build and why did I switch?"* with a proper answer and a list of the notes it got that answer from.

---

## Overview

The technique is called **RAG** - Retrieval Augmented Generation. It is two ideas glued together:

1. **Retrieval** - find the handful of paragraphs in your notes that are actually relevant to the question. We will use [ChromaDB](https://www.trychroma.com), a small vector database that runs as a Python library with no server to set up.
2. **Generation** - hand those paragraphs to a large language model and ask it to write an answer *using only those paragraphs*, with citations.

The important part is that second constraint. A model on its own will happily make things up about your robot. A model that has been handed your actual notes and told to cite them will tell you when your notes do not contain the answer - which is often the most useful thing it can say.

Everything except the final answer step runs entirely on your own machine. The embedding model is a small local one that ChromaDB downloads for you. Your notes never leave your laptop during indexing.

---

## Course Content

- What RAG is, and why plain keyword search is not enough
- Embeddings, vectors, and how "meaning" becomes numbers
- Setting up the project and the Ask My Vault package layout
- ChromaDB basics - collections, documents, metadata and queries
- Walking an Obsidian vault and skipping the folders you do not want
- Parsing YAML frontmatter, inline tags and wikilinks
- Chunking notes so retrieval finds the right paragraph, not the whole file
- Building the index, in batches, with useful metadata attached
- Incremental re-indexing so a rebuild takes a second, not ten minutes
- Querying the index and understanding what a distance score means
- Filtering by tag, folder and modified date
- Improving retrieval with oversampling, diversity and better chunk text
- Sending the retrieved context to Claude and getting an answer back
- Citations, grounding, and getting the model to admit it does not know
- Building the finished command line tool
- Keeping the index fresh automatically
- Evaluating whether your RAG is actually any good
- Troubleshooting the things that will definitely go wrong

---

## Key Results

After completing this course, you will:

- Understand how embeddings turn text into searchable meaning
- Be able to build and maintain a ChromaDB index from any folder of markdown
- Know how to chunk documents so retrieval actually works
- Be able to write a grounded prompt that cites its sources and refuses to guess
- Have a working `amv` command you can run against your own vault
- Know how to measure whether a change to your pipeline made it better or worse

---

## What you'll need

- **Python 3.10 or newer** - we use modern type hints like `list[str] | None`
- **An Obsidian vault** with at least twenty or thirty notes in it. A tiny vault makes for boring results
- **About 500MB of disk space** - the local embedding model is roughly 80MB, and Chroma's index grows with your vault
- **An Anthropic API key** for the answer generation step, from [console.anthropic.com](https://console.anthropic.com). The retrieval half of the course works with no key at all
- A terminal, and an editor you like

No GPU required. Everything here runs comfortably on a laptop, and the retrieval side runs fine on a Raspberry Pi 5.

---

## Prerequisites

You should be comfortable with basic Python - functions, classes, dictionaries and list comprehensions. If you are not there yet, start with:

- [Learn Python](/learn/python/) - the fundamentals
- [Obsidian](/learn/obsidian/) - if you are new to Obsidian itself

---

## How the course works

Each lesson builds one piece of the tool. Code appears in blocks like this:

```python
# Every code block in this course is complete and runnable
print("Hello, vault")
```

**Every code block tells you where it goes.** The first line is a comment naming the file, so you are never left guessing:

```python
# vault_rag/config.py
CHUNK_SIZE = 1200
```

When a block adds to a file you already started, the comment says where to put it - and when it replaces something from an earlier lesson, it says that too:

```python
# vault_rag/vault.py  (add below the Note class)
# vault_rag/vault.py  (replaces the load_note you wrote in lesson 5)
```

Some blocks are throwaway experiments rather than part of the tool. Those name a scratch file instead, which you can delete afterwards:

```python
# try_it.py - a scratch script in the project root
```

Blocks with no file comment are either shell commands or short fragments illustrating a point.

Most lessons finish with a **Try it Yourself** section - small experiments that show you the behaviour rather than just telling you about it - and a **Common Issues** section for the things that trip people up.

Right then. Let's find out what RAG actually is.
