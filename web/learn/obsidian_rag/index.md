---
layout: course_overview
title: Build a RAG for Your Obsidian Vault
author: Kevin McAleer
cover: /learn/obsidian_rag/assets/cover.jpg
date: 2026-08-23
description: Build Ask My Vault - a Python tool that indexes your Obsidian notes into
  ChromaDB and answers questions about them, with citations back to the original note.
  Learn embeddings, chunking, vector search and grounded generation by building the
  whole pipeline yourself.
level: intermediate
course_duration: 3h 6m
lesson_count: 20
first_lesson: 00_intro.html
quiz_total: 76
outcomes:
- Explain how embeddings turn text into points you can search by meaning
- Parse an Obsidian vault into chunks that keep their heading context
- Build and incrementally update a ChromaDB index with tag, folder and date filters
- Write a grounded prompt that cites its sources and admits when your notes do not
  know
- Run an amv command line tool against your own vault
- Measure retrieval with hit rate and mean reciprocal rank so you know a change helped
prerequisites:
- Python for beginners
- Obsidian
you_need:
- A computer with Python 3.10 or newer and about 500MB of free disk space
- An Obsidian vault of Markdown notes (a four-note practice vault is built in lesson
  3)
- An Anthropic API key for the answer step in lessons 13 to 15 (lessons 1 to 12 need
  none)
- Internet access for the first run, when the embedding model downloads
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
