---
title: What RAG Actually Is
description: Why keyword search fails on your own notes, and how retrieval augmented generation fixes it
layout: lesson
type: page
cover: /learn/obsidian_rag/assets/banners/01_what_is_rag.jpg
date_updated: 2026-08-23
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Let's start with the problem, because RAG only makes sense once you have felt the pain.

## The problem with searching your own notes

Say you have a note that says:

> The N20 motors stall on carpet. Swapped to the 100:1 gearbox version and it climbs fine now.

Six months later you type "torque" into Obsidian's search box. Nothing. You type "gear ratio". Nothing. The note is right there, it answers your question perfectly, and search cannot find it - because you never used those words.

Keyword search matches **strings**. You want to match **meaning**.

---

## Two ways to fix it, one of which is a trap

The obvious idea is: just paste the whole vault into a chatbot and ask.

That fails for three reasons:

| Problem | What actually happens |
|---|---|
| Size | A 2,000 note vault is millions of tokens. Even a large context window fills up, and you pay for every token |
| Noise | Bury three relevant sentences in a million words of shopping lists and the model's accuracy drops |
| Cost and latency | You resend the entire vault on every single question |
{:class="table table-single"}

The better idea is: find the five paragraphs that matter, and send only those.

That is RAG.

---

## The two halves

**Retrieval augmented generation** is exactly what the name says - you *augment* the model's *generation* by *retrieving* relevant context first.

```
Your question
     |
     v
[ RETRIEVE ]  <---- searches your indexed notes, returns the top few chunks
     |
     v
[ GENERATE ]  <---- Claude reads only those chunks and writes the answer
     |
     v
Answer + citations
```

Two completely separate systems, and you can debug them separately. That turns out to matter enormously. When your RAG gives a bad answer there are only two possible causes:

- **The retriever handed over the wrong paragraphs.** The model never had a chance.
- **The retriever was fine and the model mangled it.** A prompt problem, not a search problem.

Being able to tell those apart in ten seconds is why we will build a `search` command alongside the `ask` command. One shows you exactly what was retrieved; the other shows you what was done with it.

---

## What gets built when

Here is the pipeline we are constructing, and which lesson builds each piece:

| Stage | What it does | Lesson |
|---|---|---|
| Read | Walk the vault, parse frontmatter and tags | 5, 6 |
| Chunk | Split notes into paragraph sized pieces | 7 |
| Embed | Turn each chunk into a list of numbers | 8 |
| Store | Save vectors plus metadata in ChromaDB | 8, 9 |
| Retrieve | Find the chunks nearest to the question | 10, 11, 12 |
| Generate | Ask Claude, grounded in those chunks | 13, 14 |
{:class="table table-single"}

The first five stages happen once, when you index. Only retrieve and generate happen when you ask a question - which is why asking is fast even on a huge vault.

---

## Why a vector database and not just a file

You could store every chunk's numbers in a JSON file and compare them all on every query. For a hundred notes that works fine. For ten thousand chunks it takes long enough to be annoying, and you would have to write the "find the closest" logic yourself.

ChromaDB gives you three things for free:

- **An approximate nearest neighbour index** so finding the closest chunks stays fast as the vault grows
- **A local embedding model** so you do not need an API key just to search
- **Metadata filtering** so you can say "only search notes tagged `robots`, modified this year"

And it is a `pip install`. No Docker, no server, no port to remember. For a personal tool that runs on your own laptop, that is exactly the right trade.

---

## Try it Yourself

Before writing any code, get a feel for the problem in your own vault:

1. Open Obsidian and search for a concept you know you have written about - but deliberately use *different words* than you would have used when writing. How many of the right notes come back?
2. Now search using the exact phrase you think you wrote. Better? That gap between the two searches is the gap RAG closes.
3. Count your notes: `find ~/Obsidian -name "*.md" | wc -l`. Keep that number in mind - we will check indexing speed against it later.

---

## Common Issues

- **Problem**: "Can I not just use Obsidian's built-in AI plugins?"
- **Solution**: You can, and several are good. Build this anyway.
- **Why**: Every one of those plugins is this pipeline with the lid welded shut. When it returns a bad answer you cannot see why. Building it yourself means you can look at the retrieved chunks, change the chunk size, and fix it.

- **Problem**: "Does this send all my notes to an AI company?"
- **Solution**: No. Indexing is entirely local - the embedding model runs on your machine. Only the handful of retrieved chunks for a specific question are sent, and only when you run `ask`.
- **Why**: Embedding and generation are separate models. We use a local one for embedding and a hosted one for writing the answer. If you want a fully offline tool, lesson 19 covers swapping the generation step for a local model too.
