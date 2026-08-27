---
layout: lesson
title: Embeddings and Vectors
author: Kevin McAleer
type: page
cover: /learn/obsidian_rag/assets/banners/02_embeddings_and_vectors.jpg
date: 2026-08-23
previous: 01_what_is_rag.html
next: 03_project_setup.html
description: How text becomes a list of numbers, and why that makes meaning searchable
percent: 15
duration: 7
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

An **embedding** is a list of numbers that represents the meaning of a piece of text. That is the whole idea. Everything else is detail.

Feed the sentence *"the robot's motors stalled on the carpet"* into an embedding model and you get back something like:

```
[0.021, -0.184, 0.067, 0.298, ..., -0.043]
```

384 numbers, in the model we will be using. Each one is a coordinate. Together they place that sentence at a specific point in a 384 dimensional space.

Now here is the useful bit. *"the drive train couldn't handle the deep pile rug"* lands at a **nearby point**, even though it shares almost no words with the first sentence. Meanwhile *"feed the sourdough starter every 12 hours"* lands miles away.

Similar meaning, nearby points. That is the entire trick.

---

## Thinking in fewer dimensions

384 dimensions is impossible to picture, so shrink it to two. Imagine a map where the horizontal axis is roughly "how mechanical is this" and the vertical axis is roughly "how much is this about food":

```
  food
    ^
    |    sourdough starter
    |    bread proving times
    |
    |
    +-------------------------------> mechanical
         motor stall torque
         gearbox ratios
```

Ask "why did my robot stop moving?" and it lands down in the bottom right, near the motor notes and nowhere near the bread. You did not use the word "motor" - it went there because of what the question *means*.

Real embedding models do this in hundreds of dimensions, and no single axis has a tidy human name. But the principle holds exactly.

---

## Measuring closeness

Once everything is a point, "most relevant" becomes "closest". There are a few ways to measure closeness, and the one almost everyone uses for text is **cosine distance**.

Cosine distance ignores how *long* the vectors are and only cares about the *direction* they point. That matters, because a three word note and a three paragraph note about the same topic should score as similar - and with cosine distance they do.

The numbers come back like this:

| Cosine distance | Direction of the two vectors |
|---|---|
| 0.0 | Identical - as close as it gets |
| 1.0 | At right angles - nothing in common |
| 2.0 | Exactly opposite |
{:class="table table-single"}

It is a *distance*, so **smaller is better**. This catches people out constantly, because "score" usually means bigger is better. Later on we will convert it with `1 - distance` and show it as a percentage, purely so the output reads naturally.

One warning before you build a mental rule of thumb: with a small model like the one we are using, real text rarely gets anywhere near 0.0. A genuinely excellent match often scores around 0.5, and an unrelated one around 0.95. Those numbers are not a bug and they are not something to fix. Judge results by their *rank*, never by an absolute cutoff.

---

## The model doing the work

ChromaDB ships with a default embedding model called **all-MiniLM-L6-v2**. It is small - about 80MB - runs on your CPU, and produces 384 dimensional vectors.

It is not the most accurate embedding model in the world. It is, however, free, fast, private, and downloads itself the first time you use it. For personal notes it is a genuinely good choice, and the pipeline we build will let you swap it later if you want.

Here is what it looks like in practice:

```python
# embed_demo.py - a scratch script, run on its own (not part of the tool we build later)
from chromadb.utils import embedding_functions

embed = embedding_functions.DefaultEmbeddingFunction()

vectors = embed([
    "The N20 motors stall on carpet",
    "The drive train can't cope with a thick rug",
    "Feed the sourdough starter every 12 hours",
])

print(f"Each vector has {len(vectors[0])} dimensions")
```

Run that and you get `Each vector has 384 dimensions`. The first time it runs it will pause to download the model.

---

## Seeing the distances for yourself

Cosine distance is easy to compute by hand, and doing it once makes the whole idea concrete:

```python
# cosine_demo.py - another scratch script
import math
from chromadb.utils import embedding_functions

embed = embedding_functions.DefaultEmbeddingFunction()


def cosine_distance(a, b):
    """1 minus the cosine of the angle between two vectors."""
    dot = sum(x * y for x, y in zip(a, b))
    length_a = math.sqrt(sum(x * x for x in a))
    length_b = math.sqrt(sum(y * y for y in b))
    return 1 - dot / (length_a * length_b)


sentences = [
    "The N20 motors stall on carpet",
    "The drive train can't cope with a thick rug",
    "Feed the sourdough starter every 12 hours",
]
vectors = embed(sentences)

question = embed(["why won't my robot move?"])[0]

for sentence, vector in zip(sentences, vectors):
    print(f"{cosine_distance(question, vector):.3f}  {sentence}")
```

Run it and you get something like:

```
0.701  The N20 motors stall on carpet
0.712  The drive train can't cope with a thick rug
0.969  Feed the sourdough starter every 12 hours
```

Both robot sentences beat the bread one comfortably, and the question contained none of the words "motor", "stall", "carpet" or "drive". That is semantic search working.

Look at those numbers again though. The best match scored **0.70**. If you had written a rule saying "only accept results below 0.3" you would have thrown away the correct answer. Rank, not threshold.

You will never write `cosine_distance` again after this lesson - ChromaDB does it for you, and does it far faster. But now you know exactly what it is doing.

---

## What embeddings are bad at

Being honest about the limits saves you a lot of debugging later:

- **Exact identifiers.** Searching for a part number like `TB6612FNG` or an error code is a job for keyword search. Embeddings blur precise strings into general "this looks like a part number" meaning. We handle this in lesson 12.
- **Negation.** "The motor is not a stepper" and "the motor is a stepper" embed almost identically. The model captures topic far better than it captures logic.
- **Very long text.** Embed a whole 3,000 word note as one vector and you get a mushy average of everything in it. That is precisely why we chunk - lesson 7.
- **Numbers and dates.** "50mA" and "500mA" are near neighbours as far as the model is concerned. Filter on real metadata instead of hoping the embedding catches it.

---

## Try it Yourself

1. Run the distance script above, then add a sentence of your own that is *about* robots but shares no vocabulary with the others. Where does it score?
2. Try two sentences that are opposites - "the battery is fully charged" and "the battery is completely flat". Are they close or far? Does that surprise you?
3. Embed the same sentence twice and compare the vectors. The distance should be exactly 0.0 - embedding is deterministic, which is what makes caching possible.
4. Time it: embed 100 short sentences and see how long it takes. That number tells you roughly how long indexing your whole vault will take.

---

## Common Issues

- **Problem**: The first call hangs for a minute with a download progress bar.
- **Solution**: That is the model downloading to `~/.cache/chroma/onnx_models/`. Let it finish; it only happens once.
- **Why**: ChromaDB fetches the ONNX build of all-MiniLM-L6-v2 on first use rather than bundling 80MB in the package.

- **Problem**: Two sentences that mean the same thing score 0.4 apart, which does not feel "similar".
- **Solution**: Do not judge distances against an absolute threshold. Compare them against each other.
- **Why**: What matters is ranking, not the raw number. A distance of 0.4 is excellent if everything else scored 0.9. Absolute thresholds are the single most common mistake in a first RAG build.

- **Problem**: Embedding is slow on a Raspberry Pi.
- **Solution**: Expect it. Index once and let it run; queries afterwards are fast because only the question needs embedding.
- **Why**: Indexing embeds every chunk in the vault. Querying embeds one short string.
