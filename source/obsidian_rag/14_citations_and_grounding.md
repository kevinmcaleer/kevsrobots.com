---
title: Citations and Staying Grounded
description: Make the answer traceable, verify the citations it produces, and get the model to admit when your notes do not know
layout: lesson
type: page
cover: /learn/obsidian_rag/assets/banners/14_citations_and_grounding.jpg
date_updated: 2026-08-23
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

A RAG tool that answers confidently but cannot show its working is worse than no tool at all - because you will believe it.

This lesson is about the difference between *sounding* grounded and *being* grounded.

## Why citations are the whole feature

When the answer says:

> You swapped the DRV8833 for a TB6612FNG because it was running hot [1], and the N20 motors still stalled on carpet afterwards [2].

...you can click through to `robots/SMARS.md` and `daily/2026-08-20.md` and check. Ten seconds, and you know whether to trust it.

Without those markers you have a paragraph that might be a faithful summary of your notes, might be the model's general knowledge about motor drivers, or might be a blend. There is no way to tell by reading it, and the blend is the dangerous one.

---

## Printing the source list

Our CLI prints the answer, then the sources it was given:

```python
# a preview of cmd_ask - it lands in vault_rag/cli.py in lesson 15
for piece in stream_answer(question, hits):
    print(piece, end="", flush=True)

print("\n\nSources:")
for number, hit in enumerate(hits, start=1):
    print(f"  [{number}] {hit.path}  ({hit.score:.0%})")
```

```
You swapped the DRV8833 for a TB6612FNG [1]. Your notes do not say why -
the daily note from that week only mentions the motors stalling on carpet [2].

Sources:
  [1] robots/SMARS.md  (52%)
  [2] Daily.md  (20%)
```

Note that we list **everything we sent**, not only what got cited. That is deliberate. Seeing that source 4 was retrieved and *not* used tells you as much as seeing the ones that were - it usually means retrieval pulled in something irrelevant, which is a signal worth having.

---

## Verifying the citations are real

The model can hallucinate a citation number. It is rare with a clear prompt, but "rare" is not "never", and this is cheap to check:

```python
# vault_rag/answer.py  (add `import re` at the top, then this at the bottom)
import re

CITATION_RE = re.compile(r"\[(\d+)\]")


def check_citations(text: str, hit_count: int) -> tuple[set[int], set[int]]:
    """Return (cited, invalid) source numbers found in the answer."""
    cited = {int(n) for n in CITATION_RE.findall(text)}
    invalid = {n for n in cited if n < 1 or n > hit_count}
    return cited - invalid, invalid
```

Wire it in after the answer arrives:

```python
# also destined for cmd_ask in vault_rag/cli.py (lesson 15)
cited, invalid = check_citations(answer_text, len(hits))

if invalid:
    print(f"\nWarning: answer cited sources that do not exist: {sorted(invalid)}")

unused = set(range(1, len(hits) + 1)) - cited
if unused:
    print(f"Retrieved but unused: {sorted(unused)}")
```

Two useful signals for a few lines of code:

- **Invalid citations** mean the model is drifting. If you see them regularly, your system prompt needs tightening.
- **Unused sources** are a retrieval quality metric. If half of what you retrieve is never cited, `TOP_K` is too high or your chunking needs work. Lesson 17 turns this into a proper measurement.

---

## Getting it to admit ignorance

This is the hard part. A language model's default behaviour is to be helpful, and "I don't know" does not feel helpful. You have to make it explicitly acceptable.

Three things that work, in order of effect:

**Give it a preferred alternative.** "Say so plainly and suggest what note the user might need to write" replaces the unhelpful answer with a different helpful one. The model has somewhere to go.

**Ban the fallback explicitly.** "Never invent a fact to fill the gap" is a direct instruction, and direct instructions land better than implied ones.

**Never call when there are no hits.** Our `answer()` returns early on an empty hit list. There is no such thing as a well-grounded answer with no grounding.

There is a fourth trick that is easy to miss: **let weak results through**. It is tempting to filter out low-scoring hits so the model only sees good material. But a genuinely weak result set is *information* - it means your notes probably do not cover this. Hand it over and let the model say "these excerpts are about motors generally, but nothing here mentions the stall current you are asking about". That is the right answer.

---

## Testing that it actually refuses

Do not assume it works. Test it:

```python
# try_it.py - a scratch script in the project root
questions = [
    "what did I write about the Voyager 2 mission?",      # not in the vault
    "what is the capital of France?",                      # general knowledge
    "what did I decide about the motor driver?",           # in the vault
]

for question in questions:
    hits = search(question, db_path="testdb")
    print(f"\n=== {question}")
    print(answer(question, hits))
```

You want the first two to decline, and the third to answer with citations. The second is the sharper test - the model absolutely knows the capital of France, and a properly grounded assistant will still say "your notes do not cover this".

If it answers "Paris", your system prompt is not strong enough. Make the ONLY line more emphatic and try again.

---

## A note on trust

Even a well-grounded answer can be wrong in a specific way: it can be a **faithful summary of a note that was wrong**. If you wrote "the DRV8833 handles 2A per channel" and that was a misreading of the datasheet, the tool will repeat it back to you with a citation, and the citation makes it feel more true.

RAG grounds answers in your notes. It does not grade your notes. That is a genuine limitation and worth holding in mind - the citation tells you *where a claim came from*, not that the claim is correct.

The practical upshot: use the citation. Click through. The point of the link is that you read it.

---

## Try it Yourself

1. Ask three questions your vault definitely cannot answer. Count how many get a straight "your notes do not cover this".
2. Ask about something you have exactly one note on, then read that note. Is the answer a fair representation of it?
3. Add `check_citations` to your flow and run ten real questions. Any invalid numbers? How many unused sources on average?
4. Deliberately put a wrong fact in a test note, then ask about it. Watch the tool confidently repeat your mistake, with a citation. Sit with that for a moment - it is the most important thing to understand about RAG.
5. Try dropping "Never invent a fact to fill the gap" and re-run the Voyager 2 question. Does it hold the line without that sentence?

---

## Common Issues

- **Problem**: The answer says "based on the provided sources..." in every reply.
- **Solution**: Add "Do not mention the sources as a concept - just cite them" to the system prompt.
- **Why**: Instructions about sources make the model self-conscious about sources. Naming the unwanted phrasing fixes it.

- **Problem**: It cites `[1]` for everything, including claims from source 3.
- **Solution**: Check `format_sources` really is numbering from 1 and the ids appear in the prompt.
- **Why**: If the numbering is unclear the model falls back to citing the first source for everything.

- **Problem**: It refuses to answer questions your notes *do* cover.
- **Solution**: Print the prompt. The answer is probably not in the retrieved chunks.
- **Why**: Over-refusal is nearly always a retrieval failure wearing a generation costume. Fix the retrieval.

- **Problem**: Citations appear as `(1)` or `Source 1` instead of `[1]`.
- **Solution**: Show the exact format you want in the system prompt - we write `like [1] or [2][3]`.
- **Why**: Describing a format in words gets approximate compliance. Showing an example gets exact compliance, and your `CITATION_RE` depends on exact.
