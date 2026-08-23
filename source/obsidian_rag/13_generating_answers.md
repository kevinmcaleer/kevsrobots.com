---
title: Asking Claude
description: Turn retrieved chunks into a grounded prompt, call the Messages API, and stream the answer back
layout: lesson
type: page
cover: /learn/obsidian_rag/assets/banners/13_generating_answers.jpg
date_updated: 2026-08-23
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Half the pipeline is done. We can find the right paragraphs. Now let's turn them into an answer.

## Getting a key

Head to [console.anthropic.com](https://console.anthropic.com), create an API key, and put it in your environment:

```bash
export ANTHROPIC_API_KEY="sk-ant-..."
```

Put that in your shell profile so it persists. Never put it in `config.py` - that file will end up in a git repo eventually, and keys in git repos get scraped within minutes.

The SDK picks the key up from the environment automatically, which is why the client below takes no arguments.

---

## The shape of a grounded prompt

There are three moving parts, and each has a job:

| Part | Job |
|---|---|
| System prompt | The rules. Use only these sources, cite them, admit ignorance |
| Sources | The retrieved chunks, numbered so they can be referenced |
| Question | What the user actually asked |
{:class="table table-single"}

Order matters. Sources go **before** the question, because a model reading a question first and evidence second tends to answer from what it already knows and then look for support. Evidence first, question last, produces answers that stay closer to the sources.

---

## The system prompt

```python
# vault_rag/answer.py
"""Ask Claude a question using only what we retrieved from the vault."""

from .config import ANSWER_MODEL, MAX_ANSWER_TOKENS
from .retrieve import Hit

SYSTEM_PROMPT = """You are a research assistant for Kevin's personal Obsidian vault.

Rules:
- Answer using ONLY the numbered sources provided. They are the user's own notes.
- Cite the sources you used inline, like [1] or [2][3], right after the claim.
- If the sources do not answer the question, say so plainly and suggest what
  note the user might need to write. Never invent a fact to fill the gap.
- Match the user's own vocabulary. These are their notes, not a textbook.
- Be concise. Two or three short paragraphs is usually plenty."""
```

Every line of that is load bearing.

**"ONLY the numbered sources"** is the whole point of RAG. Without it you get a blend of your notes and the model's general knowledge, and you cannot tell which is which. That blend is far more dangerous than a plain wrong answer, because it reads as though it came from your notes.

**"Cite the sources"** does more than produce nice output. Asking for a citation after each claim makes the model check that each claim actually has a source - and when it cannot find one, it tends not to make the claim.

**"Suggest what note the user might need to write"** turns a dead end into something useful. "Your notes cover the motor wiring but not why you chose the DRV8833" is a genuinely helpful answer to a failed search.

**"Match the user's own vocabulary"** matters more than it sounds. If you call it "the chassis" the answer should say chassis, not "the structural frame assembly".

---

## Formatting the sources

```python
def format_sources(hits: list[Hit]) -> str:
    """Number the retrieved chunks so the model can cite them."""
    blocks = []
    for number, hit in enumerate(hits, start=1):
        blocks.append(
            f"<source id=\"{number}\" note=\"{hit.path}\" section=\"{hit.label}\">\n"
            f"{hit.text}\n"
            f"</source>"
        )
    return "\n\n".join(blocks)


def build_prompt(question: str, hits: list[Hit]) -> str:
    """Assemble the user turn: sources first, question last."""
    return (
        "Here are the most relevant excerpts from my notes:\n\n"
        f"{format_sources(hits)}\n\n"
        f"Question: {question}"
    )
```

The XML-ish tags are deliberate. Claude handles clearly delimited structure well, and unambiguous boundaries matter here because the chunk text is *markdown* - full of headings and code fences that would otherwise blur into the surrounding prompt.

Carrying `note` and `section` as attributes gives the model the citation information without you having to parse anything out of its reply afterwards.

Here is what actually goes over the wire:

```
Here are the most relevant excerpts from my notes:

<source id="1" note="robots/SMARS.md" section="SMARS Robot > SMARS">
SMARS Robot > SMARS

Now a much shorter note. The DRV8833 driver was swapped for a TB6612FNG.
</source>

<source id="2" note="Daily.md" section="Daily > 2026-08-20">
Daily > 2026-08-20

Ordered more PETG. Tested the [[SMARS]] on carpet - the N20 motors stall.
</source>

Question: what motor driver does SMARS use?
```

Readable, unambiguous, and small. That whole prompt is a few hundred tokens rather than a whole vault.

---

## Making the call

```python
def answer(question: str, hits: list[Hit], client=None, model: str = ANSWER_MODEL) -> str:
    """Send the grounded prompt to Claude and return the text of the reply."""
    if not hits:
        return "I could not find anything in your vault about that."

    if client is None:
        import anthropic
        client = anthropic.Anthropic()

    response = client.messages.create(
        model=model,
        max_tokens=MAX_ANSWER_TOKENS,
        system=SYSTEM_PROMPT,
        messages=[{"role": "user", "content": build_prompt(question, hits)}],
    )

    return "".join(block.text for block in response.content if block.type == "text")
```

Points worth pausing on:

**No hits means no call.** If retrieval found nothing there is nothing to ground an answer in, and asking anyway is exactly how you get a confident fabrication. Fail early and cheaply.

**`client=None` with lazy import.** This makes the function testable without an API key - pass a fake client and assert on the prompt. It also means `import anthropic` only happens when you actually call out, so `amv search` starts fast.

**`response.content` is a list of blocks, not a string.** Checking `block.type == "text"` and joining is the correct way to read a reply. Reaching straight for `response.content[0].text` works right up until the day the first block is something else.

**The model.** `claude-opus-5` is set in `config.py`. For a tool you run a few times a day the cost is small, and this is the step where quality shows.

---

## Streaming

A grounded answer takes a few seconds. Watching a blank terminal for a few seconds feels much longer than watching words appear, so stream it:

```python
def stream_answer(question: str, hits: list[Hit], client=None, model: str = ANSWER_MODEL):
    """Same thing, but yield the answer as it is generated."""
    if not hits:
        yield "I could not find anything in your vault about that."
        return

    if client is None:
        import anthropic
        client = anthropic.Anthropic()

    with client.messages.stream(
        model=model,
        max_tokens=MAX_ANSWER_TOKENS,
        system=SYSTEM_PROMPT,
        messages=[{"role": "user", "content": build_prompt(question, hits)}],
    ) as stream:
        yield from stream.text_stream
```

`stream.text_stream` yields text as it arrives, and the context manager cleans up the connection. Printing it is a one-liner:

```python
for piece in stream_answer(question, hits):
    print(piece, end="", flush=True)
```

`flush=True` is not optional - without it Python buffers the output and you get the whole answer in one lump at the end, which defeats the point entirely.

---

## Testing without spending anything

You do not need an API key to test the prompt building, and you should not need one to run your tests:

```python
from types import SimpleNamespace
from vault_rag.answer import answer
from vault_rag.retrieve import search


class FakeMessages:
    def create(self, **kwargs):
        assert kwargs["model"] == "claude-opus-5"
        assert "<source id=\"1\"" in kwargs["messages"][0]["content"]
        return SimpleNamespace(
            content=[SimpleNamespace(type="text", text="A grounded answer [1].")]
        )


fake = SimpleNamespace(messages=FakeMessages())
hits = search("what motor driver does SMARS use?", db_path="testdb", top_k=3)
print(answer("what motor driver does SMARS use?", hits, client=fake))
```

That runs in milliseconds, costs nothing, and catches the mistakes you actually make - a malformed prompt, a missing source block, sources in the wrong order.

---

## Try it Yourself

1. Print `build_prompt(...)` for a real question and read it as though you were the model. Is the answer findable in there? If not, this is a retrieval problem and no prompt will save it.
2. Ask something your notes definitely do not cover. Does it say so, or does it guess? If it guesses, tighten the system prompt.
3. Remove the "ONLY the numbered sources" line and ask a question about a well known topic you have *not* written about. Watch general knowledge leak in. Put the line back.
4. Count tokens: roughly `len(prompt) / 4`. Six chunks of 1200 characters is around 2,000 tokens - pennies per question.
5. Ask the same question with `TOP_K=3` and `TOP_K=10`. More context is not automatically better.

---

## Common Issues

- **Problem**: `anthropic.AuthenticationError: invalid x-api-key`
- **Solution**: Check `echo $ANTHROPIC_API_KEY` in the same shell you are running from.
- **Why**: Exporting a variable in one terminal does not affect another. Put it in your shell profile.

- **Problem**: The answer cites sources that do not exist, like `[7]` when you sent 6.
- **Solution**: Confirm your numbering starts at 1 - `enumerate(hits, start=1)`.
- **Why**: If the sources are numbered from 0 the model often "corrects" them to 1-based in its head and the citations no longer line up with your list.

- **Problem**: The answer is a wall of text despite asking for concise.
- **Solution**: Be specific in the system prompt. "Two or three short paragraphs" works far better than "be brief".
- **Why**: Vague style instructions get vague compliance. Give it a number.

- **Problem**: `AttributeError: 'ToolUseBlock' object has no attribute 'text'`
- **Solution**: Filter on `block.type == "text"` before reading `.text`.
- **Why**: `response.content` can contain several block types. Only text blocks have text.
