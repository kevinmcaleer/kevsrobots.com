"""Rewrite one sentence with Claude: "Rewrite as active" on the passive-voice hint.

The editor sends the sentence the hint is in, plus the passive phrase it
found. Claude answers with the sentence in the active voice, and the editor
shows it beside the original before anything changes. Off unless
``ANTHROPIC_API_KEY`` is set.
"""

from __future__ import annotations

import logging
from typing import Any, Literal

import anthropic
from fastapi import HTTPException
from pydantic import BaseModel

log = logging.getLogger(__name__)

MAX_SENTENCE = 1_000
# Fallbacks route a declined request to another model by its refusal category,
# so a harmless sentence that trips a classifier still gets an answer.
FALLBACK_BETA = "server-side-fallback-2026-07-01"

SYSTEM = (
    "You edit sentences from makers' robot and electronics project write-ups on kevsrobots.com. "
    "You will be given one sentence that uses the passive voice. Rewrite it in the active voice "
    "so it says who does what.\n\n"
    "- Keep the meaning, tense, tone and roughly the length. Change as little as you need to.\n"
    "- When the sentence doesn't say who does it, pick the natural subject from the writer's "
    "point of view (I, we or you, matching the sentence) rather than inventing a person.\n"
    "- Keep any Markdown inside the sentence (links, **bold**, `code`) exactly as written.\n"
    "- Use {spelling} spelling.\n"
    "- If the sentence can't be made active without changing its meaning, return it unchanged.\n\n"
    "Reply with the rewritten sentence only: no quotes, no explanation."
)

SPELLING = {"en-GB": "British English", "en-US": "American English"}


class RewriteRequest(BaseModel):
    text: str
    # The passive phrase the hint underlined, e.g. "is presented".
    phrase: str = ""
    style: Literal["active"] = "active"
    language: str = "en-GB"


def client_for(api_key: str) -> Any:
    return anthropic.AsyncAnthropic(api_key=api_key) if api_key else None


async def rewrite(client: Any, model: str, body: RewriteRequest) -> str:
    """The sentence in the active voice, or a 4xx/5xx HTTPException."""
    prompt = f"<sentence>{body.text.strip()}</sentence>"
    if body.phrase.strip():
        prompt += f"\nThe passive phrase is: {body.phrase.strip()}"
    try:
        message = await client.beta.messages.create(
            model=model,
            max_tokens=4000,
            # One sentence doesn't need deep thought; low keeps it quick.
            output_config={"effort": "low"},
            system=SYSTEM.format(spelling=SPELLING.get(body.language, "British English")),
            messages=[{"role": "user", "content": prompt}],
            betas=[FALLBACK_BETA],
            fallbacks="default",
        )
    except anthropic.RateLimitError as exc:
        raise HTTPException(status_code=429, detail="The rewriter is busy. Try again in a moment.") from exc
    except anthropic.APIStatusError as exc:
        log.warning("Claude rewrite failed: %s %s", exc.status_code, exc.message)
        raise HTTPException(status_code=502, detail="Couldn't reach the rewriter.") from exc
    except anthropic.APIConnectionError as exc:
        log.warning("Claude rewrite failed: %s", exc)
        raise HTTPException(status_code=502, detail="Couldn't reach the rewriter.") from exc

    if message.stop_reason == "refusal":
        raise HTTPException(status_code=422, detail="Couldn't suggest a rewrite for that sentence.")
    text = "".join(block.text for block in message.content if block.type == "text").strip()
    # Belt and braces: the prompt asks for no quotes, but strip a wrapping pair.
    for left, right in (('"', '"'), ("“", "”")):
        inner = text[1:-1]
        if len(text) > 1 and text[0] == left and text[-1] == right and left not in inner and right not in inner:
            text = inner.strip()
    if not text:
        raise HTTPException(status_code=502, detail="The rewriter didn't suggest anything.")
    return text
