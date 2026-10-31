"""Grammar, Look Up, Synonyms and Translate for the project editor.

The editor (markdown-studio's Markdown editor, vendored in
web/assets/js/markdown-editor/) checks spelling in the browser, so that works
with no server. It asks this service for the rest, using the same endpoints
as Markdown Studio's own server (``proofing/languageApi.ts``):

- ``GET  /api/language``                      what's on here
- ``GET  /api/language/define?word=…``        WordNet senses, with synonyms (the thesaurus)
- ``POST /api/language/check``                grammar, proxied to LanguageTool
- ``GET  /api/language/translate/languages``  LibreTranslate's languages (off by default)
- ``POST /api/language/translate``            a translation (off by default)
- ``POST /api/language/rewrite``              one sentence rewritten by Claude (off by default)

Every endpoint needs a signed-in user and is rate limited per user, so the
service can't be used as an open LanguageTool proxy.
"""

from __future__ import annotations

import json
import logging
import time
from typing import Any

import httpx
from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel, Field

from .auth import get_current_user
from .config import Settings
from .ratelimit import RateLimiter
from .rewrite import MAX_SENTENCE, RewriteRequest, client_for, rewrite
from .wordnet import WordNet

log = logging.getLogger(__name__)

LANGUAGES = {"en-GB", "en-US"}
MAX_CHECK = 20_000
MAX_TRANSLATE = 5_000
TIMEOUT = 20.0
# LanguageTool's spelling is left to the editor's own (Hunspell) check, and its
# style hints to the writing hints, so the two don't underline the same words.
DISABLED_CATEGORIES = "TYPOS,STYLE"
SKIPPED_ISSUES = {"misspelling", "style"}


class Segment(BaseModel):
    text: str | None = None
    markup: str | None = None
    interpretAs: str | None = None  # noqa: N815 (LanguageTool's own name)


class CheckRequest(BaseModel):
    language: str = "en-GB"
    # Prose and Markdown in order, so offsets in the answer match the text.
    annotation: list[Segment] = Field(default_factory=list)


class TranslateRequest(BaseModel):
    text: str
    target: str
    source: str = "auto"


class Language:
    """The services this server has, and the clients that reach them."""

    def __init__(
        self,
        settings: Settings,
        transport: httpx.AsyncBaseTransport | None = None,
        claude: Any = None,
    ):
        self.settings = settings
        self.languagetool_url = settings.languagetool_url.rstrip("/")
        self.libretranslate_url = settings.libretranslate_url.rstrip("/")
        self.libretranslate_key = settings.libretranslate_api_key
        self.wordnet = WordNet.open(settings.wordnet_dir)
        self.transport = transport
        self.limiter = RateLimiter()
        # An Anthropic client for rewrites; tests pass a stand-in.
        self.claude = claude if claude is not None else client_for(settings.anthropic_api_key)
        self._languages: tuple[float, list[dict]] | None = None
        if settings.wordnet_dir and self.wordnet is None:
            log.warning("WordNet isn't at %s, so Look Up is off", settings.wordnet_dir)

    def client(self) -> httpx.AsyncClient:
        return httpx.AsyncClient(timeout=TIMEOUT, transport=self.transport)

    def features(self) -> dict[str, bool]:
        return {
            "lookup": self.wordnet is not None,
            "grammar": bool(self.languagetool_url),
            "translate": bool(self.libretranslate_url),
            "rewrite": self.claude is not None,
        }

    async def check(self, body: CheckRequest) -> list[dict]:
        data = {"annotation": [s.model_dump(exclude_none=True) for s in body.annotation]}
        async with self.client() as client:
            response = await client.post(
                f"{self.languagetool_url}/v2/check",
                data={
                    "language": body.language,
                    "data": json.dumps(data),
                    "disabledCategories": DISABLED_CATEGORIES,
                },
            )
        response.raise_for_status()
        matches = []
        for m in response.json().get("matches", []):
            rule = m.get("rule") or {}
            if rule.get("issueType") in SKIPPED_ISSUES:
                continue
            matches.append(
                {
                    "offset": m["offset"],
                    "length": m["length"],
                    "message": m.get("message", ""),
                    "short": m.get("shortMessage", ""),
                    "replacements": [r["value"] for r in m.get("replacements", [])[:5]],
                    "rule": rule.get("id", ""),
                }
            )
        return matches

    async def languages(self) -> list[dict]:
        # They only change when the LibreTranslate server's models do.
        if self._languages and time.monotonic() - self._languages[0] < 3600:
            return self._languages[1]
        async with self.client() as client:
            response = await client.get(f"{self.libretranslate_url}/languages")
        response.raise_for_status()
        languages = [{"code": lang["code"], "name": lang["name"]} for lang in response.json()]
        self._languages = (time.monotonic(), languages)
        return languages

    async def translate(self, body: TranslateRequest) -> dict:
        payload: dict[str, Any] = {
            "q": body.text,
            "source": body.source,
            "target": body.target,
            "format": "text",
        }
        if self.libretranslate_key:
            payload["api_key"] = self.libretranslate_key
        async with self.client() as client:
            response = await client.post(f"{self.libretranslate_url}/translate", json=payload)
        if response.status_code == 400:
            detail = response.json().get("error", "That translation isn't available.")
            raise HTTPException(status_code=400, detail=detail)
        response.raise_for_status()
        data = response.json()
        detected = data.get("detectedLanguage") or {}
        return {"text": data.get("translatedText", ""), "detected": detected.get("language")}


def make_router(language: Language) -> APIRouter:
    router = APIRouter(tags=["language"])
    settings = language.settings

    def unavailable(name: str) -> HTTPException:
        return HTTPException(status_code=503, detail=f"{name} isn't set up on this server.")

    async def upstream(call, name: str):
        try:
            return await call
        except httpx.HTTPError as exc:
            log.warning("%s request failed: %s", name, exc)
            raise HTTPException(status_code=502, detail=f"Couldn't reach the {name} server.") from exc

    @router.get("/api/language")
    def features(user: str = Depends(get_current_user)):
        """What the editor can offer here, besides spelling (which is always on)."""
        return language.features()

    @router.get("/api/language/define")
    def define(word: str, user: str = Depends(get_current_user)):
        if language.wordnet is None:
            raise unavailable("The dictionary")
        language.limiter.hit(user, "lookup", settings.lookup_per_minute)
        entry = language.wordnet.define(word)
        if entry is None:
            raise HTTPException(status_code=404, detail="No definition found.")
        return entry

    @router.post("/api/language/check")
    async def check(body: CheckRequest, user: str = Depends(get_current_user)):
        if not language.languagetool_url:
            raise unavailable("Grammar checking")
        if body.language not in LANGUAGES:
            raise HTTPException(status_code=400, detail="Unsupported language.")
        size = sum(len(s.text or "") + len(s.markup or "") for s in body.annotation)
        if size > MAX_CHECK:
            raise HTTPException(status_code=400, detail="That's too much text to check at once.")
        language.limiter.hit(user, "check", settings.check_per_minute)
        return {"matches": await upstream(language.check(body), "grammar")}

    @router.get("/api/language/translate/languages")
    async def languages(user: str = Depends(get_current_user)):
        if not language.libretranslate_url:
            raise unavailable("Translation")
        return {"languages": await upstream(language.languages(), "translation")}

    @router.post("/api/language/translate")
    async def translate(body: TranslateRequest, user: str = Depends(get_current_user)):
        if not language.libretranslate_url:
            raise unavailable("Translation")
        if not body.text.strip():
            return {"text": "", "detected": None}
        if len(body.text) > MAX_TRANSLATE:
            raise HTTPException(status_code=400, detail="That's too much text to translate at once.")
        language.limiter.hit(user, "translate", settings.translate_per_minute)
        return await upstream(language.translate(body), "translation")

    @router.post("/api/language/rewrite")
    async def rewrite_sentence(body: RewriteRequest, user: str = Depends(get_current_user)):
        """One sentence in the active voice. The editor shows it before changing anything."""
        if language.claude is None:
            raise unavailable("Rewriting")
        if body.language not in LANGUAGES:
            raise HTTPException(status_code=400, detail="Unsupported language.")
        if not body.text.strip():
            raise HTTPException(status_code=400, detail="There's no sentence to rewrite.")
        if len(body.text) > MAX_SENTENCE:
            raise HTTPException(status_code=400, detail="That sentence is too long to rewrite.")
        language.limiter.hit(user, "rewrite", settings.rewrite_per_minute)
        return {"text": await rewrite(language.claude, settings.rewrite_model, body)}

    return router
