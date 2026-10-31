"""Shared pytest fixtures: a tiny WordNet, a fake LanguageTool, and tokens."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import httpx
import pytest
import pytest_asyncio
from jose import jwt

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

SECRET = "test-secret"

LICENCE = "  1 This software and database is being provided to you\n  2 under the WordNet licence.\n"

# word -> [(gloss, [synonyms])] for each part of speech; a tiny WordNet.
TINY = {
    "noun": {
        "dog": [
            ('a domesticated canid; "the dog barked all night"', ["domestic_dog"]),
            ("a dull unattractive unpleasant girl or woman", ["frump"]),
        ],
    },
    "verb": {"build": [("make by combining materials and parts", ["construct", "make"])]},
    "adj": {"quick": [("accomplished rapidly", ["speedy", "fast"])]},
    "adv": {},
}


def write_wordnet(directory: Path) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    for pos, words in TINY.items():
        data = LICENCE.encode()
        index_lines = []
        for lemma in sorted(words):
            offsets = []
            for gloss, synonyms in words[lemma]:
                offset = len(data)
                members = [lemma, *synonyms]
                head = f"{offset:08d} 03 n {len(members):02x} " + " ".join(f"{m} 0" for m in members)
                data += f"{head} 000 | {gloss}  \n".encode()
                offsets.append(offset)
            joined = " ".join(f"{o:08d}" for o in offsets)
            index_lines.append(f"{lemma} n {len(offsets)} 1 @ {len(offsets)} 0 {joined}  \n")
        (directory / f"data.{pos}").write_bytes(data)
        (directory / f"index.{pos}").write_text(LICENCE + "".join(index_lines))
    return directory


def token(sub: str = "kev", secret: str = SECRET) -> str:
    return jwt.encode({"sub": sub}, secret, algorithm="HS256")


class FakeUpstream:
    """Stands in for LanguageTool and LibreTranslate, and records what it was sent."""

    def __init__(self):
        self.requests: list[httpx.Request] = []
        self.down = False

    def __call__(self, request: httpx.Request) -> httpx.Response:
        self.requests.append(request)
        if self.down:
            raise httpx.ConnectError("refused", request=request)
        if request.url.path == "/v2/check":
            return httpx.Response(
                200,
                json={
                    "matches": [
                        {
                            "offset": 0,
                            "length": 4,
                            "message": "Did you mean “These”?",
                            "shortMessage": "Agreement",
                            "replacements": [{"value": "These"}],
                            "rule": {"id": "THIS_NNS", "issueType": "grammar"},
                        },
                        {
                            "offset": 5,
                            "length": 4,
                            "message": "Possible spelling mistake",
                            "replacements": [],
                            "rule": {"id": "MORFOLOGIK", "issueType": "misspelling"},
                        },
                    ]
                },
            )
        if request.url.path == "/languages":
            return httpx.Response(200, json=[{"code": "fr", "name": "French"}])
        if request.url.path == "/translate":
            body = json.loads(request.content)
            return httpx.Response(200, json={"translatedText": f"[{body['target']}] {body['q']}", "detectedLanguage": {"language": "en"}})
        return httpx.Response(404)


@pytest.fixture
def upstream() -> FakeUpstream:
    return FakeUpstream()


@pytest.fixture
def env(monkeypatch: pytest.MonkeyPatch, tmp_path: Path):
    """Sets the environment; a test can change it before asking for ``client``."""
    monkeypatch.setenv("JWT_SECRET", SECRET)
    monkeypatch.setenv("LANGUAGETOOL_URL", "http://languagetool:8010")
    monkeypatch.setenv("LIBRETRANSLATE_URL", "")
    monkeypatch.setenv("WORDNET_DIR", str(write_wordnet(tmp_path / "wordnet")))
    monkeypatch.setenv("CHECK_PER_MINUTE", "30")
    monkeypatch.delenv("ANTHROPIC_API_KEY", raising=False)
    return monkeypatch


@pytest_asyncio.fixture
async def client(env, upstream):
    from language_tools.config import get_settings
    from language_tools.main import create_app

    get_settings.cache_clear()
    app = create_app(httpx.MockTransport(upstream))
    async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://test") as c:
        yield c
    get_settings.cache_clear()
