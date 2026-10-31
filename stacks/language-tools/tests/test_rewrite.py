from __future__ import annotations

from types import SimpleNamespace

import anthropic
import httpx
import pytest

from .conftest import token

AUTH = {"Authorization": f"Bearer {token()}"}
SENTENCE = {"text": "The current information is presented badly.", "phrase": "is presented"}


class FakeClaude:
    """Stands in for anthropic.AsyncAnthropic and records each request."""

    def __init__(self, reply="I present the current information badly.", stop_reason="end_turn", error=None):
        self.calls: list[dict] = []
        self.reply = reply
        self.stop_reason = stop_reason
        self.error = error
        self.beta = SimpleNamespace(messages=SimpleNamespace(create=self.create))

    async def create(self, **kwargs):
        self.calls.append(kwargs)
        if self.error:
            raise self.error
        return SimpleNamespace(
            stop_reason=self.stop_reason,
            content=[SimpleNamespace(type="thinking", thinking=""), SimpleNamespace(type="text", text=self.reply)],
        )


@pytest.fixture
def make_client(env):
    from language_tools.config import get_settings
    from language_tools.main import create_app

    def make(claude=None):
        get_settings.cache_clear()
        app = create_app(httpx.MockTransport(lambda r: httpx.Response(404)), claude)
        return httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://test")

    yield make
    get_settings.cache_clear()


async def test_rewrite_is_off_without_a_key(client):
    response = await client.post("/api/language/rewrite", json=SENTENCE, headers=AUTH)
    assert response.status_code == 503
    assert (await client.get("/api/language", headers=AUTH)).json()["rewrite"] is False


async def test_the_key_turns_rewrite_on(env, make_client):
    env.setenv("ANTHROPIC_API_KEY", "sk-test")
    async with make_client() as c:
        assert (await c.get("/api/language", headers=AUTH)).json()["rewrite"] is True


async def test_rewrite_asks_claude_for_the_active_voice(make_client):
    claude = FakeClaude()
    async with make_client(claude) as c:
        response = await c.post("/api/language/rewrite", json=SENTENCE, headers=AUTH)
        assert (await c.get("/api/language", headers=AUTH)).json()["rewrite"] is True
    assert response.status_code == 200
    assert response.json() == {"text": "I present the current information badly."}
    call = claude.calls[0]
    assert call["model"] == "claude-opus-5-5"
    assert call["fallbacks"] == "default"
    assert "server-side-fallback-2026-07-01" in call["betas"]
    assert "British English" in call["system"]
    prompt = call["messages"][0]["content"]
    assert "<sentence>The current information is presented badly.</sentence>" in prompt
    assert "is presented" in prompt


async def test_rewrite_uses_american_spelling_and_the_model_setting(env, make_client):
    env.setenv("REWRITE_MODEL", "claude-sonnet-5-5")
    claude = FakeClaude()
    async with make_client(claude) as c:
        await c.post("/api/language/rewrite", json={**SENTENCE, "language": "en-US"}, headers=AUTH)
    assert claude.calls[0]["model"] == "claude-sonnet-5-5"
    assert "American English" in claude.calls[0]["system"]


async def test_rewrite_strips_wrapping_quotes(make_client):
    async with make_client(FakeClaude(reply="“I wrote it.”")) as c:
        response = await c.post("/api/language/rewrite", json=SENTENCE, headers=AUTH)
    assert response.json() == {"text": "I wrote it."}


async def test_rewrite_refuses_empty_long_and_other_languages(make_client):
    claude = FakeClaude()
    async with make_client(claude) as c:
        assert (await c.post("/api/language/rewrite", json={"text": "  "}, headers=AUTH)).status_code == 400
        assert (await c.post("/api/language/rewrite", json={"text": "x" * 1001}, headers=AUTH)).status_code == 400
        assert (await c.post("/api/language/rewrite", json={**SENTENCE, "language": "fr"}, headers=AUTH)).status_code == 400
        assert (await c.post("/api/language/rewrite", json={**SENTENCE, "style": "shorter"}, headers=AUTH)).status_code == 422
    assert claude.calls == []


async def test_a_refusal_is_a_422(make_client):
    async with make_client(FakeClaude(stop_reason="refusal", reply="")) as c:
        response = await c.post("/api/language/rewrite", json=SENTENCE, headers=AUTH)
    assert response.status_code == 422


async def test_an_api_error_is_a_502(make_client):
    request = httpx.Request("POST", "https://api.anthropic.com/v1/messages")
    error = anthropic.APIConnectionError(request=request)
    async with make_client(FakeClaude(error=error)) as c:
        response = await c.post("/api/language/rewrite", json=SENTENCE, headers=AUTH)
    assert response.status_code == 502


async def test_rewrite_is_rate_limited(env, make_client):
    env.setenv("REWRITE_PER_MINUTE", "1")
    async with make_client(FakeClaude()) as c:
        assert (await c.post("/api/language/rewrite", json=SENTENCE, headers=AUTH)).status_code == 200
        assert (await c.post("/api/language/rewrite", json=SENTENCE, headers=AUTH)).status_code == 429
