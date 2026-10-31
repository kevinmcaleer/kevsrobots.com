from __future__ import annotations

from urllib.parse import parse_qs

import pytest

from .conftest import token
from language_tools.ratelimit import RateLimiter

AUTH = {"Authorization": f"Bearer {token()}"}
CHECK = {"language": "en-GB", "annotation": [{"text": "This bots "}, {"markup": "**"}, {"text": "run"}, {"markup": "**"}]}


# ------------------------------------------------------------ sign-in


@pytest.mark.parametrize(
    "method,path,body",
    [
        ("GET", "/api/language", None),
        ("GET", "/api/language/define?word=dog", None),
        ("POST", "/api/language/check", CHECK),
        ("POST", "/api/language/translate", {"text": "hi", "target": "fr"}),
        ("POST", "/api/language/rewrite", {"text": "It was written by Kev."}),
    ],
)
async def test_every_endpoint_needs_a_session(client, upstream, method, path, body):
    response = await client.request(method, path, json=body)
    assert response.status_code == 401
    bad = await client.request(method, path, json=body, headers={"Authorization": f"Bearer {token(secret='nope')}"})
    assert bad.status_code == 401
    assert upstream.requests == []


async def test_the_session_cookie_works(client):
    client.cookies.set("access_token", token())
    response = await client.get("/api/language")
    assert response.status_code == 200


async def test_health_is_public(client):
    response = await client.get("/health")
    assert response.json() == {"status": "ok", "lookup": True, "grammar": True, "translate": False, "rewrite": False}


async def test_features(client):
    response = await client.get("/api/language", headers=AUTH)
    assert response.json() == {"lookup": True, "grammar": True, "translate": False, "rewrite": False}


async def test_cors_allows_the_site_with_credentials(client):
    response = await client.options(
        "/api/language/check",
        headers={"Origin": "https://www.kevsrobots.com", "Access-Control-Request-Method": "POST", "Access-Control-Request-Headers": "content-type"},
    )
    assert response.headers["access-control-allow-origin"] == "https://www.kevsrobots.com"
    assert response.headers["access-control-allow-credentials"] == "true"


# ------------------------------------------------------------ Look Up and Synonyms


async def test_define_gives_senses_and_synonyms(client):
    response = await client.get("/api/language/define", params={"word": "Dogs"}, headers=AUTH)
    assert response.status_code == 200
    entry = response.json()
    assert entry["lemma"] == "dog"
    senses = entry["entries"][0]["senses"]
    assert senses[0]["definition"] == "a domesticated canid"
    assert senses[0]["examples"] == ["the dog barked all night"]
    assert senses[0]["synonyms"] == ["domestic dog"]


async def test_define_finds_a_verbs_base_form(client):
    response = await client.get("/api/language/define", params={"word": "building"}, headers=AUTH)
    assert response.json()["entries"][0]["senses"][0]["synonyms"] == ["construct", "make"]


async def test_define_404s_for_an_unknown_word(client):
    response = await client.get("/api/language/define", params={"word": "zzyzx"}, headers=AUTH)
    assert response.status_code == 404


async def test_lookup_is_off_without_wordnet(env, tmp_path):
    env.setenv("WORDNET_DIR", str(tmp_path / "missing"))
    from .conftest import FakeUpstream
    import httpx
    from language_tools.config import get_settings
    from language_tools.main import create_app

    get_settings.cache_clear()
    app = create_app(httpx.MockTransport(FakeUpstream()))
    async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://test") as c:
        assert (await c.get("/api/language", headers=AUTH)).json()["lookup"] is False
        assert (await c.get("/api/language/define?word=dog", headers=AUTH)).status_code == 503
    get_settings.cache_clear()


# ------------------------------------------------------------ grammar


async def test_check_proxies_languagetool_and_drops_its_spelling(client, upstream):
    response = await client.post("/api/language/check", json=CHECK, headers=AUTH)
    assert response.status_code == 200
    assert response.json() == {
        "matches": [{"offset": 0, "length": 4, "message": "Did you mean “These”?", "short": "Agreement", "replacements": ["These"], "rule": "THIS_NNS"}]
    }
    sent = parse_qs(upstream.requests[0].content.decode())
    assert str(upstream.requests[0].url) == "http://languagetool:8010/v2/check"
    assert sent["language"] == ["en-GB"]
    assert sent["disabledCategories"] == ["TYPOS,STYLE"]
    assert '"markup": "**"' in sent["data"][0]


async def test_check_refuses_other_languages_and_huge_text(client, upstream):
    assert (await client.post("/api/language/check", json={**CHECK, "language": "xx"}, headers=AUTH)).status_code == 400
    huge = {"language": "en-GB", "annotation": [{"text": "a" * 20_001}]}
    assert (await client.post("/api/language/check", json=huge, headers=AUTH)).status_code == 400
    assert upstream.requests == []


async def test_check_502s_when_languagetool_is_down(client, upstream):
    upstream.down = True
    response = await client.post("/api/language/check", json=CHECK, headers=AUTH)
    assert response.status_code == 502


async def test_check_is_rate_limited_per_user(env, upstream):
    env.setenv("CHECK_PER_MINUTE", "2")
    import httpx
    from language_tools.config import get_settings
    from language_tools.main import create_app

    get_settings.cache_clear()
    app = create_app(httpx.MockTransport(upstream))
    async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://test") as c:
        for _ in range(2):
            assert (await c.post("/api/language/check", json=CHECK, headers=AUTH)).status_code == 200
        limited = await c.post("/api/language/check", json=CHECK, headers=AUTH)
        assert limited.status_code == 429
        assert int(limited.headers["retry-after"]) >= 1
        other = {"Authorization": f"Bearer {token('someone-else')}"}
        assert (await c.post("/api/language/check", json=CHECK, headers=other)).status_code == 200
    assert len(upstream.requests) == 3
    get_settings.cache_clear()


def test_rate_limiter_window_slides():
    now = [0.0]
    limiter = RateLimiter(clock=lambda: now[0])
    limiter.hit("kev", "check", 1)
    with pytest.raises(Exception):
        limiter.hit("kev", "check", 1)
    now[0] = 61.0
    limiter.hit("kev", "check", 1)


# ------------------------------------------------------------ translate


async def test_translate_is_off_by_default(client):
    response = await client.post("/api/language/translate", json={"text": "hello", "target": "fr"}, headers=AUTH)
    assert response.status_code == 503


async def test_translate_when_set_up(env, upstream):
    env.setenv("LIBRETRANSLATE_URL", "http://libretranslate:5000")
    import httpx
    from language_tools.config import get_settings
    from language_tools.main import create_app

    get_settings.cache_clear()
    app = create_app(httpx.MockTransport(upstream))
    async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://test") as c:
        assert (await c.get("/api/language/translate/languages", headers=AUTH)).json() == {"languages": [{"code": "fr", "name": "French"}]}
        response = await c.post("/api/language/translate", json={"text": "hello", "target": "fr"}, headers=AUTH)
        assert response.json() == {"text": "[fr] hello", "detected": "en"}
    get_settings.cache_clear()
