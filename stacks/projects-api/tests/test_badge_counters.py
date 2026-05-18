"""Tests for the external badge counters wired in issue #142.

Covers:
* Likes-received sums across the user's projects.
* A single project's Chatter call failing (5xx) is treated as 0 but
  doesn't kill the rest of the sum.
* The cache prevents a second HTTP call within TTL.
* When Chatter is fully down and the cache has a stale value, the stale
  value is returned (graceful degradation).
* Comments counter returns 0 with no HTTP call when the config endpoint
  is empty (current production state).
* Comments counter calls the configured endpoint when set and parses
  the count.
* Threshold-crossing integration: with mocked likes returning >= 10,
  Alice crosses ``well_liked_bronze`` after ``evaluate_user``.
"""

from __future__ import annotations

from typing import Any, Optional

import httpx
import pytest

from projects_api import badge_counter_cache, badges as badges_module
from projects_api.badge_counter_cache import clear_badge_counter_cache
from projects_api.badges import (
    _count_comments,
    _count_likes_received,
    evaluate_user,
    seed_badge_definitions,
)
from projects_api.models import Project


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


class _FakeResponse:
    def __init__(self, status_code: int, payload: Any) -> None:
        self.status_code = status_code
        self._payload = payload

    def json(self) -> Any:
        return self._payload


class _FakeAsyncClient:
    """Minimal stand-in for ``httpx.AsyncClient`` used in tests.

    Routes are matched by full URL substring; the first matching rule
    wins. Each rule may be a callable (called with the URL — useful for
    raising) or a ``_FakeResponse``.
    """

    def __init__(self, routes: list[tuple[str, Any]]) -> None:
        self._routes = routes
        self.calls: list[str] = []

    async def __aenter__(self) -> "_FakeAsyncClient":
        return self

    async def __aexit__(self, *exc_info: Any) -> None:
        return None

    async def get(self, url: str) -> _FakeResponse:
        self.calls.append(url)
        for needle, rule in self._routes:
            if needle in url:
                if callable(rule):
                    return rule(url)
                return rule
        raise AssertionError(f"No fake route matched {url}")


def _install_fake_client(monkeypatch: pytest.MonkeyPatch, fake: _FakeAsyncClient) -> None:
    """Replace ``httpx.AsyncClient`` inside badges.py with ``fake``.

    We patch the symbol on the badges module specifically (not globally
    on httpx) so unrelated httpx usage in the rest of the test suite is
    untouched. The fake ignores the timeout kwarg badges.py passes.
    """

    def _factory(*_args: Any, **_kwargs: Any) -> _FakeAsyncClient:
        return fake

    monkeypatch.setattr(badges_module.httpx, "AsyncClient", _factory)


@pytest.fixture(autouse=True)
def _reset_counter_cache() -> None:
    """Drop the in-process counter cache before every test in this file."""
    clear_badge_counter_cache()
    yield
    clear_badge_counter_cache()


# ---------------------------------------------------------------------------
# Likes
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_likes_sums_across_projects(session, monkeypatch) -> None:
    """Three projects, Chatter returns 3 / 4 / 5 — total should be 12."""
    from sqlalchemy import select as _select

    for title in ("A", "B", "C"):
        session.add(Project(title=title, status="wip", author_username="alice"))
    await session.commit()
    project_ids = (
        await session.scalars(
            _select(Project.id).where(Project.author_username == "alice")
        )
    ).all()
    assert len(project_ids) == 3

    # URLs from badges.py are percent-encoded so `=` becomes `%3D`.
    counts = {f"id%3D{pid}": n for pid, n in zip(project_ids, [3, 4, 5])}

    def _route(url: str) -> _FakeResponse:
        for key, count in counts.items():
            if key in url:
                return _FakeResponse(200, {"count": count})
        return _FakeResponse(200, {"count": 0})

    fake = _FakeAsyncClient([("/interact/likes/", _route)])
    _install_fake_client(monkeypatch, fake)

    total = await _count_likes_received(session, "alice")
    assert total == 12
    # One call per project.
    assert len(fake.calls) == 3


@pytest.mark.asyncio
async def test_likes_single_500_does_not_kill_sum(session, monkeypatch) -> None:
    """One project's Chatter call 500s — the other two still sum."""
    from sqlalchemy import select as _select

    for title in ("A", "B", "C"):
        session.add(Project(title=title, status="wip", author_username="alice"))
    await session.commit()
    project_ids = (
        await session.scalars(
            _select(Project.id).where(Project.author_username == "alice")
        )
    ).all()
    p_ok1, p_bad, p_ok2 = sorted(project_ids)

    def _route(url: str) -> _FakeResponse:
        if f"id%3D{p_bad}" in url:
            return _FakeResponse(500, {})
        return _FakeResponse(200, {"count": 7})

    fake = _FakeAsyncClient([("/interact/likes/", _route)])
    _install_fake_client(monkeypatch, fake)

    total = await _count_likes_received(session, "alice")
    # 7 + 0 (the 500) + 7 = 14
    assert total == 14


@pytest.mark.asyncio
async def test_likes_cache_hit_avoids_http(session, monkeypatch) -> None:
    """Second call within TTL must not touch the network."""
    session.add(Project(title="A", status="wip", author_username="alice"))
    await session.commit()

    fake = _FakeAsyncClient([
        ("/interact/likes/", _FakeResponse(200, {"count": 9})),
    ])
    _install_fake_client(monkeypatch, fake)

    first = await _count_likes_received(session, "alice")
    assert first == 9
    assert len(fake.calls) == 1

    # Second call: cache should serve, fake stays untouched.
    second = await _count_likes_received(session, "alice")
    assert second == 9
    assert len(fake.calls) == 1


@pytest.mark.asyncio
async def test_likes_stale_returned_when_chatter_down(session, monkeypatch) -> None:
    """First call seeds the cache; then Chatter goes fully down and the
    cache expires — the stale value must still be returned rather than 0."""
    session.add(Project(title="A", status="wip", author_username="alice"))
    await session.commit()

    fake_ok = _FakeAsyncClient([
        ("/interact/likes/", _FakeResponse(200, {"count": 42})),
    ])
    _install_fake_client(monkeypatch, fake_ok)

    first = await _count_likes_received(session, "alice")
    assert first == 42

    # Force the cache to look stale by patching the cache module's
    # settings reader to return TTL=0 (so any cached entry is considered
    # expired and a re-fetch is attempted).
    from projects_api import config as config_module

    real_get = config_module.get_settings

    def _zero_ttl_settings():
        return real_get().model_copy(update={"badge_counter_cache_ttl_seconds": 0})

    monkeypatch.setattr(badge_counter_cache, "get_settings", _zero_ttl_settings)

    # Now Chatter is unreachable: every call raises.
    def _boom(url: str) -> _FakeResponse:
        raise httpx.ConnectError("connection refused")

    fake_down = _FakeAsyncClient([("/interact/likes/", _boom)])
    _install_fake_client(monkeypatch, fake_down)

    second = await _count_likes_received(session, "alice")
    assert second == 42, "should return stale cached value, not 0"


# ---------------------------------------------------------------------------
# Comments
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_comments_default_returns_zero_no_http(session, monkeypatch) -> None:
    """With ``chatter_user_comments_endpoint`` unset (default), the counter
    returns 0 without ever touching the network."""
    called: list[str] = []

    def _track(url: str) -> _FakeResponse:
        called.append(url)
        return _FakeResponse(200, {"count": 999})

    fake = _FakeAsyncClient([("", _track)])
    _install_fake_client(monkeypatch, fake)

    n = await _count_comments(session, "alice")
    assert n == 0
    assert called == []


@pytest.mark.asyncio
async def test_comments_calls_configured_endpoint(session, monkeypatch) -> None:
    """When ``chatter_user_comments_endpoint`` is set, the counter calls
    it with the username substituted in and returns the parsed count."""
    from projects_api import config as config_module

    real_get = config_module.get_settings

    def _with_endpoint():
        s = real_get()
        return s.model_copy(update={
            "chatter_user_comments_endpoint": "/api/users/{username}/comments_count",
            "badge_counter_cache_ttl_seconds": 300,
        })

    monkeypatch.setattr(badges_module, "get_settings", _with_endpoint)
    monkeypatch.setattr(badge_counter_cache, "get_settings", _with_endpoint)

    fake = _FakeAsyncClient([
        ("/api/users/alice/comments_count", _FakeResponse(200, {"count": 17})),
    ])
    _install_fake_client(monkeypatch, fake)

    n = await _count_comments(session, "alice")
    assert n == 17
    assert any("alice/comments_count" in c for c in fake.calls)


# ---------------------------------------------------------------------------
# Integration with evaluate_user
# ---------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_well_liked_bronze_awarded_when_likes_cross_threshold(
    session, monkeypatch
) -> None:
    """Mocked Chatter returns 10 likes for Alice's one project — running
    the evaluator should award ``well_liked_bronze`` (threshold 10)."""
    await seed_badge_definitions(session)

    session.add(Project(title="A", status="wip", author_username="alice"))
    await session.commit()

    fake = _FakeAsyncClient([
        ("/interact/likes/", _FakeResponse(200, {"count": 10})),
    ])
    _install_fake_client(monkeypatch, fake)

    awarded = await evaluate_user(session, "alice")
    slugs = {b.slug for b in awarded}
    assert "well_liked_bronze" in slugs
    # And the silver tier (50) should NOT have been awarded with only 10.
    assert "well_liked_silver" not in slugs
