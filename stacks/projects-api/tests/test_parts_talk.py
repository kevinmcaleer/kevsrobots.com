"""Tests for parts talk pages (issue #122 Phase 2).

Covers thread creation + listing + per-thread detail, post append, post
edit (including the cross-author 403), and the 14-day account-age gate
on thread creation. Mirrors the auth-fixture pattern used by
``test_parts.py``.
"""

from __future__ import annotations

from datetime import datetime, timedelta
from typing import Optional

import pytest

from .conftest import make_auth_header


@pytest.fixture(autouse=True)
def _old_account(monkeypatch: pytest.MonkeyPatch) -> None:
    """Pretend every account is 5 years old by default."""
    from projects_api import auth as auth_module

    async def _fake_age(username: str, token: str) -> Optional[datetime]:
        return datetime.utcnow() - timedelta(days=365 * 5)

    monkeypatch.setattr(auth_module, "fetch_account_created_at", _fake_age)
    auth_module._clear_account_age_cache()


async def _create_part(client, name: str = "Servo") -> dict:
    resp = await client.post(
        "/api/parts",
        json={"name": name},
        headers=make_auth_header("kev"),
    )
    assert resp.status_code == 201, resp.text
    return resp.json()


@pytest.mark.asyncio
async def test_create_thread_writes_opening_post_atomically(client) -> None:
    part = await _create_part(client, "SG90")
    resp = await client.post(
        f"/api/parts/{part['slug']}/talk",
        json={
            "title": "Pinout question",
            "opening_post_content_md": "What's the brown wire?",
        },
        headers=make_auth_header("kev"),
    )
    assert resp.status_code == 201, resp.text
    body = resp.json()
    assert body["title"] == "Pinout question"
    assert body["part_slug"] == part["slug"]
    assert body["closed"] is False
    assert len(body["posts"]) == 1
    assert body["posts"][0]["content_md"] == "What's the brown wire?"
    assert body["posts"][0]["author_username"] == "kev"


@pytest.mark.asyncio
async def test_list_threads_orders_by_updated_at_desc(client) -> None:
    part = await _create_part(client, "Pico")
    for title in ("First thread", "Second thread", "Third thread"):
        resp = await client.post(
            f"/api/parts/{part['slug']}/talk",
            json={"title": title, "opening_post_content_md": "hi"},
            headers=make_auth_header("kev"),
        )
        assert resp.status_code == 201

    listing = await client.get(f"/api/parts/{part['slug']}/talk")
    assert listing.status_code == 200
    rows = listing.json()
    assert [r["title"] for r in rows] == ["Third thread", "Second thread", "First thread"]
    for row in rows:
        assert row["post_count"] == 1
        assert row["last_poster"] == "kev"


@pytest.mark.asyncio
async def test_add_post_bumps_thread_updated_at(client) -> None:
    part = await _create_part(client)
    create = await client.post(
        f"/api/parts/{part['slug']}/talk",
        json={"title": "Question", "opening_post_content_md": "first"},
        headers=make_auth_header("kev"),
    )
    thread_id = create.json()["id"]

    add = await client.post(
        f"/api/parts/{part['slug']}/talk/{thread_id}/posts",
        json={"content_md": "second"},
        headers=make_auth_header("alice"),
    )
    assert add.status_code == 201, add.text
    assert add.json()["author_username"] == "alice"

    detail = await client.get(f"/api/parts/{part['slug']}/talk/{thread_id}")
    assert detail.status_code == 200
    assert [p["content_md"] for p in detail.json()["posts"]] == ["first", "second"]


@pytest.mark.asyncio
async def test_post_to_closed_thread_returns_400(client) -> None:
    part = await _create_part(client)
    create = await client.post(
        f"/api/parts/{part['slug']}/talk",
        json={"title": "Question", "opening_post_content_md": "first"},
        headers=make_auth_header("kev"),
    )
    thread_id = create.json()["id"]

    close = await client.patch(
        f"/api/parts/{part['slug']}/talk/{thread_id}",
        json={"closed": True},
        headers=make_auth_header("kev"),
    )
    assert close.status_code == 200
    assert close.json()["closed"] is True

    resp = await client.post(
        f"/api/parts/{part['slug']}/talk/{thread_id}/posts",
        json={"content_md": "late"},
        headers=make_auth_header("alice"),
    )
    assert resp.status_code == 400


@pytest.mark.asyncio
async def test_close_thread_requires_op_or_admin(client) -> None:
    part = await _create_part(client)
    create = await client.post(
        f"/api/parts/{part['slug']}/talk",
        json={"title": "Question", "opening_post_content_md": "first"},
        headers=make_auth_header("kev"),
    )
    thread_id = create.json()["id"]

    # bob is not the OP and not an admin
    resp = await client.patch(
        f"/api/parts/{part['slug']}/talk/{thread_id}",
        json={"closed": True},
        headers=make_auth_header("bob"),
    )
    assert resp.status_code == 403


@pytest.mark.asyncio
async def test_edit_post_sets_edited_at_and_blocks_other_users(client) -> None:
    part = await _create_part(client)
    create = await client.post(
        f"/api/parts/{part['slug']}/talk",
        json={"title": "Question", "opening_post_content_md": "first"},
        headers=make_auth_header("alice"),
    )
    thread_id = create.json()["id"]
    post_id = create.json()["posts"][0]["id"]

    # Alice can edit her own post.
    edit = await client.patch(
        f"/api/parts/{part['slug']}/talk/{thread_id}/posts/{post_id}",
        json={"content_md": "first (edited)"},
        headers=make_auth_header("alice"),
    )
    assert edit.status_code == 200, edit.text
    assert edit.json()["content_md"] == "first (edited)"
    assert edit.json()["edited_at"] is not None

    # Bob cannot.
    bad = await client.patch(
        f"/api/parts/{part['slug']}/talk/{thread_id}/posts/{post_id}",
        json={"content_md": "vandalism"},
        headers=make_auth_header("bob"),
    )
    assert bad.status_code == 403


@pytest.mark.asyncio
async def test_create_thread_enforces_account_age_gate(client, monkeypatch) -> None:
    part = await _create_part(client)

    from projects_api import auth as auth_module

    async def _young(username: str, token: str) -> Optional[datetime]:
        return datetime.utcnow() - timedelta(days=3)

    monkeypatch.setattr(auth_module, "fetch_account_created_at", _young)
    auth_module._clear_account_age_cache()

    resp = await client.post(
        f"/api/parts/{part['slug']}/talk",
        json={"title": "Hi", "opening_post_content_md": "I'm new!"},
        headers=make_auth_header("kev"),
    )
    assert resp.status_code == 403


@pytest.mark.asyncio
async def test_post_to_thread_does_not_enforce_age_gate(client, monkeypatch) -> None:
    """The age gate is for thread creation; existing threads stay open
    to logged-in users regardless of account age (intentional design)."""
    part = await _create_part(client)
    create = await client.post(
        f"/api/parts/{part['slug']}/talk",
        json={"title": "Question", "opening_post_content_md": "first"},
        headers=make_auth_header("kev"),
    )
    thread_id = create.json()["id"]

    from projects_api import auth as auth_module

    async def _young(username: str, token: str) -> Optional[datetime]:
        return datetime.utcnow() - timedelta(days=3)

    monkeypatch.setattr(auth_module, "fetch_account_created_at", _young)
    auth_module._clear_account_age_cache()

    resp = await client.post(
        f"/api/parts/{part['slug']}/talk/{thread_id}/posts",
        json={"content_md": "freshly-joined chiming in"},
        headers=make_auth_header("newbie"),
    )
    assert resp.status_code == 201, resp.text


@pytest.mark.asyncio
async def test_get_thread_404_for_wrong_part(client) -> None:
    part_a = await _create_part(client, "Part A")
    part_b = await _create_part(client, "Part B")
    create = await client.post(
        f"/api/parts/{part_a['slug']}/talk",
        json={"title": "Question", "opening_post_content_md": "first"},
        headers=make_auth_header("kev"),
    )
    thread_id = create.json()["id"]
    # Same thread id but wrong part slug.
    resp = await client.get(f"/api/parts/{part_b['slug']}/talk/{thread_id}")
    assert resp.status_code == 404


@pytest.mark.asyncio
async def test_admin_can_edit_anyone(client) -> None:
    # 'kev' is the default admin per Settings.admin_usernames.
    part = await _create_part(client)
    create = await client.post(
        f"/api/parts/{part['slug']}/talk",
        json={"title": "Question", "opening_post_content_md": "first"},
        headers=make_auth_header("alice"),
    )
    thread_id = create.json()["id"]
    post_id = create.json()["posts"][0]["id"]

    resp = await client.patch(
        f"/api/parts/{part['slug']}/talk/{thread_id}/posts/{post_id}",
        json={"content_md": "moderator note"},
        headers=make_auth_header("kev"),  # admin
    )
    assert resp.status_code == 200
    assert resp.json()["content_md"] == "moderator note"
