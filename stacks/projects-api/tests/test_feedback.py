"""Tests for the feedback API (issue #138).

Covers POST /api/feedback (JSON + multipart) and the
/api/admin/feedback/* surface used by the admin inbox at
``web/admin/feedback.html``.

The local-fallback path in ``projects_api.storage`` writes screenshots
to ``/tmp/projects_uploads/projects/feedback/0/...`` during tests
because NAS credentials aren't configured. We poke at that path
directly to verify the file is actually written / removed.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from .conftest import make_auth_header


PNG_BYTES = (
    b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR"
    b"\x00\x00\x00\x01\x00\x00\x00\x01\x08\x06\x00\x00\x00"
    b"\x1f\x15\xc4\x89\x00\x00\x00\rIDATx\x9cc\xfc\xcf\xc0"
    b"\x00\x00\x00\x05\x00\x01\r\n-\xb4\x00\x00\x00\x00IEND\xaeB`\x82"
)


def _valid_payload(**overrides) -> dict:
    base = {
        "sentiment": "love",
        "message": "This is a long enough message for validation.",
        "email": None,
        "page_url": "https://www.kevsrobots.com/projects/view.html?id=1",
        "referrer": "",
        "user_agent": "pytest/0.0",
        "viewport": "1280x720",
    }
    base.update(overrides)
    return base


# ----- POST /api/feedback (user) -----------------------------------------


@pytest.mark.asyncio
async def test_post_feedback_json_returns_201(client) -> None:
    """A logged-in user can submit JSON feedback."""
    headers = make_auth_header("alice")
    resp = await client.post(
        "/api/feedback", json=_valid_payload(), headers=headers
    )
    assert resp.status_code == 201
    body = resp.json()
    assert isinstance(body["id"], int)
    assert body["status"] == "unread"
    assert "created_at" in body


@pytest.mark.asyncio
async def test_post_feedback_unauthenticated_returns_401(client) -> None:
    resp = await client.post("/api/feedback", json=_valid_payload())
    assert resp.status_code == 401


@pytest.mark.asyncio
async def test_post_feedback_anonymous_with_app_key_returns_201(
    client, monkeypatch
) -> None:
    """A trusted first-party app (Snakie) can submit anonymously with the shared
    X-Snakie-Key header — no Chatter session required (issue #206)."""
    monkeypatch.setenv("SNAKIE_FEEDBACK_KEY", "s3cr3t-app-key")
    resp = await client.post(
        "/api/feedback",
        json=_valid_payload(sentiment="issue", message="_SNAKIE_ crash on connect"),
        headers={"X-Snakie-Key": "s3cr3t-app-key"},
    )
    assert resp.status_code == 201
    assert isinstance(resp.json()["id"], int)


@pytest.mark.asyncio
async def test_post_feedback_wrong_app_key_returns_401(client, monkeypatch) -> None:
    """A bad key with no session falls through to the auth gate → 401."""
    monkeypatch.setenv("SNAKIE_FEEDBACK_KEY", "s3cr3t-app-key")
    resp = await client.post(
        "/api/feedback", json=_valid_payload(), headers={"X-Snakie-Key": "wrong"}
    )
    assert resp.status_code == 401


@pytest.mark.asyncio
async def test_post_feedback_anonymous_disabled_by_default_returns_401(client) -> None:
    """With no SNAKIE_FEEDBACK_KEY configured the anonymous path is OFF, so even a
    presented key falls through to auth → 401 (secure default)."""
    resp = await client.post(
        "/api/feedback", json=_valid_payload(), headers={"X-Snakie-Key": "anything"}
    )
    assert resp.status_code == 401


@pytest.mark.asyncio
async def test_post_feedback_missing_required_field_returns_400(client) -> None:
    """Pydantic surfaces a 400 via our handler (not the default 422)."""
    headers = make_auth_header("alice")
    bad = _valid_payload()
    bad.pop("sentiment")
    resp = await client.post("/api/feedback", json=bad, headers=headers)
    assert resp.status_code == 400


@pytest.mark.asyncio
async def test_post_feedback_bad_sentiment_returns_400(client) -> None:
    headers = make_auth_header("alice")
    resp = await client.post(
        "/api/feedback",
        json=_valid_payload(sentiment="hate"),
        headers=headers,
    )
    assert resp.status_code == 400


@pytest.mark.asyncio
async def test_post_feedback_short_message_returns_400(client) -> None:
    headers = make_auth_header("alice")
    resp = await client.post(
        "/api/feedback",
        json=_valid_payload(message="too short"),
        headers=headers,
    )
    assert resp.status_code == 400


@pytest.mark.asyncio
async def test_post_feedback_invalid_email_returns_400(client) -> None:
    headers = make_auth_header("alice")
    resp = await client.post(
        "/api/feedback",
        json=_valid_payload(email="not-an-email"),
        headers=headers,
    )
    assert resp.status_code == 400


@pytest.mark.asyncio
async def test_post_feedback_strips_client_user_id(client, session) -> None:
    """Spoofed user_id/username in the body must not land on the row."""
    from sqlalchemy import select
    from projects_api.models import Feedback

    headers = make_auth_header("alice")
    payload = _valid_payload()
    payload["user_id"] = "attacker"
    payload["username"] = "attacker"
    resp = await client.post("/api/feedback", json=payload, headers=headers)
    assert resp.status_code == 201
    fid = resp.json()["id"]

    row = (await session.execute(select(Feedback).where(Feedback.id == fid))).scalar_one()
    assert row.user_id == "alice"
    assert row.username == "alice"


@pytest.mark.asyncio
async def test_post_feedback_multipart_with_screenshot(client) -> None:
    """Multipart variant accepts a screenshot file part."""
    headers = make_auth_header("alice")
    resp = await client.post(
        "/api/feedback",
        data={
            "sentiment": "issue",
            "message": "Found a bug in the editor when uploading images.",
            "page_url": "https://www.kevsrobots.com/projects/view.html?id=2",
            "referrer": "",
            "user_agent": "pytest/0.0",
            "viewport": "1280x720",
        },
        files={
            "screenshot": ("bug.png", PNG_BYTES, "image/png"),
        },
        headers=headers,
    )
    assert resp.status_code == 201


@pytest.mark.asyncio
async def test_post_feedback_oversize_screenshot_returns_413(client) -> None:
    headers = make_auth_header("alice")
    # 5 MB of zeros — over the 4 MB cap.
    big = b"\x00" * (5 * 1024 * 1024)
    resp = await client.post(
        "/api/feedback",
        data={
            "sentiment": "issue",
            "message": "This screenshot should bounce off the size cap.",
            "page_url": "https://www.kevsrobots.com/p/x",
        },
        files={"screenshot": ("huge.png", big, "image/png")},
        headers=headers,
    )
    assert resp.status_code == 413


@pytest.mark.asyncio
async def test_post_feedback_wrong_mime_returns_415(client) -> None:
    headers = make_auth_header("alice")
    resp = await client.post(
        "/api/feedback",
        data={
            "sentiment": "issue",
            "message": "Screenshot has the wrong MIME type entirely.",
            "page_url": "https://www.kevsrobots.com/p/x",
        },
        files={"screenshot": ("evil.exe", b"MZbinary", "application/octet-stream")},
        headers=headers,
    )
    assert resp.status_code == 415


# ----- GET /api/admin/feedback (admin) -----------------------------------


async def _seed(client, *, user: str, sentiment: str, message: str) -> int:
    """Submit a feedback row as ``user`` and return its id."""
    resp = await client.post(
        "/api/feedback",
        json=_valid_payload(sentiment=sentiment, message=message),
        headers=make_auth_header(user),
    )
    assert resp.status_code == 201
    return resp.json()["id"]


@pytest.mark.asyncio
async def test_admin_list_requires_admin(client, monkeypatch) -> None:
    monkeypatch.setenv("ADMIN_USERNAMES", "boss")
    # Logged in but not admin -> 403.
    resp = await client.get(
        "/api/admin/feedback", headers=make_auth_header("alice")
    )
    assert resp.status_code == 403
    # Anonymous -> 401.
    resp = await client.get("/api/admin/feedback")
    assert resp.status_code == 401


@pytest.mark.asyncio
async def test_admin_list_returns_total_and_items(client, monkeypatch) -> None:
    monkeypatch.setenv("ADMIN_USERNAMES", "boss")
    await _seed(client, user="alice", sentiment="love", message="Loved the editor flow today.")
    await _seed(client, user="bob", sentiment="issue", message="Submit button is offscreen at 1024px.")
    await _seed(client, user="alice", sentiment="idea", message="Tag autocomplete would be nice please.")

    resp = await client.get(
        "/api/admin/feedback", headers=make_auth_header("boss")
    )
    assert resp.status_code == 200
    body = resp.json()
    assert body["total"] == 3
    assert len(body["items"]) == 3
    # Newest first — ids are inserted in order so the last one wins.
    assert body["items"][0]["sentiment"] == "idea"


@pytest.mark.asyncio
async def test_admin_list_filter_by_sentiment(client, monkeypatch) -> None:
    monkeypatch.setenv("ADMIN_USERNAMES", "boss")
    await _seed(client, user="alice", sentiment="love", message="Loved the editor flow today.")
    await _seed(client, user="bob", sentiment="issue", message="Submit button is offscreen at 1024px.")
    await _seed(client, user="alice", sentiment="idea", message="Tag autocomplete would be nice please.")

    resp = await client.get(
        "/api/admin/feedback?sentiment=issue",
        headers=make_auth_header("boss"),
    )
    assert resp.status_code == 200
    body = resp.json()
    assert body["total"] == 1
    assert body["items"][0]["sentiment"] == "issue"


@pytest.mark.asyncio
async def test_admin_list_q_search(client, monkeypatch) -> None:
    monkeypatch.setenv("ADMIN_USERNAMES", "boss")
    await _seed(client, user="alice", sentiment="love", message="Loved the editor flow today.")
    await _seed(client, user="bob", sentiment="issue", message="Submit button is offscreen at 1024px.")

    resp = await client.get(
        "/api/admin/feedback?q=offscreen",
        headers=make_auth_header("boss"),
    )
    assert resp.status_code == 200
    body = resp.json()
    assert body["total"] == 1
    assert "offscreen" in body["items"][0]["message"]


@pytest.mark.asyncio
async def test_admin_get_single_404(client, monkeypatch) -> None:
    monkeypatch.setenv("ADMIN_USERNAMES", "boss")
    resp = await client.get(
        "/api/admin/feedback/99999",
        headers=make_auth_header("boss"),
    )
    assert resp.status_code == 404


# ----- PATCH /api/admin/feedback/{id} ------------------------------------


@pytest.mark.asyncio
async def test_admin_patch_status_marks_read(client, monkeypatch) -> None:
    monkeypatch.setenv("ADMIN_USERNAMES", "boss")
    fid = await _seed(client, user="alice", sentiment="love", message="Loved the editor flow today.")

    resp = await client.patch(
        f"/api/admin/feedback/{fid}",
        json={"status": "read"},
        headers=make_auth_header("boss"),
    )
    assert resp.status_code == 200
    body = resp.json()
    assert body["status"] == "read"
    assert body["read_at"] is not None
    assert body["read_by_user_id"] == "boss"


@pytest.mark.asyncio
async def test_admin_patch_invalid_status_returns_422_or_400(
    client, monkeypatch
) -> None:
    monkeypatch.setenv("ADMIN_USERNAMES", "boss")
    fid = await _seed(client, user="alice", sentiment="love", message="Loved the editor flow today.")

    resp = await client.patch(
        f"/api/admin/feedback/{fid}",
        json={"status": "bogus"},
        headers=make_auth_header("boss"),
    )
    # FastAPI surfaces Pydantic validation errors as 422 by default;
    # either is acceptable per the spec ("400 — invalid status").
    assert resp.status_code in (400, 422)


# ----- DELETE /api/admin/feedback/{id} -----------------------------------


@pytest.mark.asyncio
async def test_admin_delete_removes_row_and_screenshot(
    client, session, monkeypatch
) -> None:
    monkeypatch.setenv("ADMIN_USERNAMES", "boss")
    # POST with a screenshot via multipart.
    resp = await client.post(
        "/api/feedback",
        data={
            "sentiment": "issue",
            "message": "Screenshot deletion check needs persistence.",
            "page_url": "https://www.kevsrobots.com/p/x",
        },
        files={"screenshot": ("shot.png", PNG_BYTES, "image/png")},
        headers=make_auth_header("alice"),
    )
    assert resp.status_code == 201
    fid = resp.json()["id"]

    # Read back the stored path so we can assert the file exists then vanishes.
    from sqlalchemy import select
    from projects_api.models import Feedback

    row = (
        await session.execute(select(Feedback).where(Feedback.id == fid))
    ).scalar_one()
    storage_path = row.screenshot_path
    assert storage_path and storage_path.startswith("local:")
    on_disk = Path("/tmp/projects_uploads") / storage_path[len("local:") :]
    assert on_disk.exists()

    # Delete.
    resp = await client.delete(
        f"/api/admin/feedback/{fid}",
        headers=make_auth_header("boss"),
    )
    assert resp.status_code == 204

    # Row gone.
    again = (
        await session.execute(select(Feedback).where(Feedback.id == fid))
    ).scalar_one_or_none()
    assert again is None
    # File gone.
    assert not on_disk.exists()


# ----- GET /api/admin/feedback/counts ------------------------------------


@pytest.mark.asyncio
async def test_admin_counts_match_total(client, monkeypatch) -> None:
    monkeypatch.setenv("ADMIN_USERNAMES", "boss")
    await _seed(client, user="alice", sentiment="love", message="Loved the editor flow today.")
    await _seed(client, user="bob", sentiment="issue", message="Submit button is offscreen at 1024px.")
    await _seed(client, user="alice", sentiment="idea", message="Tag autocomplete would be nice please.")
    fid_read = await _seed(client, user="alice", sentiment="like", message="Liked the new look and feel.")
    await client.patch(
        f"/api/admin/feedback/{fid_read}",
        json={"status": "read"},
        headers=make_auth_header("boss"),
    )

    resp = await client.get(
        "/api/admin/feedback/counts",
        headers=make_auth_header("boss"),
    )
    assert resp.status_code == 200
    counts = resp.json()
    assert counts["total"] == 4
    assert counts["unread"] == 3
    assert counts["read"] == 1
    assert counts["archived"] == 0
    # Status counts add up to total.
    assert counts["unread"] + counts["read"] + counts["archived"] == counts["total"]
    # Sentiment counts add up to total too.
    assert sum(counts["by_sentiment"].values()) == counts["total"]


@pytest.mark.asyncio
async def test_admin_counts_requires_admin(client, monkeypatch) -> None:
    monkeypatch.setenv("ADMIN_USERNAMES", "boss")
    resp = await client.get(
        "/api/admin/feedback/counts",
        headers=make_auth_header("alice"),
    )
    assert resp.status_code == 403
    resp = await client.get("/api/admin/feedback/counts")
    assert resp.status_code == 401
