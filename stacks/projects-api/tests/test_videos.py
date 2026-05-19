"""Project videos CRUD tests (issue #171)."""

from __future__ import annotations

import pytest
from .conftest import make_auth_header


@pytest.fixture
async def project_id(client) -> int:
    headers = make_auth_header()
    resp = await client.post(
        "/api/projects",
        json={"title": "Video Test Project"},
        headers=headers,
    )
    return resp.json()["id"]


@pytest.mark.asyncio
async def test_video_crud_round_trip(client, project_id) -> None:
    headers = make_auth_header()

    # POST — extractor should pull the id out of a full watch URL.
    resp = await client.post(
        f"/api/projects/{project_id}/videos",
        json={
            "url_or_id": "https://www.youtube.com/watch?v=dQw4w9WgXcQ",
            "title": "Build walkthrough",
        },
        headers=headers,
    )
    assert resp.status_code == 201, resp.text
    body = resp.json()
    assert body["youtube_id"] == "dQw4w9WgXcQ"
    assert body["title"] == "Build walkthrough"
    assert body["sort_order"] == 0
    video_id = body["id"]

    # GET — public, no auth needed.
    resp = await client.get(f"/api/projects/{project_id}/videos")
    assert resp.status_code == 200
    rows = resp.json()
    assert len(rows) == 1
    assert rows[0]["youtube_id"] == "dQw4w9WgXcQ"

    # PUT — title-only edit.
    resp = await client.put(
        f"/api/projects/{project_id}/videos/{video_id}",
        json={"title": "Renamed walkthrough"},
        headers=headers,
    )
    assert resp.status_code == 200
    assert resp.json()["title"] == "Renamed walkthrough"
    # youtube_id is immutable, must not have changed.
    assert resp.json()["youtube_id"] == "dQw4w9WgXcQ"

    # DELETE
    resp = await client.delete(
        f"/api/projects/{project_id}/videos/{video_id}", headers=headers
    )
    assert resp.status_code == 204
    resp = await client.get(f"/api/projects/{project_id}/videos")
    assert resp.json() == []


@pytest.mark.asyncio
async def test_video_accepts_bare_id(client, project_id) -> None:
    headers = make_auth_header()
    resp = await client.post(
        f"/api/projects/{project_id}/videos",
        json={"url_or_id": "dQw4w9WgXcQ"},
        headers=headers,
    )
    assert resp.status_code == 201
    assert resp.json()["youtube_id"] == "dQw4w9WgXcQ"


@pytest.mark.asyncio
async def test_video_accepts_youtu_be_short_url(client, project_id) -> None:
    headers = make_auth_header()
    resp = await client.post(
        f"/api/projects/{project_id}/videos",
        json={"url_or_id": "https://youtu.be/dQw4w9WgXcQ?t=10"},
        headers=headers,
    )
    assert resp.status_code == 201
    assert resp.json()["youtube_id"] == "dQw4w9WgXcQ"


@pytest.mark.asyncio
async def test_video_accepts_shorts_url(client, project_id) -> None:
    headers = make_auth_header()
    resp = await client.post(
        f"/api/projects/{project_id}/videos",
        json={"url_or_id": "https://www.youtube.com/shorts/dQw4w9WgXcQ"},
        headers=headers,
    )
    assert resp.status_code == 201
    assert resp.json()["youtube_id"] == "dQw4w9WgXcQ"


@pytest.mark.asyncio
async def test_malformed_url_returns_422(client, project_id) -> None:
    headers = make_auth_header()
    resp = await client.post(
        f"/api/projects/{project_id}/videos",
        json={"url_or_id": "https://example.com/not-a-youtube-url"},
        headers=headers,
    )
    assert resp.status_code == 422
    # The detail should mention YouTube so the frontend can show a
    # clear error rather than a generic 422.
    body = resp.json()
    assert "youtube" in str(body).lower() or "recognised" in str(body).lower()


@pytest.mark.asyncio
async def test_videos_ordered_by_sort_order(client, project_id) -> None:
    """List endpoint sorts ascending by sort_order, then by id."""
    headers = make_auth_header()
    # Three videos — they land with sort_order 0, 1, 2 in insertion order.
    ids = []
    for slug in ("aaaaaaaaaaa", "bbbbbbbbbbb", "ccccccccccc"):
        # We need a real id shape (any 11 chars matching the regex
        # passes). Lowercase letters are fine.
        resp = await client.post(
            f"/api/projects/{project_id}/videos",
            json={"url_or_id": slug},
            headers=headers,
        )
        assert resp.status_code == 201
        ids.append(resp.json()["id"])

    # Bump the last video to the top with sort_order = -1.
    resp = await client.put(
        f"/api/projects/{project_id}/videos/{ids[2]}",
        json={"sort_order": -1},
        headers=headers,
    )
    # ge=0 on the schema → -1 should 422.
    assert resp.status_code == 422

    # Use 0 instead and tie-break by id (the third row was created last,
    # so id is highest — but if sort_order ties, our ORDER BY id
    # places it AFTER the first row that's also at 0).
    resp = await client.put(
        f"/api/projects/{project_id}/videos/{ids[2]}",
        json={"sort_order": 0},
        headers=headers,
    )
    assert resp.status_code == 200

    resp = await client.get(f"/api/projects/{project_id}/videos")
    rows = resp.json()
    assert [r["id"] for r in rows] == [ids[0], ids[2], ids[1]]


@pytest.mark.asyncio
async def test_video_post_requires_owner(client, project_id) -> None:
    """A logged-in non-owner can't add a video to someone else's project."""
    other = make_auth_header(username="someoneelse")
    resp = await client.post(
        f"/api/projects/{project_id}/videos",
        json={"url_or_id": "dQw4w9WgXcQ"},
        headers=other,
    )
    assert resp.status_code == 403


@pytest.mark.asyncio
async def test_video_put_requires_owner(client, project_id) -> None:
    owner = make_auth_header()
    resp = await client.post(
        f"/api/projects/{project_id}/videos",
        json={"url_or_id": "dQw4w9WgXcQ"},
        headers=owner,
    )
    video_id = resp.json()["id"]

    other = make_auth_header(username="someoneelse")
    resp = await client.put(
        f"/api/projects/{project_id}/videos/{video_id}",
        json={"title": "hijacked"},
        headers=other,
    )
    assert resp.status_code == 403


@pytest.mark.asyncio
async def test_video_delete_requires_owner(client, project_id) -> None:
    owner = make_auth_header()
    resp = await client.post(
        f"/api/projects/{project_id}/videos",
        json={"url_or_id": "dQw4w9WgXcQ"},
        headers=owner,
    )
    video_id = resp.json()["id"]

    other = make_auth_header(username="someoneelse")
    resp = await client.delete(
        f"/api/projects/{project_id}/videos/{video_id}",
        headers=other,
    )
    assert resp.status_code == 403


@pytest.mark.asyncio
async def test_anonymous_can_read_videos(client, project_id) -> None:
    headers = make_auth_header()
    await client.post(
        f"/api/projects/{project_id}/videos",
        json={"url_or_id": "dQw4w9WgXcQ"},
        headers=headers,
    )
    # No auth header — public read should still work.
    resp = await client.get(f"/api/projects/{project_id}/videos")
    assert resp.status_code == 200
    assert len(resp.json()) == 1
