"""Tests for the follow / badges / boosted-listing endpoints — issue #140."""

from __future__ import annotations

import pytest

from .conftest import make_auth_header


# --- Follow / unfollow ---------------------------------------------------


@pytest.mark.asyncio
async def test_follow_creates_relationship(client) -> None:
    headers = make_auth_header("alice")
    resp = await client.post("/api/users/bob/follow", headers=headers)
    assert resp.status_code == 200
    body = resp.json()
    assert body == {"follower": "alice", "followee": "bob", "following": True}


@pytest.mark.asyncio
async def test_follow_requires_auth(client) -> None:
    resp = await client.post("/api/users/bob/follow")
    assert resp.status_code == 401


@pytest.mark.asyncio
async def test_cannot_follow_self(client) -> None:
    headers = make_auth_header("alice")
    resp = await client.post("/api/users/alice/follow", headers=headers)
    assert resp.status_code == 400


@pytest.mark.asyncio
async def test_follow_is_idempotent(client) -> None:
    headers = make_auth_header("alice")
    r1 = await client.post("/api/users/bob/follow", headers=headers)
    r2 = await client.post("/api/users/bob/follow", headers=headers)
    assert r1.status_code == 200 and r2.status_code == 200
    # Followers count should still be 1 — not duplicated.
    r3 = await client.get("/api/users/bob/followers/count")
    assert r3.status_code == 200 and r3.json()["count"] == 1


@pytest.mark.asyncio
async def test_unfollow_idempotent(client) -> None:
    headers = make_auth_header("alice")
    # Unfollowing without a prior follow is a no-op success.
    r = await client.delete("/api/users/bob/follow", headers=headers)
    assert r.status_code == 200
    assert r.json()["following"] is False
    # Follow then unfollow.
    await client.post("/api/users/bob/follow", headers=headers)
    r2 = await client.delete("/api/users/bob/follow", headers=headers)
    assert r2.status_code == 200
    assert r2.json()["following"] is False
    # Count back to zero.
    r3 = await client.get("/api/users/bob/followers/count")
    assert r3.json()["count"] == 0


@pytest.mark.asyncio
async def test_follow_status_requires_auth(client) -> None:
    r = await client.get("/api/users/bob/follow")
    assert r.status_code == 401


@pytest.mark.asyncio
async def test_follow_status_reflects_state(client) -> None:
    headers = make_auth_header("alice")
    r = await client.get("/api/users/bob/follow", headers=headers)
    assert r.status_code == 200 and r.json()["following"] is False
    await client.post("/api/users/bob/follow", headers=headers)
    r2 = await client.get("/api/users/bob/follow", headers=headers)
    assert r2.json()["following"] is True


@pytest.mark.asyncio
async def test_following_and_followers_counts(client) -> None:
    # alice & carol follow bob; bob follows nobody.
    await client.post("/api/users/bob/follow", headers=make_auth_header("alice"))
    await client.post("/api/users/bob/follow", headers=make_auth_header("carol"))

    bob_followers = (await client.get("/api/users/bob/followers/count")).json()
    bob_following = (await client.get("/api/users/bob/following/count")).json()
    assert bob_followers == {"username": "bob", "count": 2}
    assert bob_following == {"username": "bob", "count": 0}

    alice_following = (await client.get("/api/users/alice/following/count")).json()
    assert alice_following == {"username": "alice", "count": 1}


@pytest.mark.asyncio
async def test_my_following_list(client) -> None:
    headers = make_auth_header("alice")
    await client.post("/api/users/bob/follow", headers=headers)
    await client.post("/api/users/carol/follow", headers=headers)
    r = await client.get("/api/users/me/following", headers=headers)
    assert r.status_code == 200
    body = r.json()
    assert body["follower"] == "alice"
    assert set(body["following"]) == {"bob", "carol"}


@pytest.mark.asyncio
async def test_my_following_requires_auth(client) -> None:
    r = await client.get("/api/users/me/following")
    assert r.status_code == 401


# --- boost_followed on the list endpoint --------------------------------


@pytest.mark.asyncio
async def test_list_projects_boost_followed(client) -> None:
    # Create two projects authored by different people. carol follows
    # dave but not bob. With boost_followed=true, dave's project must
    # come ahead of bob's regardless of which was created first.
    p_bob = (
        await client.post(
            "/api/projects",
            json={"title": "Bob's Robot"},
            headers=make_auth_header("bob"),
        )
    ).json()
    p_dave = (
        await client.post(
            "/api/projects",
            json={"title": "Dave's Drone"},
            headers=make_auth_header("dave"),
        )
    ).json()
    assert p_bob["id"] != p_dave["id"]

    carol_headers = make_auth_header("carol")
    await client.post("/api/users/dave/follow", headers=carol_headers)

    boosted = await client.get(
        "/api/projects?boost_followed=true", headers=carol_headers
    )
    assert boosted.status_code == 200
    boosted_ids = [p["id"] for p in boosted.json()]
    # Dave is followed → ranks before Bob even if Bob's created_at is
    # equal-or-earlier than Dave's.
    assert boosted_ids.index(p_dave["id"]) < boosted_ids.index(p_bob["id"])


@pytest.mark.asyncio
async def test_boost_followed_completed_first(client) -> None:
    # Two projects from different authors. The viewer follows the author
    # of the WIP one, but the *completed* project from a non-followed
    # author should still rank ahead (completed > followed > everyone).
    headers_bob = make_auth_header("bob")
    headers_dave = make_auth_header("dave")

    p_bob = (await client.post(
        "/api/projects", json={"title": "Bob's Robot"}, headers=headers_bob,
    )).json()
    p_dave = (await client.post(
        "/api/projects", json={"title": "Dave's Drone"}, headers=headers_dave,
    )).json()
    # Mark Dave's as completed.
    await client.put(
        f"/api/projects/{p_dave['id']}",
        json={"status": "completed"},
        headers=headers_dave,
    )

    carol = make_auth_header("carol")
    await client.post("/api/users/bob/follow", headers=carol)

    boosted = await client.get(
        "/api/projects?boost_followed=true", headers=carol
    )
    boosted_ids = [p["id"] for p in boosted.json()]
    assert boosted_ids.index(p_dave["id"]) < boosted_ids.index(p_bob["id"])


@pytest.mark.asyncio
async def test_list_projects_author_filter(client) -> None:
    await client.post(
        "/api/projects",
        json={"title": "Bob's Robot"},
        headers=make_auth_header("bob"),
    )
    await client.post(
        "/api/projects",
        json={"title": "Dave's Drone"},
        headers=make_auth_header("dave"),
    )
    r = await client.get("/api/projects?author=bob")
    assert r.status_code == 200
    items = r.json()
    assert len(items) == 1
    assert items[0]["author_username"] == "bob"


# --- Badges --------------------------------------------------------------


@pytest.mark.asyncio
async def test_followers_count_for_unknown_user_is_zero(client) -> None:
    r = await client.get("/api/users/ghost/followers/count")
    assert r.status_code == 200
    assert r.json() == {"username": "ghost", "count": 0}
