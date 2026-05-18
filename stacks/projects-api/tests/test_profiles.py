"""Tests for the public user profile endpoints — issue #111.

Covers:
  * GET /api/users/{username}/profile (shape, 404, defaults)
  * PUT /api/users/me/profile (self-only, length + URL validation,
    featured-badge cap)
  * GET /api/users/{username}/activity (paginated, event emission)
  * GET /api/users/{username}/followers and /following lists
  * GET /api/users/me/follows/{username} probe
  * boost_followed reordering on /api/projects with a small fixture
"""

from __future__ import annotations

import pytest

from .conftest import make_auth_header


# --- GET /api/users/{username}/profile ----------------------------------


@pytest.mark.asyncio
async def test_profile_404_for_unknown_user(client) -> None:
    r = await client.get("/api/users/nobody/profile")
    assert r.status_code == 404


@pytest.mark.asyncio
async def test_profile_returns_defaults_for_user_with_only_a_project(client) -> None:
    """A user who has created a project but never edited their profile
    still gets a 200 with sensible defaults."""
    await client.post(
        "/api/projects",
        json={"title": "Alice's First"},
        headers=make_auth_header("alice"),
    )
    r = await client.get("/api/users/alice/profile")
    assert r.status_code == 200
    body = r.json()
    assert body["username"] == "alice"
    assert body["bio"] is None
    assert body["location"] is None
    assert body["website_url"] is None
    assert body["social_links"] == {
        "github": None,
        "twitter": None,
        "youtube": None,
        "mastodon": None,
    }
    assert body["featured_badge_slugs"] == []
    assert body["stats"]["projects"] == 1
    assert body["stats"]["makes"] == 0
    assert body["stats"]["followers"] == 0
    assert body["stats"]["following"] == 0


@pytest.mark.asyncio
async def test_profile_includes_followers_and_following_counts(client) -> None:
    await client.post(
        "/api/projects",
        json={"title": "Bob's Robot"},
        headers=make_auth_header("bob"),
    )
    # Alice follows Bob, Carol follows Bob.
    await client.post("/api/users/bob/follow", headers=make_auth_header("alice"))
    await client.post("/api/users/bob/follow", headers=make_auth_header("carol"))
    # Bob follows Alice.
    await client.post(
        "/api/projects",
        json={"title": "Alice's First"},
        headers=make_auth_header("alice"),
    )
    await client.post("/api/users/alice/follow", headers=make_auth_header("bob"))

    r = await client.get("/api/users/bob/profile")
    assert r.status_code == 200
    body = r.json()
    assert body["stats"]["followers"] == 2
    assert body["stats"]["following"] == 1


# --- PUT /api/users/me/profile ------------------------------------------


@pytest.mark.asyncio
async def test_put_profile_requires_auth(client) -> None:
    r = await client.put("/api/users/me/profile", json={"bio": "hi"})
    assert r.status_code == 401


@pytest.mark.asyncio
async def test_put_profile_creates_row_lazily(client) -> None:
    headers = make_auth_header("alice")
    r = await client.put(
        "/api/users/me/profile",
        json={
            "bio": "Robots are great.",
            "location": "Glasgow",
            "website_url": "https://example.com",
            "social_links": {"github": "https://github.com/alice"},
            "featured_badge_slugs": ["first-project"],
        },
        headers=headers,
    )
    assert r.status_code == 200, r.text
    body = r.json()
    assert body["bio"] == "Robots are great."
    assert body["location"] == "Glasgow"
    assert body["website_url"] == "https://example.com"
    assert body["social_links"]["github"] == "https://github.com/alice"
    assert body["featured_badge_slugs"] == ["first-project"]

    # GET reflects the saved data.
    r2 = await client.get("/api/users/alice/profile")
    assert r2.status_code == 200
    assert r2.json()["bio"] == "Robots are great."


@pytest.mark.asyncio
async def test_put_profile_rejects_oversize_bio(client) -> None:
    headers = make_auth_header("alice")
    r = await client.put(
        "/api/users/me/profile",
        json={"bio": "x" * 501},
        headers=headers,
    )
    assert r.status_code == 422


@pytest.mark.asyncio
async def test_put_profile_rejects_bad_website_url(client) -> None:
    headers = make_auth_header("alice")
    r = await client.put(
        "/api/users/me/profile",
        json={"website_url": "not a url"},
        headers=headers,
    )
    assert r.status_code == 400


@pytest.mark.asyncio
async def test_put_profile_rejects_bad_social_url(client) -> None:
    headers = make_auth_header("alice")
    r = await client.put(
        "/api/users/me/profile",
        json={"social_links": {"github": "javascript:alert(1)"}},
        headers=headers,
    )
    assert r.status_code == 400


@pytest.mark.asyncio
async def test_put_profile_allows_clearing_url_with_empty_string(client) -> None:
    headers = make_auth_header("alice")
    await client.put(
        "/api/users/me/profile",
        json={"website_url": "https://example.com"},
        headers=headers,
    )
    r = await client.put(
        "/api/users/me/profile",
        json={"website_url": ""},
        headers=headers,
    )
    assert r.status_code == 200
    assert r.json()["website_url"] is None


@pytest.mark.asyncio
async def test_put_profile_caps_featured_badges_at_three(client) -> None:
    headers = make_auth_header("alice")
    r = await client.put(
        "/api/users/me/profile",
        json={"featured_badge_slugs": ["a", "b", "c", "d"]},
        headers=headers,
    )
    # Pydantic max_length=3 → 422.
    assert r.status_code == 422


@pytest.mark.asyncio
async def test_put_profile_dedupes_featured_badges_case_insensitive(client) -> None:
    headers = make_auth_header("alice")
    r = await client.put(
        "/api/users/me/profile",
        json={"featured_badge_slugs": ["FIRST-Project", "first-project", "five-projects"]},
        headers=headers,
    )
    assert r.status_code == 200
    saved = r.json()["featured_badge_slugs"]
    # Dedup preserves insertion order of the first occurrence.
    assert saved == ["FIRST-Project", "five-projects"]


# --- GET /api/users/{username}/activity ---------------------------------


@pytest.mark.asyncio
async def test_activity_feed_records_project_create(client) -> None:
    headers = make_auth_header("alice")
    p = await client.post(
        "/api/projects",
        json={"title": "Alice's First"},
        headers=headers,
    )
    pid = p.json()["id"]
    r = await client.get("/api/users/alice/activity")
    assert r.status_code == 200
    items = r.json()["items"]
    assert len(items) >= 1
    # Most recent first.
    assert items[0]["subject_id"] == pid
    assert items[0]["kind"] in {"project_updated", "project_published"}
    assert "/projects/view.html?id=" in (items[0]["subject_url"] or "")


@pytest.mark.asyncio
async def test_activity_feed_distinguishes_publish_from_update(client) -> None:
    headers = make_auth_header("alice")
    p = await client.post(
        "/api/projects",
        json={"title": "Alice's First"},
        headers=headers,
    )
    pid = p.json()["id"]
    # Flip to completed → counts as publish.
    await client.put(
        f"/api/projects/{pid}",
        json={"status": "completed"},
        headers=headers,
    )
    r = await client.get("/api/users/alice/activity")
    kinds = [i["kind"] for i in r.json()["items"]]
    assert "project_published" in kinds


@pytest.mark.asyncio
async def test_activity_feed_pagination(client) -> None:
    headers = make_auth_header("alice")
    for i in range(5):
        await client.post(
            "/api/projects",
            json={"title": f"Project {i}"},
            headers=headers,
        )
    r1 = await client.get("/api/users/alice/activity?limit=2&offset=0")
    r2 = await client.get("/api/users/alice/activity?limit=2&offset=2")
    assert r1.status_code == 200 and r2.status_code == 200
    ids1 = [i["id"] for i in r1.json()["items"]]
    ids2 = [i["id"] for i in r2.json()["items"]]
    assert len(ids1) == 2 and len(ids2) == 2
    assert set(ids1).isdisjoint(set(ids2))


# --- followers / following lists ----------------------------------------


@pytest.mark.asyncio
async def test_followers_list(client) -> None:
    await client.post("/api/users/bob/follow", headers=make_auth_header("alice"))
    await client.post("/api/users/bob/follow", headers=make_auth_header("carol"))
    r = await client.get("/api/users/bob/followers")
    assert r.status_code == 200
    body = r.json()
    assert body["total"] == 2
    usernames = {u["username"] for u in body["users"]}
    assert usernames == {"alice", "carol"}


@pytest.mark.asyncio
async def test_following_list(client) -> None:
    headers = make_auth_header("alice")
    await client.post("/api/users/bob/follow", headers=headers)
    await client.post("/api/users/carol/follow", headers=headers)
    r = await client.get("/api/users/alice/following")
    assert r.status_code == 200
    body = r.json()
    assert body["total"] == 2
    usernames = {u["username"] for u in body["users"]}
    assert usernames == {"bob", "carol"}


@pytest.mark.asyncio
async def test_followers_list_pagination(client) -> None:
    for u in ("a1", "a2", "a3"):
        await client.post(
            "/api/users/bob/follow", headers=make_auth_header(u)
        )
    r = await client.get("/api/users/bob/followers?limit=2&offset=0")
    body = r.json()
    assert body["total"] == 3
    assert len(body["users"]) == 2


# --- me/follows/{username} probe ----------------------------------------


@pytest.mark.asyncio
async def test_follow_check_requires_auth(client) -> None:
    r = await client.get("/api/users/me/follows/bob")
    assert r.status_code == 401


@pytest.mark.asyncio
async def test_follow_check_reflects_follow_state(client) -> None:
    headers = make_auth_header("alice")
    r = await client.get("/api/users/me/follows/bob", headers=headers)
    assert r.status_code == 200 and r.json() == {"following": False}
    await client.post("/api/users/bob/follow", headers=headers)
    r2 = await client.get("/api/users/me/follows/bob", headers=headers)
    assert r2.json() == {"following": True}


# --- boost_followed on the list endpoint --------------------------------
#
# A duplicate of the follow-router test that lives in test_follows.py.
# Re-asserting here so that any regression to the boosted-listing flow
# is caught even if the follow-router test file is restructured.


@pytest.mark.asyncio
async def test_boost_followed_reorders_for_authed_viewer(client) -> None:
    p_bob = (await client.post(
        "/api/projects", json={"title": "Bob's Robot"},
        headers=make_auth_header("bob"),
    )).json()
    p_dave = (await client.post(
        "/api/projects", json={"title": "Dave's Drone"},
        headers=make_auth_header("dave"),
    )).json()
    # Carol follows Dave only.
    carol = make_auth_header("carol")
    await client.post("/api/users/dave/follow", headers=carol)

    boosted = await client.get(
        "/api/projects?boost_followed=true", headers=carol
    )
    ids = [p["id"] for p in boosted.json()]
    assert ids.index(p_dave["id"]) < ids.index(p_bob["id"])


@pytest.mark.asyncio
async def test_boost_followed_ignored_for_anonymous(client) -> None:
    # boost_followed=true on an anonymous call must not 500.
    await client.post(
        "/api/projects", json={"title": "Bob's Robot"},
        headers=make_auth_header("bob"),
    )
    r = await client.get("/api/projects?boost_followed=true")
    assert r.status_code == 200
