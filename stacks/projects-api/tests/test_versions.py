"""Project versions (epic #433): publish, live reads, restore, delete."""

from __future__ import annotations

import pytest

from .conftest import make_auth_header

ALICE = make_auth_header("alice")
BOB = make_auth_header("bob")
PNG = b"\x89PNG\r\n\x1a\n" + b"\x00" * 64
SVG_A = ('<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 10 10">'
         '<rect width="10" height="10" style="fill: rgb(1, 2, 3)"/></svg>')
SVG_B = ('<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 10 10">'
         '<circle cx="5" cy="5" r="4" style="fill: rgb(4, 5, 6)"/></svg>')


async def _draft(client, title="Rover Mk1", **fields) -> dict:
    payload = {"title": title, "content_md": "First text", "status": "draft", "tags": ["rover"]}
    payload.update(fields)
    resp = await client.post("/api/projects", json=payload, headers=ALICE)
    assert resp.status_code == 201, resp.text
    return resp.json()


async def _put(client, pid, **fields) -> dict:
    resp = await client.put(f"/api/projects/{pid}", json=fields, headers=ALICE)
    assert resp.status_code == 200, resp.text
    return resp.json()


async def _publish(client, pid, **body) -> dict:
    resp = await client.post(f"/api/projects/{pid}/publish", json=body or None, headers=ALICE)
    assert resp.status_code == 200, resp.text
    return resp.json()


async def _versions(client, pid) -> dict:
    resp = await client.get(f"/api/projects/{pid}/versions", headers=ALICE)
    assert resp.status_code == 200, resp.text
    return resp.json()


# --- #434: publish, list, unpublish ------------------------------------------


@pytest.mark.asyncio
async def test_never_published_is_v0_with_no_versions(client) -> None:
    p = await _draft(client)
    assert p["live_number"] is None
    assert p["has_unpublished_changes"] is False
    assert p["published_at"] is None
    listing = await _versions(client, p["id"])
    assert listing == {"live_number": None, "has_unpublished_changes": False, "versions": []}


@pytest.mark.asyncio
async def test_publish_creates_v1_then_v2_and_numbers_are_never_reused(client) -> None:
    p = await _draft(client)
    pid = p["id"]
    first = await _publish(client, pid, note="First go")
    assert first["version"]["number"] == 1
    assert first["version"]["is_live"] is True
    assert first["version"]["note"] == "First go"
    assert first["project"]["status"] == "completed"
    assert first["project"]["live_number"] == 1
    assert first["project"]["has_unpublished_changes"] is False

    edited = await _put(client, pid, content_md="Second text")
    assert edited["has_unpublished_changes"] is True
    second = await _publish(client, pid)
    assert second["version"]["number"] == 2

    rows = (await _versions(client, pid))["versions"]
    assert [(r["number"], r["is_live"], r["kind"]) for r in rows] == [
        (2, True, "published"), (1, False, "published"),
    ]
    # Delete v1, publish again: v3, not v1 or v2 again.
    assert (await client.delete(f"/api/projects/{pid}/versions/1", headers=ALICE)).status_code == 204
    await _put(client, pid, content_md="Third text")
    assert (await _publish(client, pid))["version"]["number"] == 3


@pytest.mark.asyncio
async def test_publish_with_no_changes_is_409(client) -> None:
    p = await _draft(client)
    await _publish(client, p["id"])
    resp = await client.post(f"/api/projects/{p['id']}/publish", headers=ALICE)
    assert resp.status_code == 409
    # Tags count as a set: the same tags in another order is no change.
    await _put(client, p["id"], tags=["rover"])
    assert (await client.post(f"/api/projects/{p['id']}/publish", headers=ALICE)).status_code == 409
    # A save that brings the text back to the live text clears the flag.
    assert (await _put(client, p["id"], content_md="Changed"))["has_unpublished_changes"] is True
    assert (await _put(client, p["id"], content_md="First text"))["has_unpublished_changes"] is False


@pytest.mark.asyncio
async def test_versions_are_the_owners_alone(client) -> None:
    p = await _draft(client)
    pid = p["id"]
    await _publish(client, pid)
    assert (await client.post(f"/api/projects/{pid}/publish", headers=BOB)).status_code == 403
    assert (await client.get(f"/api/projects/{pid}/versions", headers=BOB)).status_code == 403
    assert (await client.get(f"/api/projects/{pid}/versions/1", headers=BOB)).status_code == 403
    assert (await client.post(f"/api/projects/{pid}/versions/1/restore", headers=BOB)).status_code == 403
    assert (await client.get(f"/api/projects/{pid}/versions")).status_code == 401


@pytest.mark.asyncio
async def test_live_version_cannot_be_deleted(client) -> None:
    p = await _draft(client)
    await _publish(client, p["id"])
    assert (await client.delete(f"/api/projects/{p['id']}/versions/1", headers=ALICE)).status_code == 409
    assert (await client.delete(f"/api/projects/{p['id']}/versions/9", headers=ALICE)).status_code == 404


@pytest.mark.asyncio
async def test_unpublish_keeps_versions_and_hides_the_project(client) -> None:
    p = await _draft(client)
    pid = p["id"]
    await _publish(client, pid)
    assert (await client.get(f"/api/projects/{pid}")).status_code == 200
    resp = await client.post(f"/api/projects/{pid}/unpublish", headers=ALICE)
    assert resp.status_code == 200
    assert resp.json()["status"] == "draft"
    assert resp.json()["live_number"] is None
    assert (await client.get(f"/api/projects/{pid}")).status_code == 404
    assert pid not in [x["id"] for x in (await client.get("/api/projects")).json()]
    listing = await _versions(client, pid)
    assert listing["live_number"] is None
    assert [r["number"] for r in listing["versions"]] == [1]
    # Publish again: v2 (the working copy didn't change, but nothing is live).
    assert (await _publish(client, pid))["version"]["number"] == 2


@pytest.mark.asyncio
async def test_archive_keeps_the_live_version(client) -> None:
    p = await _draft(client)
    pid = p["id"]
    await _publish(client, pid)
    resp = await client.post(f"/api/projects/{pid}/archive", headers=ALICE)
    assert resp.json()["status"] == "archived"
    assert resp.json()["live_number"] == 1
    assert pid not in [x["id"] for x in (await client.get("/api/projects")).json()]
    assert (await client.get(f"/api/projects/{pid}")).status_code == 200
    resp = await client.post(f"/api/projects/{pid}/unarchive", headers=ALICE)
    assert resp.json()["status"] == "completed"


@pytest.mark.asyncio
async def test_expected_updated_at_mismatch_is_409(client) -> None:
    p = await _draft(client)
    stamp = p["updated_at"]
    fresh = await _put(client, p["id"], content_md="Tab A", expected_updated_at=stamp)
    resp = await client.put(
        f"/api/projects/{p['id']}", json={"content_md": "Tab B", "expected_updated_at": stamp}, headers=ALICE
    )
    assert resp.status_code == 409
    assert resp.json()["detail"] == "stale"
    await _put(client, p["id"], content_md="Tab A again", expected_updated_at=fresh["updated_at"])


@pytest.mark.asyncio
async def test_old_status_route_still_publishes(client) -> None:
    p = await _draft(client)
    resp = await _put(client, p["id"], status="completed")
    assert resp["live_number"] == 1
    # Again with nothing changed: fine, no new version.
    resp = await _put(client, p["id"], status="completed")
    assert resp["live_number"] == 1
    assert len((await _versions(client, p["id"]))["versions"]) == 1
    resp = await _put(client, p["id"], status="draft")
    assert resp["status"] == "draft" and resp["live_number"] is None


@pytest.mark.asyncio
async def test_publish_logs_activity_once_per_version(client) -> None:
    p = await _draft(client)
    pid = p["id"]
    await _publish(client, pid)
    await _put(client, pid, content_md="More")
    await _put(client, pid, content_md="More still")
    await _publish(client, pid)
    feed = (await client.get("/api/users/alice/activity")).json()["items"]
    kinds = [a["kind"] for a in feed if a.get("subject_id") == pid]
    assert sorted(kinds) == ["project_published", "project_updated"]


# --- #435: readers see the live version --------------------------------------


@pytest.mark.asyncio
async def test_stranger_sees_live_text_while_author_edits(client) -> None:
    p = await _draft(client)
    pid = p["id"]
    v1 = await _publish(client, pid)
    await _put(client, pid, content_md="Half-finished rewrite", short_description="WIP desc", tags=["secret"])

    public = (await client.get(f"/api/projects/{pid}")).json()
    assert public["content_md"] == "First text"
    assert public["tags"] == ["rover"]
    assert public["live_number"] is None and public["has_unpublished_changes"] is None
    assert public["viewing"] == "live"

    own = (await client.get(f"/api/projects/{pid}", headers=ALICE)).json()
    assert own["content_md"] == "Half-finished rewrite"
    assert own["live_number"] == 1 and own["has_unpublished_changes"] is True

    by_slug = (await client.get(f"/api/projects/by-slug/alice/{p['slug']}")).json()
    assert by_slug["content_md"] == "First text"
    # The author asking for the live version gets it, uncached.
    resp = await client.get(f"/api/projects/{pid}?version=live", headers=ALICE)
    assert resp.json()["content_md"] == "First text"
    assert resp.json()["has_unpublished_changes"] is True
    assert resp.headers["cache-control"] == "private, no-store"

    v2 = await _publish(client, pid)
    public = (await client.get(f"/api/projects/{pid}")).json()
    assert public["content_md"] == "Half-finished rewrite"
    # Readers get dates, not version numbers: first publish, and this one.
    assert public["published_at"] == v1["version"]["published_at"]
    assert public["updated_at"] == v2["version"]["published_at"]


@pytest.mark.asyncio
async def test_version_query_is_the_authors_alone(client) -> None:
    p = await _draft(client)
    pid = p["id"]
    await _publish(client, pid)
    await _put(client, pid, content_md="Working text")
    for q in ("working", "1"):
        assert (await client.get(f"/api/projects/{pid}?version={q}")).status_code == 404
        assert (await client.get(f"/api/projects/{pid}?version={q}", headers=BOB)).status_code == 404
        own = await client.get(f"/api/projects/{pid}?version={q}", headers=ALICE)
        assert own.status_code == 200
        assert own.headers["cache-control"] == "private, no-store"
    assert (await client.get(f"/api/projects/{pid}?version=working", headers=ALICE)).json()["content_md"] == "Working text"
    assert (await client.get(f"/api/projects/{pid}?version=1", headers=ALICE)).json()["content_md"] == "First text"
    assert (await client.get(f"/api/projects/{pid}?version=7", headers=ALICE)).status_code == 404
    snap = (await client.get(f"/api/projects/{pid}/versions/1", headers=ALICE)).json()
    assert snap["snapshot"]["content_md"] == "First text"


@pytest.mark.asyncio
async def test_title_edit_keeps_the_slug_until_publish(client) -> None:
    p = await _draft(client)
    pid = p["id"]
    # Never published: the slug still follows the title.
    p = await _put(client, pid, title="Rover Mark One")
    assert p["slug"] == "rover-mark-one"
    await _publish(client, pid)
    p = await _put(client, pid, title="Rover Mark Two")
    assert p["slug"] == "rover-mark-one"
    public = (await client.get("/api/projects/by-slug/alice/rover-mark-one")).json()
    assert public["title"] == "Rover Mark One"
    await _publish(client, pid)
    assert (await client.get(f"/api/projects/{pid}")).json()["slug"] == "rover-mark-two"
    old = await client.get("/api/projects/by-slug/alice/rover-mark-one", follow_redirects=False)
    assert old.status_code == 301
    assert old.headers["location"].endswith("/rover-mark-two")


@pytest.mark.asyncio
async def test_hub_search_and_cards_show_the_live_version(client) -> None:
    p = await _draft(client, title="Balancing Robot")
    pid = p["id"]
    await _put(client, pid, cover_image="https://x/a.png")
    await _publish(client, pid)
    await _put(client, pid, title="Secret New Name", cover_image="https://x/b.png", difficulty="advanced")

    hub = {x["id"]: x for x in (await client.get("/api/projects")).json()}
    assert hub[pid]["title"] == "Balancing Robot"
    assert hub[pid]["cover_image"] == "https://x/a.png"
    found = [x["id"] for x in (await client.get("/api/projects?search=Balancing")).json()]
    assert pid in found
    assert pid not in [x["id"] for x in (await client.get("/api/projects?search=Secret")).json()]
    assert pid not in [x["id"] for x in (await client.get("/api/projects?difficulty=advanced")).json()]
    assert pid in [x["id"] for x in (await client.get("/api/projects?tag=rover")).json()]

    mine = {x["id"]: x for x in (await client.get("/api/projects/my/list", headers=ALICE)).json()}
    assert mine[pid]["title"] == "Secret New Name"
    assert mine[pid]["live_number"] == 1 and mine[pid]["has_unpublished_changes"] is True

    og = await client.get(f"/og/alice/{p['slug']}")
    assert "Balancing Robot" in og.text and "Secret New Name" not in og.text


@pytest.mark.asyncio
async def test_remix_copies_the_live_version(client) -> None:
    p = await _draft(client)
    pid = p["id"]
    await _publish(client, pid)
    await _put(client, pid, content_md="Unpublished idea")
    remix = await client.post(
        f"/api/projects/{pid}/remix", json={"remix_description": "Bigger wheels for grass"}, headers=BOB
    )
    assert remix.status_code == 201, remix.text
    assert remix.json()["content_md"] == "First text"
    assert remix.json()["remixed_from"]["title"] == "Rover Mk1"


@pytest.mark.asyncio
async def test_sub_resources_serve_the_live_version(client) -> None:
    p = await _draft(client)
    pid = p["id"]
    await client.post(f"/api/projects/{pid}/links", json={"title": "Docs", "url": "https://a"}, headers=ALICE)
    await client.post(f"/api/projects/{pid}/videos", json={"url_or_id": "https://youtu.be/dQw4w9WgXcQ"}, headers=ALICE)
    await client.post(f"/api/projects/{pid}/bom", json={"name": "Servo", "quantity": 2}, headers=ALICE)
    await _publish(client, pid)

    await client.post(f"/api/projects/{pid}/links", json={"title": "New", "url": "https://b"}, headers=ALICE)
    await client.put(f"/api/projects/{pid}/bom/sync", json={"rows": [{"name": "Motor", "quantity": 4}]},
                     headers=ALICE)

    links = (await client.get(f"/api/projects/{pid}/links")).json()
    assert [x["title"] for x in links] == ["Docs"]
    bom = (await client.get(f"/api/projects/{pid}/bom")).json()
    assert [(x["name"], x["quantity"]) for x in bom] == [("Servo", 2)]
    bom_slug = (await client.get(f"/api/projects/alice/{p['slug']}/bom")).json()
    assert [x["name"] for x in bom_slug] == ["Servo"]
    videos = (await client.get(f"/api/projects/{pid}/videos")).json()
    assert [x["youtube_id"] for x in videos] == ["dQw4w9WgXcQ"]

    own_bom = (await client.get(f"/api/projects/{pid}/bom", headers=ALICE)).json()
    assert [x["name"] for x in own_bom] == ["Motor"]
    own_links = (await client.get(f"/api/projects/{pid}/links", headers=ALICE)).json()
    assert [x["title"] for x in own_links] == ["Docs", "New"]


@pytest.mark.asyncio
async def test_unpublished_image_is_private_until_published(client) -> None:
    p = await _draft(client)
    pid = p["id"]
    await _publish(client, pid)
    img = (await client.post(f"/api/projects/{pid}/images", files={"file": ("new.png", PNG, "image/png")},
                             headers=ALICE)).json()
    url = f"/api/projects/{pid}/images/{img['id']}/view"
    assert (await client.get(url)).status_code == 404
    own = await client.get(url, headers=ALICE)
    assert own.status_code == 200 and own.headers["cache-control"] == "private, no-store"
    assert img["id"] not in [i["id"] for i in (await client.get(f"/api/projects/{pid}/images")).json()]
    await _put(client, pid, content_md=f"![pic]({url})")
    await _publish(client, pid)
    resp = await client.get(url)
    assert resp.status_code == 200 and "cache-control" not in resp.headers
    assert img["id"] in [i["id"] for i in (await client.get(f"/api/projects/{pid}/images")).json()]


# --- #439: pictures and the shared pool --------------------------------------


@pytest.mark.asyncio
async def test_wiring_picture_is_frozen_per_publish(client) -> None:
    p = await _draft(client)
    pid = p["id"]
    await client.put(f"/api/projects/{pid}/wiring", json={"data": {"parts": [1]}, "svg": SVG_A}, headers=ALICE)
    await _publish(client, pid)
    await client.put(f"/api/projects/{pid}/wiring", json={"data": {"parts": [2]}, "svg": SVG_B}, headers=ALICE)

    public = await client.get(f"/api/projects/{pid}/wiring.svg")
    assert b"<rect" in public.content
    assert (await client.get(f"/api/projects/{pid}/wiring")).json()["data"] == {"parts": [1]}
    own = await client.get(f"/api/projects/{pid}/wiring.svg", headers=ALICE)
    assert b"<circle" in own.content and own.headers["cache-control"] == "private, no-store"
    old = await client.get(f"/api/projects/{pid}/wiring.svg?version=1", headers=ALICE)
    assert b"<rect" in old.content

    await _publish(client, pid)
    assert b"<circle" in (await client.get(f"/api/projects/{pid}/wiring.svg")).content
    # Editing again changes nothing public until v3.
    await client.put(f"/api/projects/{pid}/wiring", json={"data": {"parts": [3]}, "svg": SVG_A}, headers=ALICE)
    assert b"<circle" in (await client.get(f"/api/projects/{pid}/wiring.svg")).content


@pytest.mark.asyncio
async def test_unchanged_diagram_is_stored_once(client, sessionmaker_) -> None:
    from projects_api.models import ProjectVersion
    from sqlalchemy import select

    p = await _draft(client)
    pid = p["id"]
    d = (await client.post(f"/api/projects/{pid}/diagrams", json={"title": "Exploded"}, headers=ALICE)).json()
    await client.put(f"/api/projects/{pid}/diagrams/{d['id']}", json={"svg": SVG_A}, headers=ALICE)
    await _publish(client, pid)
    await _put(client, pid, content_md="Only the text changed")
    await _publish(client, pid)
    async with sessionmaker_() as session:
        rows = (await session.scalars(select(ProjectVersion).where(ProjectVersion.project_id == pid))).all()
        paths = {r.snapshot["diagrams"][str(d["id"])]["svg_path"] for r in rows}
    assert len(paths) == 1 and "projects/versions/" in paths.pop()

    # The live diagram survives the working copy deleting it.
    await client.delete(f"/api/projects/{pid}/diagrams/{d['id']}", headers=ALICE)
    assert (await client.get(f"/api/projects/{pid}/diagrams/{d['id']}.svg")).status_code == 200
    assert [x["id"] for x in (await client.get(f"/api/projects/{pid}/diagrams")).json()] == [d["id"]]
    assert (await client.get(f"/api/projects/{pid}/diagrams", headers=ALICE)).json() == []


@pytest.mark.asyncio
async def test_removing_a_versioned_image_is_soft(client) -> None:
    p = await _draft(client)
    pid = p["id"]
    up = lambda name: client.post(f"/api/projects/{pid}/images", files={"file": (name, PNG, "image/png")},
                                  headers=ALICE)
    live_img = (await up("live.png")).json()
    await _publish(client, pid)
    fresh = (await up("fresh.png")).json()

    # Used by the live page: leaves the working gallery, stays live.
    assert (await client.delete(f"/api/projects/{pid}/images/{live_img['id']}", headers=ALICE)).status_code == 204
    own = [i["id"] for i in (await client.get(f"/api/projects/{pid}/images", headers=ALICE)).json()]
    assert own == [fresh["id"]]
    assert (await client.get(f"/api/projects/{pid}/images/{live_img['id']}/view")).status_code == 200
    # Used by no version: gone for good.
    assert (await client.delete(f"/api/projects/{pid}/images/{fresh['id']}", headers=ALICE)).status_code == 204
    assert (await client.get(f"/api/projects/{pid}/images/{fresh['id']}/view", headers=ALICE)).status_code == 404

    # Restoring v1 brings the removed picture back to the gallery.
    await client.post(f"/api/projects/{pid}/versions/1/restore", headers=ALICE)
    own = [i["id"] for i in (await client.get(f"/api/projects/{pid}/images", headers=ALICE)).json()]
    assert own == [live_img["id"]]


# --- #437: restore -----------------------------------------------------------


@pytest.mark.asyncio
async def test_restore_saves_the_working_copy_first(client) -> None:
    p = await _draft(client)
    pid = p["id"]
    await client.put(f"/api/projects/{pid}/wiring", json={"data": {"parts": ["v1"]}, "svg": SVG_A}, headers=ALICE)
    d = (await client.post(f"/api/projects/{pid}/diagrams", json={"title": "One", "data": {"objects": [1]}},
                           headers=ALICE)).json()
    await _publish(client, pid)
    slug = (await client.get(f"/api/projects/{pid}", headers=ALICE)).json()["slug"]

    await _put(client, pid, title="Rover Rewritten", content_md="Typed a moment ago", tags=["new"])
    await client.put(f"/api/projects/{pid}/wiring", json={"data": {"parts": ["v2"]}, "svg": SVG_B}, headers=ALICE)
    await client.put(f"/api/projects/{pid}/diagrams/{d['id']}", json={"data": {"objects": [2]}}, headers=ALICE)

    resp = await client.post(f"/api/projects/{pid}/versions/1/restore", headers=ALICE)
    assert resp.status_code == 200, resp.text
    body = resp.json()
    assert body["restored"] == 1 and body["saved_as"] == 2
    project = body["project"]
    assert project["content_md"] == "First text" and project["title"] == "Rover Mk1"
    assert project["tags"] == ["rover"]
    assert project["live_number"] == 1 and project["has_unpublished_changes"] is False
    assert project["slug"] == slug

    own_wiring = (await client.get(f"/api/projects/{pid}/wiring", headers=ALICE)).json()
    assert own_wiring["data"] == {"parts": ["v1"]}
    assert b"<rect" in (await client.get(f"/api/projects/{pid}/wiring.svg", headers=ALICE)).content
    own_d = (await client.get(f"/api/projects/{pid}/diagrams/{d['id']}", headers=ALICE)).json()
    assert own_d["data"] == {"objects": [1]}

    rows = (await _versions(client, pid))["versions"]
    assert rows[0]["number"] == 2 and rows[0]["kind"] == "unpublished"
    assert rows[0]["note"] == "Saved before restoring v1"
    assert rows[0]["title"] == "Rover Rewritten"

    # The typed text is in the saved version; restoring it gets it back.
    back = (await client.post(f"/api/projects/{pid}/versions/2/restore", headers=ALICE)).json()
    assert back["saved_as"] is None  # the working copy was v1, already kept
    assert back["project"]["content_md"] == "Typed a moment ago"
    assert (await client.get(f"/api/projects/{pid}/wiring", headers=ALICE)).json()["data"] == {"parts": ["v2"]}
    # And nothing public moved.
    assert (await client.get(f"/api/projects/{pid}")).json()["content_md"] == "First text"
    assert b"<rect" in (await client.get(f"/api/projects/{pid}/wiring.svg")).content


@pytest.mark.asyncio
async def test_restore_twice_adds_one_saved_version(client) -> None:
    p = await _draft(client)
    pid = p["id"]
    await _publish(client, pid)
    await _put(client, pid, content_md="v2 text")
    await _publish(client, pid)
    await _put(client, pid, content_md="unsaved idea")
    first = (await client.post(f"/api/projects/{pid}/versions/1/restore", headers=ALICE)).json()
    second = (await client.post(f"/api/projects/{pid}/versions/1/restore", headers=ALICE)).json()
    assert first["saved_as"] == 3 and second["saved_as"] is None
    assert [r["number"] for r in (await _versions(client, pid))["versions"]] == [3, 2, 1]
    assert (await client.post(f"/api/projects/{pid}/versions/9/restore", headers=ALICE)).status_code == 404


@pytest.mark.asyncio
async def test_restore_makes_a_stale_tab_409(client) -> None:
    p = await _draft(client)
    pid = p["id"]
    await _publish(client, pid)
    tab_b = await _put(client, pid, content_md="Tab B text")
    await client.post(f"/api/projects/{pid}/versions/1/restore", headers=ALICE)
    resp = await client.put(
        f"/api/projects/{pid}",
        json={"content_md": "Tab B autosave", "expected_updated_at": tab_b["updated_at"]},
        headers=ALICE,
    )
    assert resp.status_code == 409


# --- Backfill -----------------------------------------------------------------


@pytest.mark.asyncio
async def test_backfill_gives_public_projects_a_live_v1(client, sessionmaker_, monkeypatch) -> None:
    from sqlalchemy import delete

    from projects_api import db
    from projects_api.models import AppliedMigration, Project, ProjectVersion

    monkeypatch.setattr(db, "get_sessionmaker", lambda: sessionmaker_)
    old_public = (await client.post("/api/projects", json={"title": "Old Public One"}, headers=ALICE)).json()
    draft = await _draft(client, title="Quiet Draft")
    async with sessionmaker_() as session:
        await session.execute(delete(AppliedMigration).where(AppliedMigration.name == "434-published-versions"))
        await session.commit()

    await db.backfill_published_versions()
    async with sessionmaker_() as session:
        row = await session.get(Project, old_public["id"])
        assert row.status == "completed" and row.published_version_id is not None
        assert (await session.get(Project, draft["id"])).published_version_id is None
        live = await session.get(ProjectVersion, row.published_version_id)
        assert live.number == 1 and live.snapshot["title"] == "Old Public One"

    await db.backfill_published_versions()  # recorded: a second run does nothing
    async with sessionmaker_() as session:
        from sqlalchemy import func, select

        count = await session.scalar(select(func.count(ProjectVersion.id)))
        assert count == 1


@pytest.mark.asyncio
async def test_snapshot_round_trip(client, sessionmaker_) -> None:
    from projects_api import versions as v
    from projects_api.models import Project

    p = await _draft(client)
    pid = p["id"]
    await client.put(f"/api/projects/{pid}/wiring", json={"data": {"parts": ["a"]}, "svg": SVG_A}, headers=ALICE)
    await client.post(f"/api/projects/{pid}/diagrams", json={"title": "D", "data": {"objects": [9]}}, headers=ALICE)
    async with sessionmaker_() as session:
        project = await session.get(Project, pid)
        snap = await v.snapshot_project(session, project)
    await _put(client, pid, title="Something Else", content_md="Other", tags=["x", "y"])
    await client.put(f"/api/projects/{pid}/wiring", json={"data": {"parts": ["b"]}}, headers=ALICE)
    async with sessionmaker_() as session:
        project = await session.get(Project, pid)
        await v.apply_snapshot(session, project, snap)
        await session.commit()
        project = await session.get(Project, pid)
        assert v.same_content(await v.snapshot_project(session, project), snap)
