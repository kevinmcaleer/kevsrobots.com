"""Layers graphic renders (graphics.py): keep the draft's and the versions'."""

from __future__ import annotations

from datetime import datetime, timedelta

import pytest
from sqlalchemy import update

from projects_api import db
from projects_api.models import ProjectImage

from .conftest import make_auth_header

ALICE = make_auth_header("alice")
PNG = b"\x89PNG\r\n\x1a\n" + b"\x00" * 64


async def _draft(client, content="Text") -> int:
    resp = await client.post("/api/projects", json={"title": "Open Duck", "content_md": content, "status": "draft"},
                             headers=ALICE)
    assert resp.status_code == 201, resp.text
    return resp.json()["id"]


async def _upload(client, pid, name="step-1.layers.png", replaces=None) -> int:
    data = {"replaces": str(replaces)} if replaces is not None else None
    resp = await client.post(f"/api/projects/{pid}/images", files={"file": (name, PNG, "image/png")},
                             data=data, headers=ALICE)
    assert resp.status_code == 201, resp.text
    return resp.json()["id"]


def _md(pid, *ids) -> str:
    return "\n\n".join(f"![Step](/api/projects/{pid}/images/{i}/view#layers)" for i in ids)


async def _put(client, pid, content) -> None:
    resp = await client.put(f"/api/projects/{pid}", json={"content_md": content}, headers=ALICE)
    assert resp.status_code == 200, resp.text


async def _publish(client, pid) -> None:
    resp = await client.post(f"/api/projects/{pid}/publish", headers=ALICE)
    assert resp.status_code == 200, resp.text


async def _age(*ids) -> None:
    """Past the hour left for undo."""
    async with db.get_sessionmaker()() as s:
        await s.execute(update(ProjectImage).where(ProjectImage.id.in_(ids))
                        .values(superseded_at=datetime(2026, 1, 1)))
        await s.commit()


async def _gallery(client, pid) -> list[int]:
    return [i["id"] for i in (await client.get(f"/api/projects/{pid}/images", headers=ALICE)).json()]


async def _exists(client, pid, iid) -> bool:
    return (await client.get(f"/api/projects/{pid}/images/{iid}/view", headers=ALICE)).status_code == 200


@pytest.mark.asyncio
async def test_superseded_draft_render_leaves_gallery_and_is_deleted(client) -> None:
    pid = await _draft(client)
    a = await _upload(client, pid)
    await _put(client, pid, _md(pid, a))
    b = await _upload(client, pid, replaces=a)
    assert await _gallery(client, pid) == [b]  # one picture per graphic
    await _put(client, pid, _md(pid, b))
    assert await _exists(client, pid, a)  # still within the hour for undo
    await _age(a)
    await _put(client, pid, _md(pid, b) + "\n\nmore")
    assert not await _exists(client, pid, a)
    assert await _exists(client, pid, b)


@pytest.mark.asyncio
async def test_published_render_is_kept_as_history(client) -> None:
    pid = await _draft(client)
    a = await _upload(client, pid)
    await _put(client, pid, _md(pid, a))
    await _publish(client, pid)
    b = await _upload(client, pid, replaces=a)
    c = await _upload(client, pid, replaces=b)
    await _put(client, pid, _md(pid, c))
    await _age(a, b)
    await _put(client, pid, _md(pid, c) + "\n")
    # The draft in between is gone; v1's stays, live, out of the gallery.
    assert not await _exists(client, pid, b)
    assert (await client.get(f"/api/projects/{pid}/images/{a}/view")).status_code == 200
    assert await _gallery(client, pid) == [c]
    hist = (await client.get(f"/api/projects/{pid}/images/history", headers=ALICE)).json()
    assert [e["id"] for e in hist[str(c)]] == [a]
    assert hist[str(c)][0]["versions"][0]["number"] == 1
    assert hist[str(c)][0]["versions"][0]["live"] is True


@pytest.mark.asyncio
async def test_render_still_in_the_text_is_kept(client) -> None:
    pid = await _draft(client)
    a = await _upload(client, pid)
    b = await _upload(client, pid, replaces=a)
    await _age(a)
    await _put(client, pid, _md(pid, a, b))  # the same graphic twice, one updated
    assert await _exists(client, pid, a)


@pytest.mark.asyncio
async def test_superseded_render_not_snapshotted(client) -> None:
    pid = await _draft(client)
    a = await _upload(client, pid)
    b = await _upload(client, pid, replaces=a)
    await _put(client, pid, _md(pid, b))
    await _publish(client, pid)
    public = [i["id"] for i in (await client.get(f"/api/projects/{pid}/images")).json()]
    assert public == [b]


@pytest.mark.asyncio
async def test_tidy_clears_old_piles_but_keeps_what_versions_show(client) -> None:
    pid = await _draft(client)
    live = await _upload(client, pid)
    photo = await _upload(client, pid, name="photo.jpg")
    await _put(client, pid, _md(pid, live) + "\n\n::: gallery\n:::\n")
    await _publish(client, pid)
    current = await _upload(client, pid)
    old = [await _upload(client, pid) for _ in range(3)]
    unsaved = await _upload(client, pid)
    await _put(client, pid, _md(pid, current))
    url = f"/api/projects/{pid}/images/tidy-graphics"
    body = {"text": _md(pid, current, unsaved)}
    preview = (await client.post(url + "?dry_run=true", json=body, headers=ALICE)).json()
    assert preview["deleted"] == 3 and preview["kept"] == 1
    assert sorted(await _gallery(client, pid)) == sorted([live, photo, current, unsaved, *old])
    done = (await client.post(url, json=body, headers=ALICE)).json()
    assert done["deleted_ids"] == sorted(old) and done["kept_ids"] == [live]
    assert sorted(await _gallery(client, pid)) == sorted([photo, current, unsaved])
    # The live page still shows its picture.
    assert (await client.get(f"/api/projects/{pid}/images/{live}/view")).status_code == 200
    for i in old:
        assert not await _exists(client, pid, i)


@pytest.mark.asyncio
async def test_tidy_keeps_unplaced_pictures_a_version_without_gallery_shows(client) -> None:
    pid = await _draft(client)
    shown = await _upload(client, pid)  # not in the text, but the page lists it below
    await _publish(client, pid)
    await _put(client, pid, "New text")
    done = (await client.post(f"/api/projects/{pid}/images/tidy-graphics", headers=ALICE)).json()
    assert done["kept_ids"] == [shown] and done["deleted"] == 0
    assert (await client.get(f"/api/projects/{pid}/images/{shown}/view")).status_code == 200


@pytest.mark.asyncio
async def test_replaces_another_projects_picture_is_ignored(client) -> None:
    p1 = await _draft(client)
    p2 = await _draft(client)
    a = await _upload(client, p1)
    await _upload(client, p2, replaces=a)
    assert await _gallery(client, p1) == [a]


@pytest.mark.asyncio
async def test_history_and_tidy_are_the_authors_alone(client) -> None:
    pid = await _draft(client)
    bob = make_auth_header("bob")
    assert (await client.get(f"/api/projects/{pid}/images/history", headers=bob)).status_code == 403
    assert (await client.post(f"/api/projects/{pid}/images/tidy-graphics", headers=bob)).status_code == 403
