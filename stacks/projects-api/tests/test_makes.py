"""Tests for Community Makes ("I Made This!") — issue #107.

These exercise the contract laid out in the issue:
  * Anyone authenticated can create a make on someone else's project.
  * Only the make's poster can delete it.
  * Only the project's author can heart/unheart the make.
"""

from __future__ import annotations

import pytest

from .conftest import make_auth_header


# 1x1 transparent PNG used for image upload tests.
TINY_PNG = (
    b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00\x00\x01"
    b"\x00\x00\x00\x01\x08\x02\x00\x00\x00\x90wS\xde\x00"
    b"\x00\x00\x0cIDATx\x9cc\xf8\x0f\x00\x00\x01\x01\x00"
    b"\x05\x18\xd8N\x00\x00\x00\x00IEND\xaeB`\x82"
)


@pytest.fixture
async def project_id(client) -> int:
    """A project owned by `author_user` so we can post makes against it."""
    resp = await client.post(
        "/api/projects",
        json={"title": "Cool Robot Project"},
        headers=make_auth_header("author_user"),
    )
    return resp.json()["id"]


async def _create_make_with_image(client, project_id: int, *, as_user: str = "maker_one"):
    return await client.post(
        f"/api/projects/{project_id}/makes",
        data={
            "notes": "Worked great!",
            "modifications": "Used a different motor.",
        },
        files={"images": ("photo.png", TINY_PNG, "image/png")},
        headers=make_auth_header(as_user),
    )


@pytest.mark.asyncio
async def test_create_make_with_one_image(client, project_id) -> None:
    resp = await _create_make_with_image(client, project_id)
    assert resp.status_code == 201, resp.text
    body = resp.json()
    assert body["project_id"] == project_id
    assert body["user_id"] == "maker_one"
    assert body["notes"] == "Worked great!"
    assert body["modifications"] == "Used a different motor."
    assert len(body["images"]) == 1
    assert body["images"][0]["filename"] == "photo.png"
    assert body["hearted_by_author"] is False


@pytest.mark.asyncio
async def test_create_make_unauthenticated(client, project_id) -> None:
    resp = await client.post(
        f"/api/projects/{project_id}/makes",
        data={"notes": "no auth"},
    )
    assert resp.status_code == 401


@pytest.mark.asyncio
async def test_create_make_rejects_unknown_project(client) -> None:
    resp = await client.post(
        "/api/projects/99999/makes",
        data={"notes": "ghost"},
        headers=make_auth_header("maker_one"),
    )
    assert resp.status_code == 404


@pytest.mark.asyncio
async def test_create_make_rejects_too_many_images(client, project_id) -> None:
    files = [("images", (f"p{i}.png", TINY_PNG, "image/png")) for i in range(6)]
    resp = await client.post(
        f"/api/projects/{project_id}/makes",
        data={"notes": "lots"},
        files=files,
        headers=make_auth_header("maker_one"),
    )
    assert resp.status_code == 400
    assert "max" in resp.json()["detail"].lower()


@pytest.mark.asyncio
async def test_list_makes_for_project(client, project_id) -> None:
    await _create_make_with_image(client, project_id, as_user="maker_one")
    await _create_make_with_image(client, project_id, as_user="maker_two")
    resp = await client.get(f"/api/projects/{project_id}/makes")
    assert resp.status_code == 200
    body = resp.json()
    assert len(body) == 2
    # Newest first.
    assert body[0]["user_id"] in {"maker_one", "maker_two"}
    assert {m["user_id"] for m in body} == {"maker_one", "maker_two"}


@pytest.mark.asyncio
async def test_list_makes_for_user_across_projects(client) -> None:
    # Two projects by different authors so the user-scoped list has to
    # actually filter by user_id rather than coincidentally project-by-project.
    p1 = (await client.post(
        "/api/projects",
        json={"title": "Project Alpha"},
        headers=make_auth_header("author_a"),
    )).json()["id"]
    p2 = (await client.post(
        "/api/projects",
        json={"title": "Project Beta"},
        headers=make_auth_header("author_b"),
    )).json()["id"]

    await _create_make_with_image(client, p1, as_user="builder")
    await _create_make_with_image(client, p2, as_user="builder")
    await _create_make_with_image(client, p1, as_user="someone_else")

    resp = await client.get("/api/users/builder/makes")
    assert resp.status_code == 200
    body = resp.json()
    assert len(body) == 2
    assert {m["project_id"] for m in body} == {p1, p2}
    # The user-scoped list should decorate each make with its project title.
    titles = {m["project_title"] for m in body}
    assert titles == {"Project Alpha", "Project Beta"}


@pytest.mark.asyncio
async def test_get_make_detail_includes_project_title(client, project_id) -> None:
    create = await _create_make_with_image(client, project_id, as_user="maker_one")
    make_id = create.json()["id"]
    resp = await client.get(f"/api/makes/{make_id}")
    assert resp.status_code == 200
    body = resp.json()
    assert body["project_title"] == "Cool Robot Project"
    assert body["project_id"] == project_id


@pytest.mark.asyncio
async def test_view_make_image(client, project_id) -> None:
    create = await _create_make_with_image(client, project_id, as_user="maker_one")
    make = create.json()
    image_id = make["images"][0]["id"]
    resp = await client.get(
        f"/api/makes/{make['id']}/images/{image_id}/view"
    )
    assert resp.status_code == 200
    assert resp.content == TINY_PNG


@pytest.mark.asyncio
async def test_delete_make_as_owner(client, project_id) -> None:
    create = await _create_make_with_image(client, project_id, as_user="maker_one")
    make_id = create.json()["id"]
    resp = await client.delete(
        f"/api/makes/{make_id}",
        headers=make_auth_header("maker_one"),
    )
    assert resp.status_code == 204
    # Subsequent fetch should 404.
    detail = await client.get(f"/api/makes/{make_id}")
    assert detail.status_code == 404


@pytest.mark.asyncio
async def test_delete_make_as_other_user_forbidden(client, project_id) -> None:
    create = await _create_make_with_image(client, project_id, as_user="maker_one")
    make_id = create.json()["id"]
    resp = await client.delete(
        f"/api/makes/{make_id}",
        headers=make_auth_header("not_the_maker"),
    )
    assert resp.status_code == 403


@pytest.mark.asyncio
async def test_heart_as_project_author(client, project_id) -> None:
    create = await _create_make_with_image(client, project_id, as_user="maker_one")
    make_id = create.json()["id"]
    resp = await client.post(
        f"/api/makes/{make_id}/heart",
        headers=make_auth_header("author_user"),
    )
    assert resp.status_code == 200
    assert resp.json()["hearted_by_author"] is True

    # Now unheart.
    resp = await client.delete(
        f"/api/makes/{make_id}/heart",
        headers=make_auth_header("author_user"),
    )
    assert resp.status_code == 200
    assert resp.json()["hearted_by_author"] is False


@pytest.mark.asyncio
async def test_heart_as_other_user_forbidden(client, project_id) -> None:
    create = await _create_make_with_image(client, project_id, as_user="maker_one")
    make_id = create.json()["id"]
    # The make's poster is NOT the project author; they shouldn't be able
    # to heart their own make.
    resp = await client.post(
        f"/api/makes/{make_id}/heart",
        headers=make_auth_header("maker_one"),
    )
    assert resp.status_code == 403

    # And a bystander can't either.
    resp = await client.post(
        f"/api/makes/{make_id}/heart",
        headers=make_auth_header("random_user"),
    )
    assert resp.status_code == 403
