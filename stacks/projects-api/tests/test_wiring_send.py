"""Snakie's "Send to kevsrobots" (kevsrobots.com#582) and app tokens from
"Sign in with kevsrobots" (Chatter#80)."""

from __future__ import annotations

import time

import pytest
from jose import jwt

from .conftest import make_auth_header

ROBOT = {"name": "Rover", "board": "pico", "parts": [{"id": "led1", "type": "led"}], "connections": []}
SVG = '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 10 10"><rect width="5" height="5"/></svg>'


def app_header(scope: str = "profile projects:read projects:write", username: str = "testuser") -> dict:
    """A Chatter app token, as Snakie holds after signing in."""
    token = jwt.encode(
        {"sub": username, "client_id": "snakie", "scope": scope, "gid": 1, "exp": int(time.time()) + 3600},
        "test-secret",
        algorithm="HS256",
    )
    return {"Authorization": f"Bearer {token}"}


@pytest.fixture(autouse=True)
def _terms_ok(monkeypatch: pytest.MonkeyPatch) -> None:
    # The suite bypasses require_terms_accepted via dependency_overrides;
    # require_scope(terms=True) calls the shared check directly, so stub it too.
    from projects_api import auth as auth_module

    async def _accepted(session, user) -> None:
        return None

    monkeypatch.setattr(auth_module, "_check_terms_accepted", _accepted)


@pytest.fixture
async def project_id(client) -> int:
    resp = await client.post("/api/projects", json={"title": "Rover build"}, headers=make_auth_header())
    assert resp.status_code == 201, resp.text
    return resp.json()["id"]


@pytest.mark.asyncio
async def test_app_token_lists_my_projects(client, project_id) -> None:
    resp = await client.get("/api/projects/my/list", headers=app_header())
    assert resp.status_code == 200, resp.text
    assert [p["id"] for p in resp.json()] == [project_id]


@pytest.mark.asyncio
async def test_listing_needs_projects_read(client, project_id) -> None:
    resp = await client.get("/api/projects/my/list", headers=app_header("profile"))
    assert resp.status_code == 403
    assert "insufficient_scope" in resp.headers.get("www-authenticate", "")


@pytest.mark.asyncio
async def test_cookie_sign_in_still_lists(client, project_id) -> None:
    assert (await client.get("/api/projects/my/list", headers=make_auth_header())).status_code == 200


@pytest.mark.asyncio
async def test_send_wiring_saves_link_and_section(client, project_id) -> None:
    resp = await client.put(
        f"/api/projects/{project_id}/wiring/send",
        json={"data": ROBOT, "svg": SVG, "repo_url": "https://github.com/maker/rover"},
        headers=app_header(),
    )
    assert resp.status_code == 200, resp.text
    body = resp.json()
    assert body["project_id"] == project_id and body["title"] == "Rover build"
    assert body["svg_url"].endswith(f"/api/projects/{project_id}/wiring.svg")
    assert body["repo_url"] == "https://github.com/maker/rover"
    assert body["added_wiring_section"] is True
    assert body["url"] is None or body["url"].startswith("https://www.kevsrobots.com/projects/testuser/")

    wiring = (await client.get(f"/api/projects/{project_id}/wiring", headers=make_auth_header())).json()
    assert wiring["data"] == ROBOT
    project = (await client.get(f"/api/projects/{project_id}", headers=make_auth_header())).json()
    assert project["code_repo_url"] == "https://github.com/maker/rover"
    assert "## Wiring" in project["content_md"]
    assert "wiring.svg" in project["content_md"]


@pytest.mark.asyncio
async def test_sending_again_updates_instead_of_copying(client, project_id) -> None:
    url = f"/api/projects/{project_id}/wiring/send"
    first = await client.put(url, json={"data": ROBOT, "svg": SVG}, headers=app_header())
    assert first.json()["added_wiring_section"] is True
    changed = dict(ROBOT, name="Rover 2")
    second = await client.put(url, json={"data": changed, "svg": SVG}, headers=app_header())
    assert second.status_code == 200
    assert second.json()["added_wiring_section"] is False
    wiring = (await client.get(f"/api/projects/{project_id}/wiring", headers=make_auth_header())).json()
    assert wiring["data"]["name"] == "Rover 2"
    project = (await client.get(f"/api/projects/{project_id}", headers=make_auth_header())).json()
    assert project["content_md"].count("## Wiring") == 1


@pytest.mark.asyncio
async def test_send_without_picture_adds_no_section(client, project_id) -> None:
    resp = await client.put(f"/api/projects/{project_id}/wiring/send", json={"data": ROBOT}, headers=app_header())
    assert resp.status_code == 200
    assert resp.json()["svg_url"] is None and resp.json()["added_wiring_section"] is False


@pytest.mark.asyncio
async def test_send_needs_projects_write(client, project_id) -> None:
    resp = await client.put(
        f"/api/projects/{project_id}/wiring/send", json={"data": ROBOT}, headers=app_header("profile projects:read")
    )
    assert resp.status_code == 403


@pytest.mark.asyncio
async def test_send_only_to_your_own_project(client, project_id) -> None:
    resp = await client.put(
        f"/api/projects/{project_id}/wiring/send", json={"data": ROBOT}, headers=app_header(username="someoneelse")
    )
    assert resp.status_code == 403


@pytest.mark.asyncio
@pytest.mark.parametrize("repo_url", ["http://github.com/a/b", "https://evil.example/a/b", "https://github.com/onlyowner"])
async def test_repo_url_must_be_github(client, project_id, repo_url) -> None:
    resp = await client.put(
        f"/api/projects/{project_id}/wiring/send", json={"data": ROBOT, "repo_url": repo_url}, headers=app_header()
    )
    assert resp.status_code == 422


@pytest.mark.asyncio
async def test_bad_robot_is_refused(client, project_id) -> None:
    resp = await client.put(
        f"/api/projects/{project_id}/wiring/send", json={"data": {"parts": "nope"}}, headers=app_header()
    )
    assert resp.status_code == 422


@pytest.mark.asyncio
async def test_cookie_sign_in_can_send_too(client, project_id) -> None:
    resp = await client.put(f"/api/projects/{project_id}/wiring/send", json={"data": ROBOT}, headers=make_auth_header())
    assert resp.status_code == 200


@pytest.mark.asyncio
async def test_app_tokens_cant_use_other_routes(client, project_id) -> None:
    # Everything else takes a full sign-in only: an app can't edit or delete a project.
    assert (await client.put(f"/api/projects/{project_id}", json={"title": "Hacked"}, headers=app_header())).status_code == 403
    assert (await client.delete(f"/api/projects/{project_id}", headers=app_header())).status_code == 403
    assert (await client.put(f"/api/projects/{project_id}/wiring", json={"data": ROBOT}, headers=app_header())).status_code == 403


@pytest.mark.asyncio
async def test_app_token_reads_public_pages_as_a_visitor(client, project_id) -> None:
    # Optional sign-in routes treat an app token like no sign-in, rather than failing.
    resp = await client.get("/api/projects", headers=app_header())
    assert resp.status_code == 200
