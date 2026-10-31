"""Project diagrams, the freeform diagram tool (issue #270)."""

from __future__ import annotations

import base64

import pytest

from .conftest import make_auth_header

CANVAS = {"version": "6.4.0", "objects": [{"type": "Rect", "left": 10, "top": 10, "width": 50, "height": 20, "name": "Box"}]}
SVG = '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 50"><rect x="10" y="10" width="50" height="20"/></svg>'
PNG = "data:image/png;base64," + base64.b64encode(b"\x89PNG\r\n\x1a\n" + b"\x00" * 16).decode()


@pytest.fixture
async def project_id(client) -> int:
    resp = await client.post("/api/projects", json={"title": "Annotated build"}, headers=make_auth_header())
    assert resp.status_code == 201, resp.text
    return resp.json()["id"]


@pytest.mark.asyncio
async def test_create_edit_export_delete(client, project_id) -> None:
    h = make_auth_header()
    resp = await client.post(f"/api/projects/{project_id}/diagrams", json={"title": "Wiring the arm", "data": CANVAS}, headers=h)
    assert resp.status_code == 201, resp.text
    d = resp.json()
    did = d["id"]
    assert d["data"] == CANVAS and d["svg_url"] is None
    assert (await client.get(f"/api/projects/{project_id}/diagrams/{did}.svg")).status_code == 404

    moved = {"version": "6.4.0", "objects": [dict(CANVAS["objects"][0], left=40)]}
    resp = await client.put(f"/api/projects/{project_id}/diagrams/{did}",
                            json={"data": moved, "svg": SVG, "png": PNG, "title": "Arm"}, headers=h)
    assert resp.status_code == 200, resp.text
    d = resp.json()
    assert d["title"] == "Arm" and d["data"]["objects"][0]["left"] == 40
    assert d["svg_url"].endswith(f"/diagrams/{did}.svg") and d["png_url"].endswith(f"/diagrams/{did}.png")

    svg = await client.get(f"/api/projects/{project_id}/diagrams/{did}.svg")
    assert svg.status_code == 200 and svg.headers["content-type"] == "image/svg+xml"
    assert "default-src 'none'" in svg.headers["content-security-policy"]
    png = await client.get(f"/api/projects/{project_id}/diagrams/{did}.png")
    assert png.status_code == 200 and png.content.startswith(b"\x89PNG")

    listed = (await client.get(f"/api/projects/{project_id}/diagrams")).json()
    assert [x["title"] for x in listed] == ["Arm"]
    assert "data" not in listed[0]

    assert (await client.delete(f"/api/projects/{project_id}/diagrams/{did}", headers=h)).status_code == 204
    assert (await client.get(f"/api/projects/{project_id}/diagrams/{did}")).status_code == 404


@pytest.mark.asyncio
async def test_new_diagram_starts_empty(client, project_id) -> None:
    resp = await client.post(f"/api/projects/{project_id}/diagrams", json={}, headers=make_auth_header())
    assert resp.status_code == 201
    assert resp.json()["data"]["objects"] == [] and resp.json()["title"] == "Diagram"


@pytest.mark.asyncio
async def test_owner_only_and_bad_input(client, project_id) -> None:
    other = make_auth_header("someone-else")
    assert (await client.post(f"/api/projects/{project_id}/diagrams", json={}, headers=other)).status_code == 403
    did = (await client.post(f"/api/projects/{project_id}/diagrams", json={}, headers=make_auth_header())).json()["id"]
    assert (await client.put(f"/api/projects/{project_id}/diagrams/{did}", json={"title": "x"}, headers=other)).status_code == 403
    h = make_auth_header()
    for body in ({"data": {"objects": "x"}}, {"svg": "<script/>"}, {"png": "data:image/jpeg;base64,AAAA"},
                 {"png": "data:image/png;base64," + base64.b64encode(b"not a png").decode()}):
        assert (await client.put(f"/api/projects/{project_id}/diagrams/{did}", json=body, headers=h)).status_code == 422, body
    # Another project's diagram id isn't reachable through this one.
    other_proj = (await client.post("/api/projects", json={"title": "Other"}, headers=h)).json()["id"]
    assert (await client.get(f"/api/projects/{other_proj}/diagrams/{did}")).status_code == 404


@pytest.mark.asyncio
async def test_draft_diagram_exports_are_never_cached(client) -> None:
    h = make_auth_header()
    pid = (await client.post("/api/projects", json={"title": "Secret drawing", "status": "draft"}, headers=h)).json()["id"]
    did = (await client.post(f"/api/projects/{pid}/diagrams", json={"title": "Arm", "data": CANVAS}, headers=h)).json()["id"]
    unexported = await client.get(f"/api/projects/{pid}/diagrams/{did}.svg", headers=h)
    assert unexported.status_code == 404 and "no-store" in unexported.headers["cache-control"]
    await client.put(f"/api/projects/{pid}/diagrams/{did}", json={"data": CANVAS, "svg": SVG, "png": PNG}, headers=h)
    for ext in ("svg", "png"):
        hidden = await client.get(f"/api/projects/{pid}/diagrams/{did}.{ext}")
        assert hidden.status_code == 404 and "no-store" in hidden.headers["cache-control"]
        own = await client.get(f"/api/projects/{pid}/diagrams/{did}.{ext}", headers=h)
        assert own.status_code == 200 and own.headers["cache-control"] == "private, no-store"
