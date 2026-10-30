"""Project wiring from Snakie's electronics workspace (issue #268), and the
SVG cleaning it relies on."""

from __future__ import annotations

import pytest

from projects_api.svg_safety import UnsafeSvg, clean_svg
from .conftest import make_auth_header

ROBOT = {
    "board": "pico",
    "parts": [{"id": "sg90", "lib": "kevsrobots", "part": "sg90", "x": 400, "y": 120}],
    "connections": [{"from": "board.GP0#0", "to": "sg90.Signal#0"}],
}
SVG = ('<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 10 10" width="10" height="10">'
       '<rect width="10" height="10" style="fill: rgb(1, 2, 3)"/></svg>')


@pytest.fixture
async def project_id(client) -> int:
    resp = await client.post("/api/projects", json={"title": "Wired robot"}, headers=make_auth_header())
    assert resp.status_code == 201, resp.text
    return resp.json()["id"]


@pytest.mark.asyncio
async def test_wiring_round_trip(client, project_id) -> None:
    assert (await client.get(f"/api/projects/{project_id}/wiring")).status_code == 404
    assert (await client.get(f"/api/projects/{project_id}/wiring.svg")).status_code == 404

    resp = await client.put(f"/api/projects/{project_id}/wiring", json={"data": ROBOT, "svg": SVG},
                            headers=make_auth_header())
    assert resp.status_code == 200, resp.text
    body = resp.json()
    assert body["data"] == ROBOT
    assert body["svg_url"].endswith(f"/api/projects/{project_id}/wiring.svg")

    # Re-opening restores exactly what was saved.
    assert (await client.get(f"/api/projects/{project_id}/wiring")).json()["data"] == ROBOT
    svg = await client.get(f"/api/projects/{project_id}/wiring.svg")
    assert svg.status_code == 200
    assert svg.headers["content-type"] == "image/svg+xml"
    assert "script-src" not in svg.headers["content-security-policy"]
    assert svg.headers["content-security-policy"].startswith("default-src 'none'")
    assert b"<rect" in svg.content

    # Saving the layout alone keeps the last render.
    moved = dict(ROBOT, boardX=10)
    resp = await client.put(f"/api/projects/{project_id}/wiring", json={"data": moved},
                            headers=make_auth_header())
    assert resp.json()["data"]["boardX"] == 10
    assert (await client.get(f"/api/projects/{project_id}/wiring.svg")).status_code == 200

    assert (await client.delete(f"/api/projects/{project_id}/wiring", headers=make_auth_header())).status_code == 204
    assert (await client.get(f"/api/projects/{project_id}/wiring")).status_code == 404


@pytest.mark.asyncio
async def test_only_the_owner_saves(client, project_id) -> None:
    resp = await client.put(f"/api/projects/{project_id}/wiring", json={"data": ROBOT},
                            headers=make_auth_header("someone-else"))
    assert resp.status_code == 403
    assert (await client.put(f"/api/projects/{project_id}/wiring", json={"data": ROBOT})).status_code == 401


@pytest.mark.asyncio
async def test_bad_robot_and_svg_rejected(client, project_id) -> None:
    h = make_auth_header()
    assert (await client.put(f"/api/projects/{project_id}/wiring", json={"data": {"parts": "x"}},
                             headers=h)).status_code == 422
    assert (await client.put(f"/api/projects/{project_id}/wiring", json={"data": ROBOT, "svg": "<html/>"},
                             headers=h)).status_code == 422


def test_clean_svg_strips_script() -> None:
    dirty = ('<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" onload="x()">'
             '<script>alert(1)</script><a href="javascript:alert(1)"><rect/></a>'
             '<image href="data:image/png;base64,AAAA"/><image xlink:href="https://evil.example/x.png"/>'
             '<foreignObject><div xmlns="http://www.w3.org/1999/xhtml">hi</div></foreignObject>'
             '<rect style="fill:url(https://evil.example/)"/><rect fill="url(#g)"/></svg>')
    out = clean_svg(dirty)
    assert "script" not in out and "onload" not in out and "javascript" not in out
    assert "foreignObject" not in out and "evil.example" not in out
    assert "data:image/png;base64,AAAA" in out
    assert 'fill="url(#g)"' in out


def test_clean_svg_refuses_doctype_and_non_svg() -> None:
    with pytest.raises(UnsafeSvg):
        clean_svg('<!DOCTYPE svg [<!ENTITY a "b">]><svg xmlns="http://www.w3.org/2000/svg"/>')
    with pytest.raises(UnsafeSvg):
        clean_svg("<html/>")
    with pytest.raises(UnsafeSvg):
        clean_svg("not xml")
