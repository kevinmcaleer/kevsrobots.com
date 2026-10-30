"""Admin tools for moving existing projects to the new editor (issue #273)."""

from __future__ import annotations

import pytest
from sqlalchemy import select

from projects_api.models import Project
from projects_api.routers.admin_migration import GENERATED_MARKER
from .conftest import make_auth_header

TABLE = "# Bill of Materials\n\n| Part | Qty | Price | Total | Notes |\n| --- | --- | --- | --- | --- |\n| Arduino Uno | 1 | £18.00 |  |  |\n"


@pytest.fixture
def admin(monkeypatch) -> dict:
    monkeypatch.setenv("ADMIN_USERNAMES", "admin")
    return make_auth_header("admin")


@pytest.fixture
async def project_id(client) -> int:
    resp = await client.post(
        "/api/projects",
        json={"title": "Old Rover", "content_md": "# Overview\n\nAn older rover.\n", "status": "draft"},
        headers=make_auth_header(),
    )
    pid = resp.json()["id"]
    resp = await client.post(
        f"/api/projects/{pid}/bom",
        json={"name": "Arduino Uno", "quantity": 1, "unit_cost": 18.0, "currency_code": "GBP"},
        headers=make_auth_header(),
    )
    assert resp.status_code == 201, resp.text
    return pid


@pytest.mark.asyncio
async def test_routes_are_admin_only(client, project_id, admin) -> None:
    for path in ("/api/admin/migration/bom", "/api/admin/migration/schematics", "/api/admin/migration/contents"):
        assert (await client.get(path, headers=make_auth_header())).status_code == 403
    resp = await client.put(
        f"/api/admin/migration/projects/{project_id}/bom-table",
        json={"expected_content_md": "", "content_md": ""}, headers=make_auth_header(),
    )
    assert resp.status_code == 403


@pytest.mark.asyncio
async def test_lists_projects_with_bom_rows_including_drafts(client, project_id, admin) -> None:
    rows = (await client.get("/api/admin/migration/bom", headers=admin)).json()
    assert [r["project_id"] for r in rows] == [project_id]
    assert rows[0]["content_md"] == "# Overview\n\nAn older rover.\n"
    assert rows[0]["rows"][0]["name"] == "Arduino Uno"
    assert rows[0]["rows"][0]["currency_code"] == "GBP"


@pytest.mark.asyncio
async def test_appends_a_generated_table_without_touching_updated_at(client, session, project_id, admin) -> None:
    before = (await session.get(Project, project_id)).updated_at
    old = "# Overview\n\nAn older rover.\n"
    new = old + "\n" + GENERATED_MARKER + "\n" + TABLE
    resp = await client.put(
        f"/api/admin/migration/projects/{project_id}/bom-table",
        json={"expected_content_md": old, "content_md": new}, headers=admin,
    )
    assert resp.status_code == 204, resp.text
    session.expire_all()
    project = (await session.scalars(select(Project).where(Project.id == project_id))).one()
    assert project.content_md == new
    assert project.updated_at == before


@pytest.mark.asyncio
async def test_refuses_anything_but_an_append(client, project_id, admin) -> None:
    old = "# Overview\n\nAn older rover.\n"
    url = f"/api/admin/migration/projects/{project_id}/bom-table"
    # Rewrites the existing text.
    resp = await client.put(url, json={"expected_content_md": old, "content_md": "# Gone\n" + GENERATED_MARKER}, headers=admin)
    assert resp.status_code == 400
    # Appends something without the marker.
    resp = await client.put(url, json={"expected_content_md": old, "content_md": old + "\n" + TABLE}, headers=admin)
    assert resp.status_code == 400
    # The owner saved since the page read the text.
    resp = await client.put(
        url, json={"expected_content_md": "stale", "content_md": "stale\n" + GENERATED_MARKER}, headers=admin,
    )
    assert resp.status_code == 409


@pytest.mark.asyncio
async def test_lists_legacy_schematics(client, project_id, admin) -> None:
    assert (await client.get("/api/admin/migration/schematics", headers=admin)).json() == []
    resp = await client.post(
        f"/api/projects/{project_id}/schematic", json={"name": "Rover"}, headers=make_auth_header(),
    )
    assert resp.status_code in (200, 201), resp.text
    rows = (await client.get("/api/admin/migration/schematics", headers=admin)).json()
    assert [(r["project_id"], r["title"]) for r in rows] == [(project_id, "Old Rover")]


@pytest.mark.asyncio
async def test_a_project_redrawn_as_wiring_is_done(client, session, project_id, admin) -> None:
    from projects_api.models import ProjectSchematic, ProjectWiring
    session.add(ProjectSchematic(project_id=project_id, schematic_data="{}"))
    session.add(ProjectWiring(project_id=project_id, data={}))
    await session.commit()
    assert (await client.get("/api/admin/migration/schematics", headers=admin)).json() == []


@pytest.mark.asyncio
async def test_pages_through_every_projects_text(client, project_id, admin) -> None:
    await client.post("/api/projects", json={"title": "Second project"}, headers=make_auth_header())
    assert (await client.get("/api/admin/migration/contents/count", headers=admin)).json() == {"count": 2}
    first = (await client.get("/api/admin/migration/contents?limit=1", headers=admin)).json()
    second = (await client.get("/api/admin/migration/contents?offset=1&limit=1", headers=admin)).json()
    assert [p["project_id"] for p in first + second] == [project_id, project_id + 1]
    assert second[0]["content_md"] == ""
