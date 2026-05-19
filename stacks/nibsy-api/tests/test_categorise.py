"""AI categorisation export/import tests (#75).

After #158 the /api/admin/* routes require admin auth — every request
here passes ``make_admin_header()`` so we exercise the same code paths
authenticated.
"""

from __future__ import annotations

import pytest

from .conftest import make_admin_header


@pytest.mark.asyncio
async def test_export_returns_items(client) -> None:
    response = await client.get(
        "/api/admin/categorise/export", headers=make_admin_header()
    )
    assert response.status_code == 200
    body = response.json()
    assert "items" in body
    assert "count" in body
    assert "prompt_hint" in body
    assert body["count"] == len(body["items"])
    assert body["count"] > 0
    for item in body["items"]:
        assert "id" in item
        assert "title" in item
        assert "url" in item


@pytest.mark.asyncio
async def test_export_skips_already_categorised(client) -> None:
    # Import a categorisation for item 1.
    await client.post(
        "/api/admin/categorise/import",
        json=[{"id": 1, "topics": ["test"], "difficulty": "beginner"}],
        headers=make_admin_header(),
    )
    # Export without force — item 1 should be skipped.
    response = await client.get(
        "/api/admin/categorise/export", headers=make_admin_header()
    )
    body = response.json()
    ids = [item["id"] for item in body["items"]]
    assert 1 not in ids

    # Export with force — item 1 should appear.
    response_force = await client.get(
        "/api/admin/categorise/export?force=true", headers=make_admin_header()
    )
    body_force = response_force.json()
    ids_force = [item["id"] for item in body_force["items"]]
    assert 1 in ids_force


@pytest.mark.asyncio
async def test_import_categorisation(client) -> None:
    response = await client.post(
        "/api/admin/categorise/import",
        json=[
            {
                "id": 1,
                "topics": ["micropython", "servo motors", "robotics"],
                "difficulty": "beginner",
                "introduces": ["PWM", "servo control"],
                "assumes": ["basic Python"],
            }
        ],
        headers=make_admin_header(),
    )
    assert response.status_code == 200
    body = response.json()
    assert body["updated"] == 1
    assert body["not_found"] == 0


@pytest.mark.asyncio
async def test_import_with_pathway(client) -> None:
    response = await client.post(
        "/api/admin/categorise/import",
        json=[
            {
                "id": 1,
                "topics": ["micropython"],
                "difficulty": "beginner",
                "pathway": {"name": "micropython-basics", "order": 1, "total": 3},
            }
        ],
        headers=make_admin_header(),
    )
    assert response.status_code == 200
    assert response.json()["updated"] == 1


@pytest.mark.asyncio
async def test_import_not_found(client) -> None:
    response = await client.post(
        "/api/admin/categorise/import",
        json=[{"id": 999999, "topics": ["test"]}],
        headers=make_admin_header(),
    )
    assert response.status_code == 200
    assert response.json()["not_found"] == 1
