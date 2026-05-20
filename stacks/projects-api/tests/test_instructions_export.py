"""Instruction PDF export tests (issue #178, Phase 2b).

Covers the hybrid PDF pipeline at the route level. The browser-side
half (Fabric rendering, ZIP assembly) lives in the frontend and isn't
exercised here.

Mirrors the fixture pattern used by ``tests/test_instructions.py``:
per-test project, then exercise the export endpoint against it. The
terms-gate test deliberately strips the ``conftest.client`` bypass so
we know the gate fires for the export route too.
"""

from __future__ import annotations

import pytest

from .conftest import make_auth_header

# A 1x1 transparent PNG — the smallest valid PNG payload. The PDF
# pipeline doesn't care about visual content, only that the bytes
# round-trip through ReportLab's Image parser.
_TINY_PNG_DATAURL = (
    "data:image/png;base64,"
    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNkYAAAAAYAAjCB0C8AAAAASUVORK5CYII="
)


@pytest.fixture
async def project_id(client) -> int:
    headers = make_auth_header()
    resp = await client.post(
        "/api/projects",
        json={"title": "Instruction Export Project"},
        headers=headers,
    )
    assert resp.status_code == 201, resp.text
    pid = resp.json()["id"]

    # Ensure there's an instruction row so the export route can find
    # the project's instructions (not strictly required by the route
    # itself, but keeps the fixtures realistic).
    inst = await client.post(
        f"/api/projects/{pid}/instruction",
        json={"title": "Assembly"},
        headers=headers,
    )
    assert inst.status_code == 200, inst.text
    return pid


def _one_step_payload() -> dict:
    return {
        "steps": [
            {
                "step_number": 1,
                "title": "Bolt the chassis",
                "description": "Use four M3x8 screws.",
                "image_data_url": _TINY_PNG_DATAURL,
            }
        ],
        "steps_per_page": 1,
        "include_title_page": True,
        "project_title": "My Project",
    }


# --- Happy paths --------------------------------------------------------


@pytest.mark.asyncio
async def test_export_pdf_one_step_returns_pdf(client, project_id) -> None:
    headers = make_auth_header()
    resp = await client.post(
        f"/api/projects/{project_id}/instruction/export/pdf",
        json=_one_step_payload(),
        headers=headers,
    )
    assert resp.status_code == 200, resp.text
    assert resp.headers["content-type"] == "application/pdf"
    # The download header should suggest a sensible filename.
    cd = resp.headers.get("content-disposition", "")
    assert "attachment" in cd
    assert f"instructions-{project_id}.pdf" in cd
    body = resp.content
    assert body.startswith(b"%PDF-"), body[:20]
    # Sanity: a real PDF is well over a hundred bytes even with one
    # tiny 1x1 image.
    assert len(body) > 500


@pytest.mark.asyncio
async def test_export_pdf_two_per_page(client, project_id) -> None:
    headers = make_auth_header()
    payload = {
        "steps": [
            {
                "step_number": i,
                "title": f"Step {i}",
                "description": f"Body for step {i}",
                "image_data_url": _TINY_PNG_DATAURL,
            }
            for i in range(1, 6)
        ],
        "steps_per_page": 2,
        "include_title_page": True,
        "project_title": "Five-Step Build",
    }
    resp = await client.post(
        f"/api/projects/{project_id}/instruction/export/pdf",
        json=payload,
        headers=headers,
    )
    assert resp.status_code == 200, resp.text
    assert resp.content.startswith(b"%PDF-")


@pytest.mark.asyncio
async def test_export_pdf_four_per_page(client, project_id) -> None:
    headers = make_auth_header()
    payload = {
        "steps": [
            {
                "step_number": i,
                "title": f"Step {i}",
                "image_data_url": _TINY_PNG_DATAURL,
            }
            for i in range(1, 8)
        ],
        "steps_per_page": 4,
        "include_title_page": False,
        "project_title": "Compact Build",
    }
    resp = await client.post(
        f"/api/projects/{project_id}/instruction/export/pdf",
        json=payload,
        headers=headers,
    )
    assert resp.status_code == 200, resp.text
    assert resp.content.startswith(b"%PDF-")


@pytest.mark.asyncio
async def test_export_pdf_without_title_page(client, project_id) -> None:
    """``include_title_page=False`` should still produce a valid PDF."""
    headers = make_auth_header()
    payload = _one_step_payload()
    payload["include_title_page"] = False
    resp = await client.post(
        f"/api/projects/{project_id}/instruction/export/pdf",
        json=payload,
        headers=headers,
    )
    assert resp.status_code == 200, resp.text
    assert resp.content.startswith(b"%PDF-")


# --- Validation -------------------------------------------------------


@pytest.mark.asyncio
async def test_export_pdf_invalid_layout_returns_422(client, project_id) -> None:
    headers = make_auth_header()
    payload = _one_step_payload()
    payload["steps_per_page"] = 3
    resp = await client.post(
        f"/api/projects/{project_id}/instruction/export/pdf",
        json=payload,
        headers=headers,
    )
    assert resp.status_code == 422


@pytest.mark.asyncio
async def test_export_pdf_empty_steps_returns_422(client, project_id) -> None:
    headers = make_auth_header()
    payload = _one_step_payload()
    payload["steps"] = []
    resp = await client.post(
        f"/api/projects/{project_id}/instruction/export/pdf",
        json=payload,
        headers=headers,
    )
    assert resp.status_code == 422


@pytest.mark.asyncio
async def test_export_pdf_bad_image_prefix_returns_422(client, project_id) -> None:
    headers = make_auth_header()
    payload = _one_step_payload()
    # Drop the dataURL prefix — must be rejected.
    payload["steps"][0]["image_data_url"] = "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNkYAAAAAYAAjCB0C8AAAAASUVORK5CYII="
    resp = await client.post(
        f"/api/projects/{project_id}/instruction/export/pdf",
        json=payload,
        headers=headers,
    )
    assert resp.status_code == 422


@pytest.mark.asyncio
async def test_export_pdf_jpeg_prefix_returns_422(client, project_id) -> None:
    """Even a valid JPEG dataURL should be rejected — we only accept PNG."""
    headers = make_auth_header()
    payload = _one_step_payload()
    payload["steps"][0]["image_data_url"] = (
        "data:image/jpeg;base64,/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAA=="
    )
    resp = await client.post(
        f"/api/projects/{project_id}/instruction/export/pdf",
        json=payload,
        headers=headers,
    )
    assert resp.status_code == 422


# --- Auth + ownership ------------------------------------------------


@pytest.mark.asyncio
async def test_export_pdf_non_owner_returns_403(client, project_id) -> None:
    other = make_auth_header(username="someoneelse")
    resp = await client.post(
        f"/api/projects/{project_id}/instruction/export/pdf",
        json=_one_step_payload(),
        headers=other,
    )
    assert resp.status_code == 403


@pytest.mark.asyncio
async def test_export_pdf_anonymous_returns_401(client, project_id) -> None:
    """No auth header at all — the auth dep should reject before
    the route runs."""
    resp = await client.post(
        f"/api/projects/{project_id}/instruction/export/pdf",
        json=_one_step_payload(),
    )
    assert resp.status_code == 401


# --- Terms gate ------------------------------------------------------
#
# Mirrors the gate test in ``test_instructions.py`` — opt back into the
# real ``require_terms_accepted`` dependency and confirm an export
# attempt without acceptance hits 403.


@pytest.mark.asyncio
async def test_export_pdf_without_terms_acceptance_returns_403(client) -> None:
    from projects_api import auth as auth_module

    app = client._transport.app
    for dep in (
        auth_module.require_terms_accepted,
        auth_module.require_terms_accepted_aged,
    ):
        app.dependency_overrides.pop(dep, None)

    headers = make_auth_header("alice")

    # Accept current terms so the project create can succeed.
    acc = await client.post(
        "/api/users/me/accept-terms",
        json={"version": "1.0"},
        headers=headers,
    )
    assert acc.status_code == 200, acc.text
    proj = await client.post(
        "/api/projects",
        json={"title": "Gated Export Project"},
        headers=headers,
    )
    assert proj.status_code == 201, proj.text
    pid = proj.json()["id"]

    # Now bump the current terms version so alice's old acceptance
    # goes stale.
    from projects_api import config as config_module
    from projects_api.routers import users as users_router

    def _fake_get_settings():
        s = config_module.Settings()
        return s.model_copy(update={"current_terms_version": "2.0"})

    monkeypatch = pytest.MonkeyPatch()
    monkeypatch.setattr(config_module, "get_settings", _fake_get_settings)
    monkeypatch.setattr(auth_module, "get_settings", _fake_get_settings)
    monkeypatch.setattr(users_router, "get_settings", _fake_get_settings)

    try:
        resp = await client.post(
            f"/api/projects/{pid}/instruction/export/pdf",
            json=_one_step_payload(),
            headers=headers,
        )
        assert resp.status_code == 403
        detail = resp.json().get("detail")
        assert isinstance(detail, dict)
        assert detail.get("detail") == "terms_not_accepted"
    finally:
        monkeypatch.undo()
