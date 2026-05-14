"""Tests for the moderation endpoints."""

from __future__ import annotations

import pytest

from .conftest import make_auth_header


@pytest.mark.asyncio
async def test_report_project_authenticated(client, session):
    """Authenticated user can report a project."""
    # Create a project first
    headers = make_auth_header("owner")
    resp = await client.post(
        "/api/projects",
        json={"title": "Test Project For Report", "short_description": "desc"},
        headers=headers,
    )
    assert resp.status_code == 201
    project_id = resp.json()["id"]

    # Report as different user
    reporter_headers = make_auth_header("reporter")
    resp = await client.post(
        f"/api/projects/{project_id}/report",
        json={"reason": "Inappropriate content in this project"},
        headers=reporter_headers,
    )
    assert resp.status_code == 201
    data = resp.json()
    assert data["project_id"] == project_id
    assert data["reporter_username"] == "reporter"
    assert data["reason"] == "Inappropriate content in this project"
    assert data["status"] == "pending"


@pytest.mark.asyncio
async def test_report_project_unauthenticated(client, session):
    """Unauthenticated user cannot report a project (401)."""
    resp = await client.post(
        "/api/projects/1/report",
        json={"reason": "Some reason here"},
    )
    assert resp.status_code == 401


@pytest.mark.asyncio
async def test_report_own_project(client, session):
    """User can report their own project (allowed per spec)."""
    headers = make_auth_header("owner")
    resp = await client.post(
        "/api/projects",
        json={"title": "My Own Project Test", "short_description": "desc"},
        headers=headers,
    )
    project_id = resp.json()["id"]

    resp = await client.post(
        f"/api/projects/{project_id}/report",
        json={"reason": "Self-report for testing purposes"},
        headers=headers,
    )
    assert resp.status_code == 201
    assert resp.json()["reporter_username"] == "owner"


@pytest.mark.asyncio
async def test_report_duplicate_rejected(client, session):
    """Same user cannot report the same project twice."""
    headers = make_auth_header("creator")
    resp = await client.post(
        "/api/projects",
        json={"title": "Duplicate Report Test", "short_description": "desc"},
        headers=headers,
    )
    project_id = resp.json()["id"]

    reporter = make_auth_header("reporter2")
    resp = await client.post(
        f"/api/projects/{project_id}/report",
        json={"reason": "First report on this project"},
        headers=reporter,
    )
    assert resp.status_code == 201

    resp = await client.post(
        f"/api/projects/{project_id}/report",
        json={"reason": "Second report attempt here"},
        headers=reporter,
    )
    assert resp.status_code == 409


@pytest.mark.asyncio
async def test_admin_list_reports(client, session, monkeypatch):
    """Admin can list pending reports."""
    monkeypatch.setenv("ADMIN_USERNAMES", "admin")

    # Create project and report
    headers = make_auth_header("creator")
    resp = await client.post(
        "/api/projects",
        json={"title": "Admin Report List Test", "short_description": "desc"},
        headers=headers,
    )
    project_id = resp.json()["id"]

    reporter = make_auth_header("reporter3")
    await client.post(
        f"/api/projects/{project_id}/report",
        json={"reason": "Needs admin review now"},
        headers=reporter,
    )

    # Admin lists reports
    admin_headers = make_auth_header("admin")
    resp = await client.get("/api/admin/reports", headers=admin_headers)
    assert resp.status_code == 200
    reports = resp.json()
    assert len(reports) >= 1
    assert any(r["project_id"] == project_id for r in reports)


@pytest.mark.asyncio
async def test_admin_update_report(client, session, monkeypatch):
    """Admin can update report status."""
    monkeypatch.setenv("ADMIN_USERNAMES", "admin")

    headers = make_auth_header("creator")
    resp = await client.post(
        "/api/projects",
        json={"title": "Admin Update Report Test", "short_description": "desc"},
        headers=headers,
    )
    project_id = resp.json()["id"]

    reporter = make_auth_header("reporter4")
    resp = await client.post(
        f"/api/projects/{project_id}/report",
        json={"reason": "Update this report status"},
        headers=reporter,
    )
    report_id = resp.json()["id"]

    admin_headers = make_auth_header("admin")
    resp = await client.put(
        f"/api/admin/reports/{report_id}",
        json={"status": "reviewed"},
        headers=admin_headers,
    )
    assert resp.status_code == 200
    assert resp.json()["status"] == "reviewed"
    assert resp.json()["reviewed_by"] == "admin"


@pytest.mark.asyncio
async def test_admin_block_unblock_project(client, session, monkeypatch):
    """Admin can block and unblock a project."""
    monkeypatch.setenv("ADMIN_USERNAMES", "admin")

    headers = make_auth_header("creator")
    resp = await client.post(
        "/api/projects",
        json={"title": "Block Unblock Test Proj", "short_description": "desc"},
        headers=headers,
    )
    project_id = resp.json()["id"]

    admin_headers = make_auth_header("admin")

    # Block
    resp = await client.put(
        f"/api/admin/projects/{project_id}/block",
        headers=admin_headers,
    )
    assert resp.status_code == 200

    # Check blocked list
    resp = await client.get("/api/admin/projects/blocked", headers=admin_headers)
    assert resp.status_code == 200
    assert any(p["id"] == project_id for p in resp.json())

    # Unblock
    resp = await client.put(
        f"/api/admin/projects/{project_id}/unblock",
        headers=admin_headers,
    )
    assert resp.status_code == 200

    # No longer in blocked list
    resp = await client.get("/api/admin/projects/blocked", headers=admin_headers)
    assert not any(p["id"] == project_id for p in resp.json())


@pytest.mark.asyncio
async def test_blocked_project_not_in_listing(client, session, monkeypatch):
    """Blocked project should not appear in the public listing."""
    monkeypatch.setenv("ADMIN_USERNAMES", "admin")

    headers = make_auth_header("creator")
    resp = await client.post(
        "/api/projects",
        json={"title": "Hidden Blocked Project", "short_description": "desc"},
        headers=headers,
    )
    project_id = resp.json()["id"]

    # Visible before blocking
    resp = await client.get("/api/projects")
    assert any(p["id"] == project_id for p in resp.json())

    # Block it
    admin_headers = make_auth_header("admin")
    await client.put(
        f"/api/admin/projects/{project_id}/block",
        headers=admin_headers,
    )

    # Not visible after blocking
    resp = await client.get("/api/projects")
    assert not any(p["id"] == project_id for p in resp.json())


@pytest.mark.asyncio
async def test_non_admin_cannot_access_admin_endpoints(client, session):
    """Non-admin users get 403 on admin endpoints."""
    headers = make_auth_header("regularuser")

    resp = await client.get("/api/admin/reports", headers=headers)
    assert resp.status_code == 403

    resp = await client.put(
        "/api/admin/reports/1",
        json={"status": "reviewed"},
        headers=headers,
    )
    assert resp.status_code == 403

    resp = await client.put(
        "/api/admin/projects/1/block",
        headers=headers,
    )
    assert resp.status_code == 403

    resp = await client.put(
        "/api/admin/projects/1/unblock",
        headers=headers,
    )
    assert resp.status_code == 403

    resp = await client.get("/api/admin/projects/blocked", headers=headers)
    assert resp.status_code == 403


@pytest.mark.asyncio
async def test_admin_set_moderation_note(client, session, monkeypatch):
    """Admin can set a moderation note on a project."""
    monkeypatch.setenv("ADMIN_USERNAMES", "admin")

    headers = make_auth_header("creator")
    resp = await client.post(
        "/api/projects",
        json={"title": "Moderation Note Test Proj", "short_description": "desc"},
        headers=headers,
    )
    project_id = resp.json()["id"]

    admin_headers = make_auth_header("admin")
    resp = await client.put(
        f"/api/admin/projects/{project_id}/moderate",
        json={"moderation_note": "Reviewed, looks fine"},
        headers=admin_headers,
    )
    assert resp.status_code == 200
    assert resp.json()["detail"] == "Moderation note updated"
