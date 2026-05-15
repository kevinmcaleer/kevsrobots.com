"""Download tracking + analytics tests."""

from __future__ import annotations

from datetime import datetime, timedelta, timezone

import pytest
from sqlalchemy import select

from .conftest import make_auth_header


# ----- Fixtures -----


@pytest.fixture
async def project_with_file(client) -> dict:
    """Create a project + one uploaded file, return ids."""
    headers = make_auth_header()
    proj = await client.post(
        "/api/projects",
        json={"title": "Download Test Project"},
        headers=headers,
    )
    project_id = proj.json()["id"]

    upload = await client.post(
        f"/api/projects/{project_id}/files",
        files={"file": ("model.stl", b"binary-stl-data", "application/octet-stream")},
        headers=headers,
    )
    file_id = upload.json()["id"]
    return {"project_id": project_id, "file_id": file_id}


# ----- Logging -----


@pytest.mark.asyncio
async def test_anonymous_download_logs_with_ip_hash(client, session, project_with_file) -> None:
    """Anonymous request hits the download endpoint -> Download row stored
    with ip_hash and no user_id."""
    from projects_api.models import Download

    pid = project_with_file["project_id"]
    fid = project_with_file["file_id"]

    resp = await client.get(f"/api/projects/{pid}/files/{fid}/download")
    assert resp.status_code == 200
    assert resp.content == b"binary-stl-data"

    rows = (await session.execute(select(Download))).scalars().all()
    assert len(rows) == 1
    row = rows[0]
    assert row.project_id == pid
    assert row.file_id == fid
    assert row.user_id is None
    assert row.ip_hash is not None
    assert len(row.ip_hash) == 64  # sha256 hex


@pytest.mark.asyncio
async def test_authenticated_download_logs_with_user_id(client, session, project_with_file) -> None:
    """Authenticated user downloading -> Download row stores user_id, not ip_hash."""
    from projects_api.models import Download

    pid = project_with_file["project_id"]
    fid = project_with_file["file_id"]

    headers = make_auth_header("alice")
    resp = await client.get(
        f"/api/projects/{pid}/files/{fid}/download",
        headers=headers,
    )
    assert resp.status_code == 200

    rows = (await session.execute(select(Download))).scalars().all()
    assert len(rows) == 1
    assert rows[0].user_id == "alice"
    assert rows[0].ip_hash is None


@pytest.mark.asyncio
async def test_dedup_within_24h(client, session, project_with_file) -> None:
    """Same authenticated user downloading the same file twice within 24h
    only produces one Download row."""
    from projects_api.models import Download

    pid = project_with_file["project_id"]
    fid = project_with_file["file_id"]

    headers = make_auth_header("bob")
    for _ in range(3):
        resp = await client.get(
            f"/api/projects/{pid}/files/{fid}/download",
            headers=headers,
        )
        assert resp.status_code == 200

    rows = (await session.execute(select(Download))).scalars().all()
    assert len(rows) == 1


@pytest.mark.asyncio
async def test_dedup_anonymous_same_ip(client, session, project_with_file) -> None:
    """Same anonymous client (same TestClient -> same client IP) hitting
    twice within 24h dedups."""
    from projects_api.models import Download

    pid = project_with_file["project_id"]
    fid = project_with_file["file_id"]

    for _ in range(2):
        resp = await client.get(f"/api/projects/{pid}/files/{fid}/download")
        assert resp.status_code == 200

    rows = (await session.execute(select(Download))).scalars().all()
    assert len(rows) == 1


@pytest.mark.asyncio
async def test_logging_failure_does_not_break_download(
    client, project_with_file, monkeypatch
) -> None:
    """If the Download insert blows up, the user still gets their file."""
    from projects_api.routers import files as files_router

    async def _exploding_log(*args, **kwargs):
        raise RuntimeError("simulated database explosion")

    monkeypatch.setattr(files_router, "_log_download", _exploding_log)

    pid = project_with_file["project_id"]
    fid = project_with_file["file_id"]
    resp = await client.get(f"/api/projects/{pid}/files/{fid}/download")
    assert resp.status_code == 200
    assert resp.content == b"binary-stl-data"


# ----- Stats endpoint -----


@pytest.mark.asyncio
async def test_stats_requires_owner(client, project_with_file) -> None:
    """Non-owner gets 403."""
    pid = project_with_file["project_id"]
    headers = make_auth_header("not-the-owner")
    resp = await client.get(
        f"/api/projects/{pid}/downloads/stats",
        headers=headers,
    )
    assert resp.status_code == 403


@pytest.mark.asyncio
async def test_stats_unauthenticated(client, project_with_file) -> None:
    """No token -> 401."""
    pid = project_with_file["project_id"]
    resp = await client.get(f"/api/projects/{pid}/downloads/stats")
    assert resp.status_code == 401


@pytest.mark.asyncio
async def test_stats_owner_returns_breakdown(client, session, project_with_file) -> None:
    """Owner gets per-file counts and a contiguous 30-day daily series."""
    from projects_api.models import Download

    pid = project_with_file["project_id"]
    fid = project_with_file["file_id"]

    # Seed downloads directly so we can pin timestamps
    now = datetime.now(timezone.utc).replace(tzinfo=None)
    session.add_all([
        Download(project_id=pid, file_id=fid, user_id="u1", downloaded_at=now),
        Download(project_id=pid, file_id=fid, user_id="u2", downloaded_at=now - timedelta(days=2)),
        Download(project_id=pid, file_id=fid, user_id="u3", downloaded_at=now - timedelta(days=10)),
        # Outside the 30d window — should not appear in 30d/7d totals
        Download(project_id=pid, file_id=fid, user_id="u4", downloaded_at=now - timedelta(days=45)),
    ])
    await session.commit()

    headers = make_auth_header()  # default 'testuser' is the owner
    resp = await client.get(
        f"/api/projects/{pid}/downloads/stats",
        headers=headers,
    )
    assert resp.status_code == 200
    body = resp.json()
    assert body["project_id"] == pid
    assert body["total"] == 4
    assert body["last_7d"] == 2
    assert body["last_30d"] == 3
    assert len(body["per_file"]) == 1
    assert body["per_file"][0]["file_id"] == fid
    assert body["per_file"][0]["total"] == 4
    assert body["per_file"][0]["last_7d"] == 2
    assert body["per_file"][0]["last_30d"] == 3
    # Daily series — exactly 30 contiguous days, every entry has a date+count
    assert len(body["daily"]) == 30
    dates = [d["date"] for d in body["daily"]]
    assert dates == sorted(dates), "daily series should be ascending"


# ----- Popular endpoint -----


@pytest.mark.asyncio
async def test_popular_orders_by_count_in_window(client, session) -> None:
    """Project with more downloads in window ranks first."""
    from projects_api.models import Download

    headers = make_auth_header()
    # Create two projects, each with one file.
    p1 = (await client.post(
        "/api/projects",
        json={"title": "Quiet Project"},
        headers=headers,
    )).json()
    p2 = (await client.post(
        "/api/projects",
        json={"title": "Popular Project"},
        headers=headers,
    )).json()

    f1 = (await client.post(
        f"/api/projects/{p1['id']}/files",
        files={"file": ("a.stl", b"a", "application/octet-stream")},
        headers=headers,
    )).json()
    f2 = (await client.post(
        f"/api/projects/{p2['id']}/files",
        files={"file": ("b.stl", b"b", "application/octet-stream")},
        headers=headers,
    )).json()

    now = datetime.now(timezone.utc).replace(tzinfo=None)
    # 1 download for p1 inside 30d, 5 for p2 inside 30d
    session.add(Download(project_id=p1["id"], file_id=f1["id"], user_id="x", downloaded_at=now))
    for i in range(5):
        session.add(Download(
            project_id=p2["id"],
            file_id=f2["id"],
            user_id=f"u{i}",
            downloaded_at=now - timedelta(hours=i),
        ))
    await session.commit()

    resp = await client.get("/api/projects/popular?window=30d&limit=10")
    assert resp.status_code == 200
    items = resp.json()
    assert len(items) == 2
    assert items[0]["id"] == p2["id"]
    assert items[0]["download_count"] == 5
    assert items[1]["id"] == p1["id"]
    assert items[1]["download_count"] == 1


@pytest.mark.asyncio
async def test_popular_window_filters_old_downloads(client, session) -> None:
    """Downloads older than the window should not be counted."""
    from projects_api.models import Download

    headers = make_auth_header()
    p = (await client.post(
        "/api/projects",
        json={"title": "Vintage Project"},
        headers=headers,
    )).json()
    f = (await client.post(
        f"/api/projects/{p['id']}/files",
        files={"file": ("old.stl", b"old", "application/octet-stream")},
        headers=headers,
    )).json()

    now = datetime.now(timezone.utc).replace(tzinfo=None)
    session.add_all([
        Download(project_id=p["id"], file_id=f["id"], user_id="u1", downloaded_at=now - timedelta(days=60)),
        Download(project_id=p["id"], file_id=f["id"], user_id="u2", downloaded_at=now - timedelta(days=90)),
    ])
    await session.commit()

    resp_30d = await client.get("/api/projects/popular?window=30d")
    assert resp_30d.status_code == 200
    assert resp_30d.json() == []  # both downloads are older than 30d

    resp_all = await client.get("/api/projects/popular?window=all")
    assert resp_all.status_code == 200
    body = resp_all.json()
    assert len(body) == 1
    assert body[0]["download_count"] == 2


# ----- ProjectResponse download_count -----


@pytest.mark.asyncio
async def test_project_response_includes_download_count(
    client, session, project_with_file
) -> None:
    """The single-project GET should include the aggregated download_count."""
    from projects_api.models import Download

    pid = project_with_file["project_id"]
    fid = project_with_file["file_id"]

    now = datetime.now(timezone.utc).replace(tzinfo=None)
    session.add_all([
        Download(project_id=pid, file_id=fid, user_id="u1", downloaded_at=now),
        Download(project_id=pid, file_id=fid, user_id="u2", downloaded_at=now),
    ])
    await session.commit()

    resp = await client.get(f"/api/projects/{pid}")
    assert resp.status_code == 200
    assert resp.json()["download_count"] == 2
