"""Instruction GIF / MP4 export tests (issue #178, Phase 3b/3c).

Covers the FFmpeg-backed video pipeline at the route level. Mirrors
the fixture pattern from ``tests/test_instructions_export.py``:
per-test project, exercise the export endpoints against it.

Skips the happy-path tests when FFmpeg isn't on ``$PATH`` — the
production Docker image installs it via the Dockerfile, but local
dev / CI without it would otherwise hard-fail. Validation tests
(empty steps, bad PNG prefix, bad frame_seconds, auth) run regardless
because they fail before FFmpeg is invoked.
"""

from __future__ import annotations

import shutil

import pytest

from .conftest import make_auth_header

# A 1x1 transparent PNG — smallest valid PNG payload. The video
# pipeline doesn't care about visual content, only that the bytes
# round-trip through Pillow / FFmpeg's image-sequence demuxer.
_TINY_PNG_DATAURL = (
    "data:image/png;base64,"
    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNkYAAAAAYAAjCB0C8AAAAASUVORK5CYII="
)

_FFMPEG_AVAILABLE = shutil.which("ffmpeg") is not None
_skip_no_ffmpeg = pytest.mark.skipif(
    not _FFMPEG_AVAILABLE,
    reason="ffmpeg binary not on PATH — install ffmpeg to exercise the video pipeline",
)


@pytest.fixture
async def project_id(client) -> int:
    headers = make_auth_header()
    resp = await client.post(
        "/api/projects",
        json={"title": "Video Export Project"},
        headers=headers,
    )
    assert resp.status_code == 201, resp.text
    pid = resp.json()["id"]
    inst = await client.post(
        f"/api/projects/{pid}/instruction",
        json={"title": "Assembly"},
        headers=headers,
    )
    assert inst.status_code == 200, inst.text
    return pid


def _one_step_payload(**overrides) -> dict:
    base = {
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
        # Tight timing — keeps the test encode well under 1s with only
        # a couple of frames in play.
        "frame_seconds": 0.5,
        "final_slide_seconds": 1.0,
    }
    base.update(overrides)
    return base


# --- Happy paths --------------------------------------------------------


@_skip_no_ffmpeg
@pytest.mark.asyncio
async def test_export_gif_one_step_returns_gif(client, project_id) -> None:
    headers = make_auth_header()
    resp = await client.post(
        f"/api/projects/{project_id}/instruction/export/gif",
        json=_one_step_payload(),
        headers=headers,
    )
    assert resp.status_code == 200, resp.text
    assert resp.headers["content-type"] == "image/gif"
    cd = resp.headers.get("content-disposition", "")
    assert "attachment" in cd
    assert f"instructions-{project_id}.gif" in cd
    body = resp.content
    # GIF magic — either GIF87a or GIF89a; FFmpeg writes 89a.
    assert body[:6] in (b"GIF87a", b"GIF89a"), body[:6]
    # Even a 2-frame GIF (1 step + 1 slide hold) has a non-trivial header.
    assert len(body) > 1000


@_skip_no_ffmpeg
@pytest.mark.asyncio
async def test_export_mp4_one_step_returns_mp4(client, project_id) -> None:
    headers = make_auth_header()
    resp = await client.post(
        f"/api/projects/{project_id}/instruction/export/mp4",
        json=_one_step_payload(),
        headers=headers,
    )
    assert resp.status_code == 200, resp.text
    assert resp.headers["content-type"] == "video/mp4"
    cd = resp.headers.get("content-disposition", "")
    assert "attachment" in cd
    assert f"instructions-{project_id}.mp4" in cd
    body = resp.content
    # ISO BMFF marker: 'ftyp' box appears within the first ~32 bytes of
    # any well-formed MP4. Faststart hoists it to the front so a casual
    # contains check is enough.
    assert b"ftyp" in body[:32], body[:32]
    assert len(body) > 1000


@_skip_no_ffmpeg
@pytest.mark.asyncio
async def test_export_gif_multiple_steps(client, project_id) -> None:
    """Smoke test with a couple of steps — both frame sequence
    handling and the closing-slide hold should round-trip cleanly."""
    headers = make_auth_header()
    payload = {
        "steps": [
            {
                "step_number": i,
                "title": f"Step {i}",
                "description": f"Body {i}",
                "image_data_url": _TINY_PNG_DATAURL,
            }
            for i in range(1, 3)
        ],
        "steps_per_page": 1,
        "include_title_page": True,
        "project_title": "Multi-step",
        "frame_seconds": 0.5,
        "final_slide_seconds": 1.0,
    }
    resp = await client.post(
        f"/api/projects/{project_id}/instruction/export/gif",
        json=payload,
        headers=headers,
    )
    assert resp.status_code == 200, resp.text
    assert resp.content[:6] in (b"GIF87a", b"GIF89a")


# --- Validation -------------------------------------------------------


@pytest.mark.asyncio
@pytest.mark.parametrize("fmt", ["gif", "mp4"])
async def test_export_video_empty_steps_returns_422(client, project_id, fmt) -> None:
    headers = make_auth_header()
    payload = _one_step_payload()
    payload["steps"] = []
    resp = await client.post(
        f"/api/projects/{project_id}/instruction/export/{fmt}",
        json=payload,
        headers=headers,
    )
    assert resp.status_code == 422


@pytest.mark.asyncio
@pytest.mark.parametrize("fmt", ["gif", "mp4"])
@pytest.mark.parametrize("bad", [0, -1, 999])
async def test_export_video_invalid_frame_seconds_returns_422(
    client, project_id, fmt, bad
) -> None:
    headers = make_auth_header()
    payload = _one_step_payload(frame_seconds=bad)
    resp = await client.post(
        f"/api/projects/{project_id}/instruction/export/{fmt}",
        json=payload,
        headers=headers,
    )
    assert resp.status_code == 422


@pytest.mark.asyncio
@pytest.mark.parametrize("fmt", ["gif", "mp4"])
@pytest.mark.parametrize("bad", [0, 999])
async def test_export_video_invalid_slide_seconds_returns_422(
    client, project_id, fmt, bad
) -> None:
    headers = make_auth_header()
    payload = _one_step_payload(final_slide_seconds=bad)
    resp = await client.post(
        f"/api/projects/{project_id}/instruction/export/{fmt}",
        json=payload,
        headers=headers,
    )
    assert resp.status_code == 422


@pytest.mark.asyncio
@pytest.mark.parametrize("fmt", ["gif", "mp4"])
async def test_export_video_bad_image_prefix_returns_422(
    client, project_id, fmt
) -> None:
    headers = make_auth_header()
    payload = _one_step_payload()
    payload["steps"][0]["image_data_url"] = (
        "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNkYAAAAAYAAjCB0C8AAAAASUVORK5CYII="
    )
    resp = await client.post(
        f"/api/projects/{project_id}/instruction/export/{fmt}",
        json=payload,
        headers=headers,
    )
    assert resp.status_code == 422


@pytest.mark.asyncio
@pytest.mark.parametrize("fmt", ["gif", "mp4"])
async def test_export_video_jpeg_prefix_returns_422(
    client, project_id, fmt
) -> None:
    """A valid JPEG dataURL is still rejected — we only accept PNG."""
    headers = make_auth_header()
    payload = _one_step_payload()
    payload["steps"][0]["image_data_url"] = (
        "data:image/jpeg;base64,/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAA=="
    )
    resp = await client.post(
        f"/api/projects/{project_id}/instruction/export/{fmt}",
        json=payload,
        headers=headers,
    )
    assert resp.status_code == 422


# --- Auth + ownership ------------------------------------------------


@pytest.mark.asyncio
@pytest.mark.parametrize("fmt", ["gif", "mp4"])
async def test_export_video_non_owner_returns_403(client, project_id, fmt) -> None:
    other = make_auth_header(username="someoneelse")
    resp = await client.post(
        f"/api/projects/{project_id}/instruction/export/{fmt}",
        json=_one_step_payload(),
        headers=other,
    )
    assert resp.status_code == 403


@pytest.mark.asyncio
@pytest.mark.parametrize("fmt", ["gif", "mp4"])
async def test_export_video_anonymous_returns_401(client, project_id, fmt) -> None:
    """No auth header — auth dep rejects before the route runs."""
    resp = await client.post(
        f"/api/projects/{project_id}/instruction/export/{fmt}",
        json=_one_step_payload(),
    )
    assert resp.status_code == 401


# --- Terms gate ------------------------------------------------------
#
# Mirrors the gate test in ``test_instructions_export.py`` — opt back
# into the real ``require_terms_accepted`` dependency for both the GIF
# and MP4 routes.


@pytest.mark.asyncio
@pytest.mark.parametrize("fmt", ["gif", "mp4"])
async def test_export_video_without_terms_acceptance_returns_403(
    client, fmt
) -> None:
    from projects_api import auth as auth_module

    app = client._transport.app
    for dep in (
        auth_module.require_terms_accepted,
        auth_module.require_terms_accepted_aged,
    ):
        app.dependency_overrides.pop(dep, None)

    headers = make_auth_header("alice")
    acc = await client.post(
        "/api/users/me/accept-terms",
        json={"version": "1.0"},
        headers=headers,
    )
    assert acc.status_code == 200, acc.text
    proj = await client.post(
        "/api/projects",
        json={"title": "Gated Video Project"},
        headers=headers,
    )
    assert proj.status_code == 201, proj.text
    pid = proj.json()["id"]

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
            f"/api/projects/{pid}/instruction/export/{fmt}",
            json=_one_step_payload(),
            headers=headers,
        )
        assert resp.status_code == 403
        detail = resp.json().get("detail")
        assert isinstance(detail, dict)
        assert detail.get("detail") == "terms_not_accepted"
    finally:
        monkeypatch.undo()


# --- Branding slide module --------------------------------------------
#
# Exercise the Pillow slide renderer directly (no FFmpeg needed). The
# slide is generated identically for PDF / GIF / MP4 so it's worth a
# tiny unit test guarding the fallback-font path.


def test_render_made_with_slide_returns_png_bytes() -> None:
    from projects_api.instructions_branding import render_made_with_slide

    png = render_made_with_slide(project_title="Test Build")
    assert png.startswith(b"\x89PNG\r\n\x1a\n"), png[:8]
    # Headline + subtitle + wordmark + project title — even at the
    # default 1200x900 a meaningful slide is well over 5KB.
    assert len(png) > 5000


def test_render_made_with_slide_without_title() -> None:
    """Optional ``project_title`` must not be required."""
    from projects_api.instructions_branding import render_made_with_slide

    png = render_made_with_slide()
    assert png.startswith(b"\x89PNG\r\n\x1a\n")
