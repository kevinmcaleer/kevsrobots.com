"""Build-instructions CRUD tests (issue #178, Phase 0).

Mirrors the fixture pattern in ``tests/test_videos.py``: a per-test
project is created up front, then we exercise the instruction + step
endpoints against it. The terms-gate test deliberately strips the
``conftest.client`` bypass to confirm the gate fires on writes.
"""

from __future__ import annotations

import pytest

from .conftest import make_auth_header


@pytest.fixture
async def project_id(client) -> int:
    headers = make_auth_header()
    resp = await client.post(
        "/api/projects",
        json={"title": "Instruction Test Project"},
        headers=headers,
    )
    assert resp.status_code == 201, resp.text
    return resp.json()["id"]


# --- Instruction (singular) ----------------------------------------------


@pytest.mark.asyncio
async def test_instruction_post_is_idempotent(client, project_id) -> None:
    """A second POST returns the SAME row with 200, not 201."""
    headers = make_auth_header()
    first = await client.post(
        f"/api/projects/{project_id}/instruction",
        json={"title": "Assembly", "description": "How to build it"},
        headers=headers,
    )
    assert first.status_code == 200, first.text
    first_body = first.json()
    assert first_body["title"] == "Assembly"
    assert first_body["description"] == "How to build it"
    assert first_body["project_id"] == project_id
    assert first_body["steps"] == []

    second = await client.post(
        f"/api/projects/{project_id}/instruction",
        json={"title": "Different on second POST"},
        headers=headers,
    )
    assert second.status_code == 200
    second_body = second.json()
    # Idempotent: same row id, original title preserved (no overwrite).
    assert second_body["id"] == first_body["id"]
    assert second_body["title"] == "Assembly"


@pytest.mark.asyncio
async def test_get_returns_404_when_no_instruction(client, project_id) -> None:
    resp = await client.get(f"/api/projects/{project_id}/instruction")
    assert resp.status_code == 404


@pytest.mark.asyncio
async def test_put_updates_title_and_description(client, project_id) -> None:
    headers = make_auth_header()
    await client.post(
        f"/api/projects/{project_id}/instruction",
        json={"title": "Initial"},
        headers=headers,
    )
    resp = await client.put(
        f"/api/projects/{project_id}/instruction",
        json={"title": "Updated", "description": "New blurb"},
        headers=headers,
    )
    assert resp.status_code == 200, resp.text
    body = resp.json()
    assert body["title"] == "Updated"
    assert body["description"] == "New blurb"

    get_resp = await client.get(f"/api/projects/{project_id}/instruction")
    assert get_resp.status_code == 200
    assert get_resp.json()["title"] == "Updated"


@pytest.mark.asyncio
async def test_delete_removes_instruction(client, project_id) -> None:
    headers = make_auth_header()
    await client.post(
        f"/api/projects/{project_id}/instruction", json={}, headers=headers
    )
    resp = await client.delete(
        f"/api/projects/{project_id}/instruction", headers=headers
    )
    assert resp.status_code == 204

    get_resp = await client.get(f"/api/projects/{project_id}/instruction")
    assert get_resp.status_code == 404


# --- Steps ---------------------------------------------------------------


async def _create_instruction(client, project_id: int) -> None:
    headers = make_auth_header()
    resp = await client.post(
        f"/api/projects/{project_id}/instruction", json={}, headers=headers
    )
    assert resp.status_code == 200, resp.text


@pytest.mark.asyncio
async def test_steps_post_assigns_sequential_numbers(client, project_id) -> None:
    headers = make_auth_header()
    await _create_instruction(client, project_id)

    ids = []
    for title in ("Step A", "Step B", "Step C"):
        resp = await client.post(
            f"/api/projects/{project_id}/instruction/steps",
            json={"title": title},
            headers=headers,
        )
        assert resp.status_code == 201, resp.text
        ids.append(resp.json()["id"])

    list_resp = await client.get(
        f"/api/projects/{project_id}/instruction/steps"
    )
    assert list_resp.status_code == 200
    rows = list_resp.json()
    assert [r["step_number"] for r in rows] == [1, 2, 3]
    assert [r["id"] for r in rows] == ids


@pytest.mark.asyncio
async def test_step_put_step_number_reorders_and_renumbers(
    client, project_id
) -> None:
    """Moving the third step to position 1 should renumber survivors so
    the resulting sequence is 1..3 with the moved step first."""
    headers = make_auth_header()
    await _create_instruction(client, project_id)

    ids = []
    for title in ("first", "second", "third"):
        resp = await client.post(
            f"/api/projects/{project_id}/instruction/steps",
            json={"title": title},
            headers=headers,
        )
        ids.append(resp.json()["id"])

    # PUT step_number=1 on the third step.
    resp = await client.put(
        f"/api/projects/{project_id}/instruction/steps/{ids[2]}",
        json={"step_number": 1},
        headers=headers,
    )
    assert resp.status_code == 200, resp.text
    assert resp.json()["step_number"] == 1

    rows = (
        await client.get(f"/api/projects/{project_id}/instruction/steps")
    ).json()
    assert [r["id"] for r in rows] == [ids[2], ids[0], ids[1]]
    assert [r["step_number"] for r in rows] == [1, 2, 3]


@pytest.mark.asyncio
async def test_step_delete_leaves_gaps(client, project_id) -> None:
    """Deleting the middle step leaves survivors with their original
    numbers — gaps are allowed."""
    headers = make_auth_header()
    await _create_instruction(client, project_id)

    ids = []
    for title in ("a", "b", "c"):
        resp = await client.post(
            f"/api/projects/{project_id}/instruction/steps",
            json={"title": title},
            headers=headers,
        )
        ids.append(resp.json()["id"])

    # Delete the middle step (step_number=2).
    resp = await client.delete(
        f"/api/projects/{project_id}/instruction/steps/{ids[1]}",
        headers=headers,
    )
    assert resp.status_code == 204

    rows = (
        await client.get(f"/api/projects/{project_id}/instruction/steps")
    ).json()
    assert [r["id"] for r in rows] == [ids[0], ids[2]]
    # Numbers are unchanged — the survivors are still 1 and 3.
    assert [r["step_number"] for r in rows] == [1, 3]


@pytest.mark.asyncio
async def test_step_put_step_number_zero_returns_422(client, project_id) -> None:
    headers = make_auth_header()
    await _create_instruction(client, project_id)
    resp = await client.post(
        f"/api/projects/{project_id}/instruction/steps",
        json={"title": "only step"},
        headers=headers,
    )
    step_id = resp.json()["id"]

    resp = await client.put(
        f"/api/projects/{project_id}/instruction/steps/{step_id}",
        json={"step_number": 0},
        headers=headers,
    )
    assert resp.status_code == 422


@pytest.mark.asyncio
async def test_step_put_step_number_clamps_past_end(client, project_id) -> None:
    """Passing step_number well beyond the list length clamps to "last"
    rather than 422 — the frontend's move-down button doesn't have to
    know the current length."""
    headers = make_auth_header()
    await _create_instruction(client, project_id)
    ids = []
    for title in ("a", "b", "c"):
        resp = await client.post(
            f"/api/projects/{project_id}/instruction/steps",
            json={"title": title},
            headers=headers,
        )
        ids.append(resp.json()["id"])

    # Move the first step to step_number=999 → should land last.
    resp = await client.put(
        f"/api/projects/{project_id}/instruction/steps/{ids[0]}",
        json={"step_number": 999},
        headers=headers,
    )
    assert resp.status_code == 200
    assert resp.json()["step_number"] == 3

    rows = (
        await client.get(f"/api/projects/{project_id}/instruction/steps")
    ).json()
    assert [r["id"] for r in rows] == [ids[1], ids[2], ids[0]]


# --- Ownership + auth ---------------------------------------------------


@pytest.mark.asyncio
async def test_non_owner_post_returns_403(client, project_id) -> None:
    other = make_auth_header(username="someoneelse")
    resp = await client.post(
        f"/api/projects/{project_id}/instruction",
        json={"title": "hijack"},
        headers=other,
    )
    assert resp.status_code == 403


@pytest.mark.asyncio
async def test_non_owner_put_returns_403(client, project_id) -> None:
    owner = make_auth_header()
    await client.post(
        f"/api/projects/{project_id}/instruction",
        json={"title": "ok"},
        headers=owner,
    )
    other = make_auth_header(username="someoneelse")
    resp = await client.put(
        f"/api/projects/{project_id}/instruction",
        json={"title": "hijacked"},
        headers=other,
    )
    assert resp.status_code == 403


@pytest.mark.asyncio
async def test_non_owner_delete_returns_403(client, project_id) -> None:
    owner = make_auth_header()
    await client.post(
        f"/api/projects/{project_id}/instruction", json={}, headers=owner
    )
    other = make_auth_header(username="someoneelse")
    resp = await client.delete(
        f"/api/projects/{project_id}/instruction", headers=other
    )
    assert resp.status_code == 403


@pytest.mark.asyncio
async def test_non_owner_step_writes_return_403(client, project_id) -> None:
    owner = make_auth_header()
    await client.post(
        f"/api/projects/{project_id}/instruction", json={}, headers=owner
    )
    resp = await client.post(
        f"/api/projects/{project_id}/instruction/steps",
        json={"title": "owner-added"},
        headers=owner,
    )
    step_id = resp.json()["id"]

    other = make_auth_header(username="someoneelse")
    for verb, url, payload in (
        ("post", f"/api/projects/{project_id}/instruction/steps", {"title": "x"}),
        ("put", f"/api/projects/{project_id}/instruction/steps/{step_id}", {"title": "x"}),
        ("delete", f"/api/projects/{project_id}/instruction/steps/{step_id}", None),
    ):
        method = getattr(client, verb)
        if payload is None:
            resp = await method(url, headers=other)
        else:
            resp = await method(url, json=payload, headers=other)
        assert resp.status_code == 403, f"{verb} {url} should have been 403"


@pytest.mark.asyncio
async def test_anonymous_can_read_instruction(client, project_id) -> None:
    """Public GETs work without any auth header."""
    headers = make_auth_header()
    await client.post(
        f"/api/projects/{project_id}/instruction",
        json={"title": "public read"},
        headers=headers,
    )
    await client.post(
        f"/api/projects/{project_id}/instruction/steps",
        json={"title": "first step"},
        headers=headers,
    )

    # No auth header.
    resp = await client.get(f"/api/projects/{project_id}/instruction")
    assert resp.status_code == 200
    assert resp.json()["title"] == "public read"

    steps_resp = await client.get(
        f"/api/projects/{project_id}/instruction/steps"
    )
    assert steps_resp.status_code == 200
    assert len(steps_resp.json()) == 1


# --- Terms gate ---------------------------------------------------------
#
# The default ``client`` fixture in ``conftest.py`` patches
# ``require_terms_accepted`` to no-op so the bulk of the suite doesn't
# need to first POST /api/users/me/accept-terms. This single test opts
# back into the real gate (same pattern as ``tests/test_terms.py``)
# and confirms an instruction-write hits 403 before acceptance.


@pytest.mark.asyncio
async def test_write_without_terms_acceptance_returns_403(client) -> None:
    from projects_api import auth as auth_module

    app = client._transport.app
    for dep in (
        auth_module.require_terms_accepted,
        auth_module.require_terms_accepted_aged,
    ):
        app.dependency_overrides.pop(dep, None)

    headers = make_auth_header("alice")

    # Accept terms so the project itself can be created (POST /api/projects
    # is also gated by ``require_terms_accepted``).
    acc = await client.post(
        "/api/users/me/accept-terms",
        json={"version": "1.0"},
        headers=headers,
    )
    assert acc.status_code == 200, acc.text
    proj = await client.post(
        "/api/projects",
        json={"title": "Gated Instruction Project"},
        headers=headers,
    )
    assert proj.status_code == 201, proj.text
    pid = proj.json()["id"]

    # Now strip the user's terms acceptance to simulate a stale state.
    # Easiest way: create a brand-new user (different sub) with no
    # acceptance record and try to write to bob's project — fails 403
    # because bob isn't the owner. We need the same user to hit
    # not-accepted, so instead bump the version and have alice's
    # 1.0 acceptance go stale.
    from projects_api import config as config_module
    from projects_api.routers import users as users_router

    def _fake_get_settings():
        s = config_module.Settings()
        return s.model_copy(update={"current_terms_version": "2.0"})

    import pytest as _pytest

    monkeypatch = _pytest.MonkeyPatch()
    monkeypatch.setattr(config_module, "get_settings", _fake_get_settings)
    monkeypatch.setattr(auth_module, "get_settings", _fake_get_settings)
    monkeypatch.setattr(users_router, "get_settings", _fake_get_settings)

    try:
        resp = await client.post(
            f"/api/projects/{pid}/instruction",
            json={"title": "should be blocked"},
            headers=headers,
        )
        assert resp.status_code == 403
        detail = resp.json().get("detail")
        assert isinstance(detail, dict)
        assert detail.get("detail") == "terms_not_accepted"
    finally:
        monkeypatch.undo()
