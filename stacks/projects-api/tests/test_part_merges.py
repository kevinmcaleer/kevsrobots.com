"""Tests for the parts catalog merge proposals (issue #123).

Covers:
* propose (happy path + self-merge rejected + unknown target 404)
* withdraw (proposer / admin)
* vote insert + update (changing your mind)
* 48h gate + threshold-driven auto-merge
* threshold rule (5 approves, 2:1 ratio, 48h age)
"""

from __future__ import annotations

from datetime import datetime, timedelta
from typing import Optional

import pytest
from sqlalchemy import select


from .conftest import make_auth_header


@pytest.fixture(autouse=True)
def _old_account(monkeypatch: pytest.MonkeyPatch) -> None:
    """Pretend every account is 5 years old by default."""
    from projects_api import auth as auth_module

    async def _fake_age(username: str, token: str) -> Optional[datetime]:
        return datetime.utcnow() - timedelta(days=365 * 5)

    monkeypatch.setattr(auth_module, "fetch_account_created_at", _fake_age)
    auth_module._clear_account_age_cache()


@pytest.fixture(autouse=True)
def _admin_user(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("ADMIN_USERNAMES", "admin")


async def _make_part(client, name: str, *, author: str = "alice") -> dict:
    resp = await client.post(
        "/api/parts", json={"name": name}, headers=make_auth_header(author)
    )
    assert resp.status_code == 201, resp.text
    return resp.json()


async def _propose(client, source_slug: str, target_slug: str, *, proposer: str = "alice") -> dict:
    resp = await client.post(
        f"/api/parts/{source_slug}/merge-proposal",
        json={"target_slug": target_slug, "rationale": "These are the same part with different SKUs"},
        headers=make_auth_header(proposer),
    )
    return resp


@pytest.mark.asyncio
async def test_propose_merge_happy_path(client) -> None:
    src = await _make_part(client, "Pi Pico H")
    tgt = await _make_part(client, "Pi Pico")

    resp = await _propose(client, src["slug"], tgt["slug"])
    assert resp.status_code == 201, resp.text
    body = resp.json()
    assert body["source"]["slug"] == src["slug"]
    assert body["target"]["slug"] == tgt["slug"]
    assert body["approves"] == 0
    assert body["rejects"] == 0
    assert body["outcome"] is None


@pytest.mark.asyncio
async def test_self_merge_rejected(client) -> None:
    p = await _make_part(client, "Lonely")
    resp = await _propose(client, p["slug"], p["slug"])
    assert resp.status_code == 400
    assert "itself" in resp.json()["detail"].lower()


@pytest.mark.asyncio
async def test_propose_requires_account_age(client, monkeypatch) -> None:
    src = await _make_part(client, "Pi Pico H")
    tgt = await _make_part(client, "Pi Pico")

    from projects_api import auth as auth_module

    async def _young(username: str, token: str) -> Optional[datetime]:
        return datetime.utcnow() - timedelta(days=3)

    monkeypatch.setattr(auth_module, "fetch_account_created_at", _young)
    auth_module._clear_account_age_cache()

    resp = await _propose(client, src["slug"], tgt["slug"], proposer="bob")
    assert resp.status_code == 403


@pytest.mark.asyncio
async def test_propose_unknown_target_404(client) -> None:
    src = await _make_part(client, "Widget")
    resp = await client.post(
        f"/api/parts/{src['slug']}/merge-proposal",
        json={"target_slug": "does-not-exist", "rationale": "x" * 20},
        headers=make_auth_header("alice"),
    )
    assert resp.status_code == 404


@pytest.mark.asyncio
async def test_duplicate_open_proposal_409(client) -> None:
    src = await _make_part(client, "Pi Pico H")
    tgt = await _make_part(client, "Pi Pico")
    r1 = await _propose(client, src["slug"], tgt["slug"])
    assert r1.status_code == 201
    r2 = await _propose(client, src["slug"], tgt["slug"], proposer="bob")
    assert r2.status_code == 409


@pytest.mark.asyncio
async def test_list_proposals_public_and_filters(client) -> None:
    src = await _make_part(client, "Pi Pico H")
    tgt = await _make_part(client, "Pi Pico")
    create = await _propose(client, src["slug"], tgt["slug"])
    pid = create.json()["id"]

    # Public list (no auth)
    resp = await client.get("/api/parts/merge-proposals")
    assert resp.status_code == 200
    body = resp.json()
    assert body["total"] == 1
    assert body["items"][0]["id"] == pid

    # Single-proposal endpoint also public.
    one = await client.get(f"/api/parts/merge-proposals/{pid}")
    assert one.status_code == 200
    assert one.json()["id"] == pid


@pytest.mark.asyncio
async def test_vote_insert_and_update(client) -> None:
    src = await _make_part(client, "Pi Pico H")
    tgt = await _make_part(client, "Pi Pico")
    create = await _propose(client, src["slug"], tgt["slug"])
    pid = create.json()["id"]

    # First vote
    r1 = await client.post(
        f"/api/parts/merge-proposals/{pid}/vote",
        json={"vote": "approve"},
        headers=make_auth_header("bob"),
    )
    assert r1.status_code == 200, r1.text
    assert r1.json()["approves"] == 1
    assert r1.json()["rejects"] == 0

    # Same voter changes their mind -> updates the existing row.
    r2 = await client.post(
        f"/api/parts/merge-proposals/{pid}/vote",
        json={"vote": "reject"},
        headers=make_auth_header("bob"),
    )
    assert r2.status_code == 200
    assert r2.json()["approves"] == 0
    assert r2.json()["rejects"] == 1


@pytest.mark.asyncio
async def test_withdraw_by_proposer(client) -> None:
    src = await _make_part(client, "Pi Pico H")
    tgt = await _make_part(client, "Pi Pico")
    create = await _propose(client, src["slug"], tgt["slug"], proposer="alice")
    pid = create.json()["id"]

    # Non-proposer non-admin can't withdraw.
    forbidden = await client.delete(
        f"/api/parts/merge-proposals/{pid}",
        headers=make_auth_header("bob"),
    )
    assert forbidden.status_code == 403

    # Proposer can.
    ok = await client.delete(
        f"/api/parts/merge-proposals/{pid}",
        headers=make_auth_header("alice"),
    )
    assert ok.status_code == 204

    detail = await client.get(f"/api/parts/merge-proposals/{pid}")
    assert detail.json()["outcome"] == "withdrawn"
    assert detail.json()["resolved_at"] is not None


@pytest.mark.asyncio
async def test_withdraw_by_admin(client) -> None:
    src = await _make_part(client, "Pi Pico H")
    tgt = await _make_part(client, "Pi Pico")
    create = await _propose(client, src["slug"], tgt["slug"], proposer="alice")
    pid = create.json()["id"]
    ok = await client.delete(
        f"/api/parts/merge-proposals/{pid}",
        headers=make_auth_header("admin"),
    )
    assert ok.status_code == 204


@pytest.mark.asyncio
async def test_voting_blocked_after_resolve(client) -> None:
    src = await _make_part(client, "Pi Pico H")
    tgt = await _make_part(client, "Pi Pico")
    create = await _propose(client, src["slug"], tgt["slug"], proposer="alice")
    pid = create.json()["id"]
    await client.delete(
        f"/api/parts/merge-proposals/{pid}",
        headers=make_auth_header("alice"),
    )
    blocked = await client.post(
        f"/api/parts/merge-proposals/{pid}/vote",
        json={"vote": "approve"},
        headers=make_auth_header("bob"),
    )
    assert blocked.status_code == 409


@pytest.mark.asyncio
async def test_threshold_blocked_by_48h_age(client, session) -> None:
    """Five approves on a brand-new proposal don't trigger — the 48h age
    gate is still in force."""
    src = await _make_part(client, "Pi Pico H")
    tgt = await _make_part(client, "Pi Pico")
    create = await _propose(client, src["slug"], tgt["slug"])
    pid = create.json()["id"]

    for voter in ("v1", "v2", "v3", "v4", "v5"):
        r = await client.post(
            f"/api/parts/merge-proposals/{pid}/vote",
            json={"vote": "approve"},
            headers=make_auth_header(voter),
        )
        assert r.status_code == 200

    detail = await client.get(f"/api/parts/merge-proposals/{pid}")
    assert detail.json()["outcome"] is None  # NOT auto-merged
    assert detail.json()["approves"] == 5


@pytest.mark.asyncio
async def test_threshold_triggers_auto_merge(client, session) -> None:
    """Five approves on a >48h-old proposal triggers the merge."""
    from projects_api.models import PartMergeProposal

    src = await _make_part(client, "Pi Pico H")
    tgt = await _make_part(client, "Pi Pico")
    create = await _propose(client, src["slug"], tgt["slug"])
    pid = create.json()["id"]

    # Backdate the proposal so it clears the 48h gate.
    proposal = await session.get(PartMergeProposal, pid)
    proposal.created_at = datetime.utcnow() - timedelta(hours=49)
    await session.commit()

    last = None
    for voter in ("v1", "v2", "v3", "v4", "v5"):
        last = await client.post(
            f"/api/parts/merge-proposals/{pid}/vote",
            json={"vote": "approve"},
            headers=make_auth_header(voter),
        )
        assert last.status_code == 200

    # The fifth vote (the one that hit threshold) should report the merge
    # outcome inline.
    assert last is not None
    body = last.json()
    assert body["outcome"] == "merged", body
    assert body["resolved_at"] is not None

    # Source part is gone; target is still there.
    src_gone = await client.get(f"/api/parts/{src['slug']}")
    assert src_gone.status_code == 404
    tgt_alive = await client.get(f"/api/parts/{tgt['slug']}")
    assert tgt_alive.status_code == 200


@pytest.mark.asyncio
async def test_threshold_blocked_by_2_to_1_ratio(client, session) -> None:
    """5 approves with 3 rejects fails the ratio (5 < 2*3) even past 48h."""
    from projects_api.models import PartMergeProposal

    src = await _make_part(client, "Pi Pico H")
    tgt = await _make_part(client, "Pi Pico")
    create = await _propose(client, src["slug"], tgt["slug"])
    pid = create.json()["id"]

    proposal = await session.get(PartMergeProposal, pid)
    proposal.created_at = datetime.utcnow() - timedelta(hours=49)
    await session.commit()

    # Front-load the rejects so the ratio gate is already failing by the
    # time the 5th approve lands. (If we approve first, the proposal
    # auto-merges on vote #5 before any rejects arrive.)
    for v in ("r1", "r2", "r3"):
        await client.post(
            f"/api/parts/merge-proposals/{pid}/vote",
            json={"vote": "reject"},
            headers=make_auth_header(v),
        )
    for v in ("a1", "a2", "a3", "a4", "a5"):
        await client.post(
            f"/api/parts/merge-proposals/{pid}/vote",
            json={"vote": "approve"},
            headers=make_auth_header(v),
        )

    detail = await client.get(f"/api/parts/merge-proposals/{pid}")
    body = detail.json()
    assert body["outcome"] is None  # not merged — ratio blocks it
    assert body["approves"] == 5
    assert body["rejects"] == 3


@pytest.mark.asyncio
async def test_auto_merge_moves_bom_rows(client, session) -> None:
    """When a merge fires, BOM rows that referenced source now reference target."""
    from projects_api.models import PartMergeProposal, ProjectBOMItem

    src = await _make_part(client, "Pi Pico H")
    tgt = await _make_part(client, "Pi Pico")

    # Wire up a project + BOM row pointing at source.
    proj = await client.post(
        "/api/projects",
        json={"title": "Demo project"},
        headers=make_auth_header("alice"),
    )
    project_id = proj.json()["id"]
    bom = await client.post(
        f"/api/projects/{project_id}/bom",
        json={"name": "Pi Pico H", "quantity": 1, "part_id": src["id"]},
        headers=make_auth_header("alice"),
    )
    assert bom.status_code == 201
    bom_id = bom.json()["id"]

    create = await _propose(client, src["slug"], tgt["slug"])
    pid = create.json()["id"]
    proposal = await session.get(PartMergeProposal, pid)
    proposal.created_at = datetime.utcnow() - timedelta(hours=49)
    await session.commit()

    for voter in ("v1", "v2", "v3", "v4", "v5"):
        await client.post(
            f"/api/parts/merge-proposals/{pid}/vote",
            json={"vote": "approve"},
            headers=make_auth_header(voter),
        )

    # BOM row now points at the target.
    row = await session.get(ProjectBOMItem, bom_id)
    assert row.part_id == tgt["id"], "BOM row should now reference the merge target"

    # Target's usage_count should have been recomputed to include the
    # moved row.
    detail = await client.get(f"/api/parts/{tgt['slug']}")
    assert detail.json()["usage_count"] == 1
