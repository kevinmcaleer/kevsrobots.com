"""Tests for the auto-verify lifecycle (issue #122 Phase 2).

Promotion rules:
* >= 2 distinct revision authors EXCLUDING the original creator.
* >= 1 supplier that has been health-checked AND is not broken.
* Part has existed for >= 7 days.

These tests exercise ``compute_part_status`` directly so we can control
``Part.created_at`` without time-travelling the request handlers.
"""

from __future__ import annotations

from datetime import datetime, timedelta, timezone
from typing import Optional

import pytest

from .conftest import make_auth_header


@pytest.fixture(autouse=True)
def _old_account(monkeypatch: pytest.MonkeyPatch) -> None:
    from projects_api import auth as auth_module

    async def _fake_age(username: str, token: str) -> Optional[datetime]:
        return datetime.utcnow() - timedelta(days=365 * 5)

    monkeypatch.setattr(auth_module, "fetch_account_created_at", _fake_age)
    auth_module._clear_account_age_cache()


def _naive_now() -> datetime:
    return datetime.now(timezone.utc).replace(tzinfo=None)


async def _seed_part(session, *, slug: str, created_at: datetime) -> int:
    from projects_api.models import Part, PartRevision

    part = Part(
        slug=slug,
        name=slug.title(),
        status="draft",
        created_by="kev",
        created_at=created_at,
        updated_at=created_at,
        usage_count=0,
    )
    session.add(part)
    await session.flush()
    # Initial revision by the creator (NOT counted in the "other authors"
    # threshold).
    session.add(
        PartRevision(
            part_id=part.id,
            author="kev",
            change_summary="Initial draft",
            name=part.name,
            created_at=created_at,
        )
    )
    await session.commit()
    return part.id


async def _add_revision(session, part_id: int, author: str) -> None:
    from projects_api.models import PartRevision

    session.add(
        PartRevision(
            part_id=part_id,
            author=author,
            change_summary="Tweak",
            name="(any)",
            created_at=_naive_now(),
        )
    )
    await session.commit()


async def _add_supplier(
    session, part_id: int, *, healthy: bool = True
) -> None:
    from projects_api.models import PartSupplier

    sup = PartSupplier(
        part_id=part_id,
        url="https://example.com/x",
        last_checked_at=_naive_now() if healthy else None,
        is_broken=not healthy if healthy else False,
        last_status="ok" if healthy else None,
    )
    if not healthy:
        sup.last_checked_at = None
    session.add(sup)
    await session.commit()


@pytest.mark.asyncio
async def test_promotion_requires_all_three_rules(session) -> None:
    from projects_api.models import Part
    from projects_api.parts_lifecycle import compute_part_status

    old_enough = _naive_now() - timedelta(days=10)
    part_id = await _seed_part(session, slug="widget", created_at=old_enough)
    # Old-enough age + zero other authors + no supplier → still draft.
    part = await session.get(Part, part_id)
    await compute_part_status(session, part)
    assert part.status == "draft"

    # Add one other author — still draft (need 2).
    await _add_revision(session, part_id, "alice")
    await compute_part_status(session, part)
    assert part.status == "draft"

    # Add second other author — still draft (no healthy supplier yet).
    await _add_revision(session, part_id, "bob")
    await compute_part_status(session, part)
    assert part.status == "draft"

    # Add a healthy supplier — now all three rules pass.
    await _add_supplier(session, part_id, healthy=True)
    await compute_part_status(session, part)
    assert part.status == "verified"
    assert part.verified_at is not None
    assert part.verified_signals >= 1


@pytest.mark.asyncio
async def test_promotion_blocked_by_young_age(session) -> None:
    from projects_api.models import Part
    from projects_api.parts_lifecycle import compute_part_status

    young = _naive_now() - timedelta(days=2)
    part_id = await _seed_part(session, slug="fresh", created_at=young)
    await _add_revision(session, part_id, "alice")
    await _add_revision(session, part_id, "bob")
    await _add_supplier(session, part_id, healthy=True)

    part = await session.get(Part, part_id)
    await compute_part_status(session, part)
    assert part.status == "draft"
    assert part.verified_at is None


@pytest.mark.asyncio
async def test_promotion_blocked_by_unhealthy_supplier(session) -> None:
    from projects_api.models import Part
    from projects_api.parts_lifecycle import compute_part_status

    old_enough = _naive_now() - timedelta(days=10)
    part_id = await _seed_part(session, slug="dead-link", created_at=old_enough)
    await _add_revision(session, part_id, "alice")
    await _add_revision(session, part_id, "bob")
    # Add a supplier that has never been checked (so doesn't count).
    await _add_supplier(session, part_id, healthy=False)

    part = await session.get(Part, part_id)
    await compute_part_status(session, part)
    assert part.status == "draft"


@pytest.mark.asyncio
async def test_promotion_excludes_original_creator_from_threshold(session) -> None:
    from projects_api.models import Part
    from projects_api.parts_lifecycle import compute_part_status

    old_enough = _naive_now() - timedelta(days=10)
    part_id = await _seed_part(session, slug="solo", created_at=old_enough)
    # Two additional revisions but both by the same creator — should NOT
    # count toward the "2 other authors" rule.
    await _add_revision(session, part_id, "kev")
    await _add_revision(session, part_id, "kev")
    await _add_supplier(session, part_id, healthy=True)

    part = await session.get(Part, part_id)
    await compute_part_status(session, part)
    assert part.status == "draft"


@pytest.mark.asyncio
async def test_promotion_is_idempotent(session) -> None:
    from projects_api.models import Part
    from projects_api.parts_lifecycle import compute_part_status

    old_enough = _naive_now() - timedelta(days=10)
    part_id = await _seed_part(session, slug="repeat", created_at=old_enough)
    await _add_revision(session, part_id, "alice")
    await _add_revision(session, part_id, "bob")
    await _add_supplier(session, part_id, healthy=True)

    part = await session.get(Part, part_id)
    await compute_part_status(session, part)
    assert part.status == "verified"
    signals_after_first = part.verified_signals
    verified_at_first = part.verified_at

    # Running it again on an already-verified part is a no-op (we don't
    # bump signals because we never enter the promotion branch).
    await compute_part_status(session, part)
    assert part.status == "verified"
    assert part.verified_signals == signals_after_first
    assert part.verified_at == verified_at_first


@pytest.mark.asyncio
async def test_missing_part_reports_table_does_not_raise(session) -> None:
    """If Phase 3 hasn't shipped yet, the demotion check must silently
    no-op rather than crash on the missing table."""
    from projects_api.models import Part
    from projects_api.parts_lifecycle import _has_open_disputed_report

    old_enough = _naive_now() - timedelta(days=10)
    part_id = await _seed_part(session, slug="phase3-pending", created_at=old_enough)
    # The part_reports table does not exist in the test schema; helper
    # must return False instead of raising.
    result = await _has_open_disputed_report(session, part_id)
    assert result is False


@pytest.mark.asyncio
async def test_lifecycle_promotes_after_put_update(client, session) -> None:
    """End-to-end: when an edit pushes the part over the threshold, the
    PUT response should already show ``status='verified'``."""
    from projects_api.models import Part, PartRevision, PartSupplier

    # Set the scene: a part old enough, with a healthy supplier, and one
    # other author's revision already in place. The next PUT (by a
    # second other author) should trigger promotion.
    create = await client.post(
        "/api/parts",
        json={
            "name": "Almost Verified",
            "supplier_url": "https://example.com/av",
        },
        headers=make_auth_header("kev"),
    )
    assert create.status_code == 201
    slug = create.json()["slug"]
    part_id = create.json()["id"]

    # Backdate the part by 10 days and the supplier's last_checked_at
    # so the lifecycle rules pass.
    old = _naive_now() - timedelta(days=10)
    from sqlalchemy import update

    await session.execute(
        update(Part).where(Part.id == part_id).values(created_at=old)
    )
    await session.execute(
        update(PartSupplier)
        .where(PartSupplier.part_id == part_id)
        .values(last_checked_at=old, is_broken=False, last_status="ok", last_status_code=200)
    )
    # Drop in a first "other-author" revision.
    session.add(
        PartRevision(
            part_id=part_id,
            author="alice",
            change_summary="alice's tweak",
            name="Almost Verified",
            created_at=old,
        )
    )
    await session.commit()

    # Now bob edits — should be the second other author.
    upd = await client.put(
        f"/api/parts/{slug}",
        json={"description_md": "bob added docs", "change_summary": "bob's docs"},
        headers=make_auth_header("bob"),
    )
    assert upd.status_code == 200, upd.text
    assert upd.json()["status"] == "verified"
    assert upd.json()["verified_at"] is not None
    assert upd.json()["verified_signals"] >= 1
