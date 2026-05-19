"""Tests for the supplier link health checker (issue #122 Phase 2).

Exercises the in-process status update logic and the admin
``recheck-suppliers`` endpoint with a mocked ``httpx`` transport, so we
never hit the real network during tests.
"""

from __future__ import annotations

from datetime import datetime, timedelta
from typing import Optional

import httpx
import pytest

from .conftest import make_auth_header


@pytest.fixture(autouse=True)
def _old_account(monkeypatch: pytest.MonkeyPatch) -> None:
    from projects_api import auth as auth_module

    async def _fake_age(username: str, token: str) -> Optional[datetime]:
        return datetime.utcnow() - timedelta(days=365 * 5)

    monkeypatch.setattr(auth_module, "fetch_account_created_at", _fake_age)
    auth_module._clear_account_age_cache()


def _mock_client(handler) -> httpx.AsyncClient:
    """Return an AsyncClient backed by an in-process MockTransport."""
    transport = httpx.MockTransport(handler)
    return httpx.AsyncClient(transport=transport, timeout=5.0)


@pytest.mark.asyncio
async def test_success_sets_status_ok_and_clears_counters(session) -> None:
    from projects_api.models import Part, PartSupplier
    from projects_api.supplier_health import run_health_check_for_suppliers

    part = Part(
        slug="widget",
        name="Widget",
        status="draft",
        created_by="kev",
        usage_count=0,
    )
    session.add(part)
    await session.flush()
    sup = PartSupplier(
        part_id=part.id,
        supplier_name="ThePiHut",
        url="https://example.com/widget",
        consecutive_failures=2,  # was about to flip but not yet broken
        is_broken=False,
    )
    session.add(sup)
    await session.commit()

    def handler(request):
        return httpx.Response(200)

    async with _mock_client(handler) as client:
        n = await run_health_check_for_suppliers(session, [sup], client=client)
    assert n == 1
    await session.refresh(sup)
    assert sup.last_status == "ok"
    assert sup.last_status_code == 200
    assert sup.is_broken is False
    assert sup.consecutive_failures == 0
    assert sup.last_checked_at is not None


@pytest.mark.asyncio
async def test_three_consecutive_failures_marks_broken(session) -> None:
    from projects_api.models import Part, PartSupplier
    from projects_api.supplier_health import run_health_check_for_suppliers

    part = Part(slug="thing", name="Thing", status="draft", created_by="kev", usage_count=0)
    session.add(part)
    await session.flush()
    sup = PartSupplier(part_id=part.id, url="https://example.com/dead")
    session.add(sup)
    await session.commit()

    def handler(request):
        return httpx.Response(500)

    for expected_fail_count in (1, 2, 3):
        async with _mock_client(handler) as client:
            await run_health_check_for_suppliers(session, [sup], client=client)
        await session.refresh(sup)
        assert sup.consecutive_failures == expected_fail_count
        assert sup.last_status_code == 500
        if expected_fail_count < 3:
            assert sup.is_broken is False
            assert sup.last_status == "unknown"
        else:
            assert sup.is_broken is True
            assert sup.last_status == "broken"


@pytest.mark.asyncio
async def test_success_after_failures_resets_counters(session) -> None:
    from projects_api.models import Part, PartSupplier
    from projects_api.supplier_health import run_health_check_for_suppliers

    part = Part(slug="thing2", name="Thing", status="draft", created_by="kev", usage_count=0)
    session.add(part)
    await session.flush()
    sup = PartSupplier(
        part_id=part.id,
        url="https://example.com/recovered",
        consecutive_failures=3,
        is_broken=True,
        last_status="broken",
    )
    session.add(sup)
    await session.commit()

    def handler(request):
        return httpx.Response(200)

    async with _mock_client(handler) as client:
        await run_health_check_for_suppliers(session, [sup], client=client)
    await session.refresh(sup)
    assert sup.is_broken is False
    assert sup.consecutive_failures == 0
    assert sup.last_status == "ok"
    assert sup.last_status_code == 200


@pytest.mark.asyncio
async def test_timeout_counts_as_failure(session) -> None:
    from projects_api.models import Part, PartSupplier
    from projects_api.supplier_health import run_health_check_for_suppliers

    part = Part(slug="thing3", name="Thing", status="draft", created_by="kev", usage_count=0)
    session.add(part)
    await session.flush()
    sup = PartSupplier(part_id=part.id, url="https://example.com/slow")
    session.add(sup)
    await session.commit()

    def handler(request):
        raise httpx.ConnectTimeout("simulated timeout")

    async with _mock_client(handler) as client:
        await run_health_check_for_suppliers(session, [sup], client=client)
    await session.refresh(sup)
    assert sup.last_status_code is None
    assert sup.consecutive_failures == 1
    assert sup.is_broken is False
    assert sup.last_status == "unknown"


@pytest.mark.asyncio
async def test_admin_recheck_endpoint_runs_synchronously(
    client, monkeypatch
) -> None:
    """The admin force-recheck endpoint should update the supplier rows."""
    # Create a part with a supplier as 'kev' (the default admin).
    create = await client.post(
        "/api/parts",
        json={
            "name": "Pico",
            "supplier_url": "https://example.com/pico",
            "supplier_name": "ThePiHut",
        },
        headers=make_auth_header("kev"),
    )
    assert create.status_code == 201
    slug = create.json()["slug"]

    # Patch the AsyncClient factory inside supplier_health to return a
    # transport-backed client that responds 200 to every request.
    import projects_api.supplier_health as sh

    def handler(request):
        return httpx.Response(200)

    _RealAsyncClient = httpx.AsyncClient

    def _fake_factory(*args, **kwargs):
        return _RealAsyncClient(transport=httpx.MockTransport(handler), timeout=5.0)

    monkeypatch.setattr(sh.httpx, "AsyncClient", _fake_factory)

    resp = await client.post(
        f"/api/admin/parts/{slug}/recheck-suppliers",
        headers=make_auth_header("kev"),
    )
    assert resp.status_code == 200, resp.text
    rows = resp.json()
    assert len(rows) == 1
    assert rows[0]["is_broken"] is False
    assert rows[0]["last_status_code"] == 200
    assert rows[0]["last_status"] == "ok"


@pytest.mark.asyncio
async def test_admin_recheck_endpoint_requires_admin(client) -> None:
    create = await client.post(
        "/api/parts",
        json={"name": "Servo", "supplier_url": "https://example.com/servo"},
        headers=make_auth_header("kev"),
    )
    slug = create.json()["slug"]
    # 'alice' is not in admin_usernames (default is just "kev").
    resp = await client.post(
        f"/api/admin/parts/{slug}/recheck-suppliers",
        headers=make_auth_header("alice"),
    )
    assert resp.status_code == 403


@pytest.mark.asyncio
async def test_admin_recheck_endpoint_404_for_unknown_slug(client) -> None:
    resp = await client.post(
        "/api/admin/parts/does-not-exist/recheck-suppliers",
        headers=make_auth_header("kev"),
    )
    assert resp.status_code == 404


@pytest.mark.asyncio
async def test_part_detail_surfaces_is_broken_flag(client, session) -> None:
    """The view payload should include ``is_broken`` so the frontend
    can render the warning pill."""
    create = await client.post(
        "/api/parts",
        json={"name": "Widget X", "supplier_url": "https://example.com/x"},
        headers=make_auth_header("kev"),
    )
    slug = create.json()["slug"]
    detail = await client.get(f"/api/parts/{slug}")
    assert detail.status_code == 200
    suppliers = detail.json()["suppliers"]
    assert suppliers and suppliers[0]["is_broken"] is False

    # Flip the flag at the DB layer to simulate a broken supplier and
    # check it surfaces through the API.
    from projects_api.models import PartSupplier
    from sqlalchemy import select, update

    await session.execute(
        update(PartSupplier)
        .where(PartSupplier.url == "https://example.com/x")
        .values(is_broken=True, consecutive_failures=3, last_status="broken", last_status_code=500)
    )
    await session.commit()

    detail2 = await client.get(f"/api/parts/{slug}")
    suppliers2 = detail2.json()["suppliers"]
    assert suppliers2[0]["is_broken"] is True
    assert suppliers2[0]["consecutive_failures"] == 3
    assert suppliers2[0]["last_status_code"] == 500
