"""The visual parts catalog's data (issue #265)."""

from __future__ import annotations

from datetime import datetime, timedelta
from typing import Optional

import pytest

from projects_api.import_snakie_parts import import_library
from .conftest import make_auth_header
from .test_snakie_parts import LIBRARY


@pytest.fixture(autouse=True)
def _old_account(monkeypatch: pytest.MonkeyPatch) -> None:
    from projects_api import auth as auth_module

    async def _fake_age(username: str, token: str) -> Optional[datetime]:
        return datetime.utcnow() - timedelta(days=365 * 5)

    monkeypatch.setattr(auth_module, "fetch_account_created_at", _fake_age)
    auth_module._clear_account_age_cache()


@pytest.mark.asyncio
async def test_catalog_lists_every_part_with_facets(client, session) -> None:
    await import_library(LIBRARY, session)
    resp = await client.post("/api/parts", json={"name": "Jumper wires", "tags": ["wire"]},
                             headers=make_auth_header())
    slug = resp.json()["slug"]
    await client.put(f"/api/parts/{slug}", json={"change_summary": "Shop", "suppliers": [
        {"url": "https://thepihut.com/jumpers", "unit_cost": 2.0, "currency_code": "GBP"},
        {"name": "Datasheet", "url": "https://example.com/d.pdf", "kind": "datasheet"},
    ]}, headers=make_auth_header())

    resp = await client.get("/api/parts/_catalog")
    assert resp.status_code == 200, resp.text
    rows = {r["slug"]: r for r in resp.json()}
    assert list(rows) == [slug, "pico", "sg90"]  # by name
    assert rows["pico"]["manufacturer"] == "Raspberry Pi"
    assert rows["pico"]["cover_url"].startswith("/api/parts/pico/photos/")
    assert rows["pico"]["has_price"] is False
    assert rows[slug]["has_price"] is True and rows[slug]["tags"] == ["wire"]
    assert rows["sg90"]["cover_url"] is None
