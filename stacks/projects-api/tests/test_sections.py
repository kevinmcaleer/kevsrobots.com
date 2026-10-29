"""Canonical project sections on list cards (issue #259)."""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from projects_api.sections import SECTIONS, canonical_sections
from .conftest import make_auth_header

SECTIONS_JS = Path(__file__).resolve().parents[3] / "web" / "assets" / "js" / "project-sections.js"


def test_matches_headings_in_canonical_order() -> None:
    md = "\n".join([
        "# Intro",
        "## Code",
        "# wiring:",
        "# Bill-of-materials",
        "### Not Code really",
    ])
    assert canonical_sections(md) == ["bill-of-materials", "wiring", "code"]


def test_code_repository_is_its_own_section() -> None:
    md = "# Code repository\n\nhttps://github.com/example/robot\n"
    assert canonical_sections(md) == ["code-repository"]
    assert canonical_sections(md + "# Code\n") == ["code", "code-repository"]


def test_ignores_headings_in_code_fences_and_plain_text() -> None:
    md = "Code\n```\n# Wiring\n```\n~~~~\n# Code\n~~~~\n#Code\n"
    assert canonical_sections(md) == []


def test_structured_bom_counts() -> None:
    assert canonical_sections(None, has_bom=True) == ["bill-of-materials"]


@pytest.mark.skipif(not SECTIONS_JS.exists(), reason="web/ not checked out")
def test_same_list_as_project_sections_js() -> None:
    js = SECTIONS_JS.read_text()
    pairs = re.findall(r"id: '([^']+)',\s*heading: '([^']+)'", js)
    assert pairs == SECTIONS


@pytest.mark.asyncio
async def test_list_reports_sections(client) -> None:
    headers = make_auth_header()
    created = await client.post(
        "/api/projects",
        json={"title": "Sectioned Robot", "content_md": "# Wiring\n\n# Code\n"},
        headers=headers,
    )
    project_id = created.json()["id"]

    listed = (await client.get("/api/projects")).json()
    assert listed[0]["sections"] == ["wiring", "code"]

    await client.post(
        f"/api/projects/{project_id}/bom",
        json={"name": "Servo", "quantity": 2},
        headers=headers,
    )
    mine = (await client.get("/api/projects/my/list", headers=headers)).json()
    assert mine[0]["sections"] == ["bill-of-materials", "wiring", "code"]
