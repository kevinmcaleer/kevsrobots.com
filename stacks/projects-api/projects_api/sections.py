"""Canonical project sections (issue #259).

Mirrors ``web/assets/js/project-sections.js``, the one list the editor's
``/`` commands and the project page navigation read. The hub and search
cards show which of these a project has, so the list endpoints report
them; ``tests/test_sections.py`` checks the two lists stay in step.
"""

from __future__ import annotations

import re
from typing import Optional

# (id, heading) in the order the page navigation shows them.
SECTIONS: list[tuple[str, str]] = [
    ("bill-of-materials", "Bill of Materials"),
    ("assembly-instructions", "Assembly instructions"),
    ("wiring", "Wiring"),
    ("code", "Code"),
    ("code-repository", "Code repository"),
    ("3d-printed-parts", "3D printed parts"),
    ("videos", "Videos"),
]

_HEADING = re.compile(r"^ {0,3}(#{1,6})\s+(.+?)\s*#*\s*$")
_FENCE = re.compile(r"^ {0,3}(`{3,}|~{3,})")


def _normalise(text: str) -> str:
    # "Bill of Materials", "bill of materials:" and "Bill-of-materials"
    # are the same section, as in project-sections.js.
    return re.sub(r"[^a-z0-9]+", " ", text.lower()).strip()


_BY_NAME = {_normalise(heading): sid for sid, heading in SECTIONS}


def canonical_sections(content_md: Optional[str], has_bom: bool = False) -> list[str]:
    """Ids of the canonical sections a project has, in canonical order.

    A section counts when the Markdown has a heading naming it (outside
    code fences). ``has_bom`` adds the Bill of Materials for projects
    whose parts list lives in the structured BOM rather than the text.
    """
    found: set[str] = set()
    if has_bom:
        found.add("bill-of-materials")
    fence: Optional[str] = None
    for line in (content_md or "").splitlines():
        m = _FENCE.match(line)
        if m:
            marker = m.group(1)
            if fence is None:
                fence = marker
            elif marker[0] == fence[0] and len(marker) >= len(fence):
                fence = None
            continue
        if fence is not None:
            continue
        m = _HEADING.match(line)
        if m:
            sid = _BY_NAME.get(_normalise(m.group(2)))
            if sid:
                found.add(sid)
    return [sid for sid, _ in SECTIONS if sid in found]
