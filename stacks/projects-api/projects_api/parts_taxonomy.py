"""Part category taxonomy — a small, self-organising, canonical vocabulary.

Categories are stored as a free string on ``parts.category`` (no FK), so
users can coin new ones rather than waiting on an admin. To keep that from
fragmenting a vocabulary that's *meant* to stay small ("Single Board
Computer" is a type bucket, unlike the open-ended ``family``), we converge
variants onto a single canonical label via a format-insensitive key:
``"Sensor"`` / ``"sensor"`` / ``"motor-driver"`` / ``"Motor Driver"`` all
collapse to one entry. The curated starter list seeds the vocabulary with
human-readable labels; real usage takes over as the catalog grows.

Single source of truth: the parts router (write-path + ``/_meta/categories``)
and the startup re-case migration in ``db.py`` both import from here.

A future release adds admin rename/merge + a lock on the canonical set
(see the GitHub issue) — until then, first-writer's casing wins for any
category not in the curated list.
"""

from __future__ import annotations

import re
from typing import Iterable, Optional

#: Curated starter categories as human-readable display labels. The order is
#: the default shown before per-category usage data accrues.
PART_CATEGORIES: list[str] = [
    "Microcontroller",
    "Single Board Computer",
    "Sensor",
    "Motor",
    "Motor Driver",
    "Display",
    "Actuator",
    "Power",
    "Communication",
    "Passive",
    "Connector",
    "Tool",
    "Kit",
    "Other",
]

_WS = re.compile(r"\s+")
_SEP = re.compile(r"[-_]+")


def canon_key(value: Optional[str]) -> str:
    """Format-insensitive dedup key.

    Lowercases, turns hyphens/underscores into spaces, and collapses
    whitespace, so ``"motor-driver"``, ``"Motor Driver"`` and
    ``"motor  driver"`` all map to ``"motor driver"``.
    """
    s = _SEP.sub(" ", value or "")
    s = _WS.sub(" ", s).strip().lower()
    return s


def clean_label(value: Optional[str]) -> str:
    """Trim and collapse internal whitespace, preserving the user's casing."""
    return _WS.sub(" ", (value or "").strip())


_CURATED_BY_KEY = {canon_key(c): c for c in PART_CATEGORIES}


def canonicalize(raw: Optional[str], existing: Iterable[str] = ()) -> Optional[str]:
    """Resolve a user-entered category to its canonical label.

    * Empty / whitespace-only → ``None`` (uncategorised).
    * Key matches a curated label → the curated casing wins.
    * Key matches an existing DB label → reuse that label's casing.
    * Otherwise → the cleaned input becomes a brand-new category.
    """
    cleaned = clean_label(raw)
    if not cleaned:
        return None
    key = canon_key(cleaned)
    curated = _CURATED_BY_KEY.get(key)
    if curated is not None:
        return curated
    for label in existing:
        if label and canon_key(label) == key:
            return label
    return cleaned


def merge_for_suggestions(db_labels_by_usage: Iterable[str]) -> list[str]:
    """Build the autocomplete list: curated labels first (canonical order),
    then any DB-only categories (already ordered by usage), deduped by key.
    """
    result: list[str] = []
    seen: set[str] = set()
    for c in PART_CATEGORIES:
        k = canon_key(c)
        if k not in seen:
            seen.add(k)
            result.append(c)
    for label in db_labels_by_usage:
        k = canon_key(label)
        if label and k not in seen:
            seen.add(k)
            result.append(label)
    return result
