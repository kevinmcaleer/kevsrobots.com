"""Layers graphic renders: keep the draft's and the versions', no more.

A Layers graphic is a gallery picture (``*.layers.png``) whose PNG holds
its layers. Saving or updating one uploads a new render with
``replaces=<old id>``; the new render joins the old one's graphic
(``graphic_id``, the first render's id) and the old one is marked
``superseded_at``. A superseded render:

- never shows in the author's gallery or the "pictures aren't on your
  page" list (``list_images`` leaves it out), and isn't snapshotted into a
  new version unless the text still shows it;
- is pruned on the next save once it's an hour old (time to undo): deleted
  for good when neither the working copy nor any version shows it, or
  soft-removed (``removed_at``) when a version does, so that version's page
  keeps it. Those kept renders are the graphic's history.

"Shows it" means the text (``content_md``) or cover links to it, or, for a
version with no ``::: gallery`` block, that it's in the version's pictures
(the public page shows those under the text then).

:func:`tidy_graphics` clears the piles made before renders were linked:
graphics the working copy's text doesn't show, deleted unless a version
shows them (then kept for it, hidden).
"""

from __future__ import annotations

import re
from datetime import datetime, timedelta, timezone
from typing import Any, Iterable, Optional

from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from .models import Instruction, InstructionStep, Project, ProjectImage, ProjectVersion
from .storage import delete_file

# Time to undo an update before the superseded render can go.
GRACE = timedelta(hours=1)

_GALLERY = re.compile(r"^[ \t]{0,3}:::[ \t]*gallery\b", re.I | re.M)


def _now() -> datetime:
    return datetime.now(timezone.utc).replace(tzinfo=None)


def is_graphic(img: ProjectImage) -> bool:
    return (img.filename or "").lower().endswith(".layers.png")


def referenced_ids(project_id: int, *texts: Optional[str]) -> set[int]:
    """This project's picture ids that ``texts`` link to."""
    pattern = re.compile(rf"/api/projects/{project_id}/images/(\d+)/view")
    out: set[int] = set()
    for t in texts:
        if t:
            out.update(int(m) for m in pattern.findall(t))
    return out


def snapshot_shows(project_id: int, snap: Optional[dict]) -> set[int]:
    """The picture ids a version's page shows."""
    snap = snap or {}
    text = snap.get("content_md") or ""
    ids = referenced_ids(project_id, text, snap.get("cover_image"))
    if not _GALLERY.search(text):
        ids.update(int(r["id"]) for r in snap.get("images") or [] if r.get("id") is not None)
    return ids


async def _versions(session: AsyncSession, project_id: int) -> list[ProjectVersion]:
    return list(
        (
            await session.scalars(
                select(ProjectVersion)
                .where(ProjectVersion.project_id == project_id)
                .order_by(ProjectVersion.number)
            )
        ).all()
    )


async def version_usage(session: AsyncSession, project_id: int) -> dict[int, list[ProjectVersion]]:
    """Picture id -> the versions whose page shows it."""
    out: dict[int, list[ProjectVersion]] = {}
    for v in await _versions(session, project_id):
        for i in snapshot_shows(project_id, v.snapshot):
            out.setdefault(i, []).append(v)
    return out


async def working_shows(session: AsyncSession, project: Project, *extra_text: Optional[str]) -> set[int]:
    """The picture ids the working copy shows: its text and cover, and (to
    be safe) the Instruction Builder's steps, which can use the gallery's pictures."""
    steps = (
        await session.execute(
            select(InstructionStep.description, InstructionStep.canvas_json)
            .join(Instruction, Instruction.id == InstructionStep.instruction_id)
            .where(Instruction.project_id == project.id)
        )
    ).all()
    step_text = [t for row in steps for t in row]
    return referenced_ids(project.id, project.content_md, project.cover_image, *extra_text, *step_text)


async def supersede(session: AsyncSession, project: Project, new: ProjectImage, replaces: Optional[int]) -> None:
    """``new`` is a newer render of picture ``replaces``. Caller commits."""
    if replaces is None:
        return
    old = await session.get(ProjectImage, replaces)
    if old is None or old.project_id != project.id or old.id == new.id:
        return
    new.graphic_id = old.graphic_id or old.id
    if old.graphic_id is None:
        old.graphic_id = old.id
    if old.superseded_at is None:
        old.superseded_at = _now()


def _drop(img: ProjectImage) -> None:
    delete_file(img.file_path)


async def prune_superseded(
    session: AsyncSession, project: Project, *, grace: timedelta = GRACE
) -> dict[str, int]:
    """Superseded renders older than ``grace`` that the working copy doesn't
    show: deleted, or soft-removed when a version shows them. Caller commits."""
    rows = (
        await session.scalars(
            select(ProjectImage).where(
                ProjectImage.project_id == project.id,
                ProjectImage.superseded_at.is_not(None),
                ProjectImage.removed_at.is_(None),
            )
        )
    ).all()
    cutoff = _now() - grace
    rows = [r for r in rows if r.superseded_at <= cutoff]
    counts = {"deleted": 0, "kept": 0}
    if not rows:
        return counts
    shown = await working_shows(session, project)
    used = await version_usage(session, project.id)
    for r in rows:
        if r.id in shown:
            continue
        if r.id in used:
            r.removed_at = _now()
            counts["kept"] += 1
        else:
            _drop(r)
            await session.delete(r)
            counts["deleted"] += 1
    return counts


async def tidy_graphics(
    session: AsyncSession, project: Project, *, text: Optional[str] = None, dry_run: bool = False
) -> dict[str, Any]:
    """Old graphic renders in the gallery that the working copy (and
    ``text``, the editor's unsaved text) doesn't show. Deleted unless a
    version shows them, then kept for it but hidden. Caller commits."""
    rows = (
        await session.scalars(
            select(ProjectImage).where(
                ProjectImage.project_id == project.id, ProjectImage.removed_at.is_(None)
            )
        )
    ).all()
    shown = await working_shows(session, project, text)
    used = await version_usage(session, project.id)
    gone = [r for r in rows if is_graphic(r) and r.id not in shown]
    delete = [r for r in gone if r.id not in used]
    keep = [r for r in gone if r.id in used]
    if not dry_run:
        now = _now()
        for r in keep:
            r.removed_at = now
            r.superseded_at = r.superseded_at or now
        for r in delete:
            _drop(r)
            await session.delete(r)
    return {
        "deleted": len(delete),
        "kept": len(keep),
        "deleted_ids": sorted(r.id for r in delete),
        "kept_ids": sorted(r.id for r in keep),
    }


def _label(v: ProjectVersion) -> dict[str, Any]:
    return {
        "number": v.number,
        "kind": v.kind,
        "published_at": v.published_at.isoformat() if v.published_at else None,
    }


async def histories(session: AsyncSession, project: Project) -> dict[int, list[dict[str, Any]]]:
    """For each current graphic render: its earlier renders still kept
    (newest first), each with the versions that show it."""
    rows = (
        await session.scalars(
            select(ProjectImage).where(
                ProjectImage.project_id == project.id, ProjectImage.graphic_id.is_not(None)
            )
        )
    ).all()
    by_graphic: dict[int, list[ProjectImage]] = {}
    for r in rows:
        by_graphic.setdefault(r.graphic_id, []).append(r)
    used = await version_usage(session, project.id)
    live_id = project.published_version_id
    out: dict[int, list[dict[str, Any]]] = {}
    for renders in by_graphic.values():
        current = [r for r in renders if r.superseded_at is None and r.removed_at is None]
        earlier = sorted(
            (r for r in renders if r.superseded_at is not None),
            key=lambda r: (r.uploaded_at, r.id),
            reverse=True,
        )
        entries = [
            {
                "id": r.id,
                "uploaded_at": r.uploaded_at.isoformat() if r.uploaded_at else None,
                "versions": [dict(_label(v), live=v.id == live_id) for v in used.get(r.id, [])],
            }
            for r in earlier
            if r.id in used
        ]
        if not entries:
            continue
        for c in current:
            out[c.id] = entries
    return out


def hidden(img: ProjectImage) -> bool:
    return img.superseded_at is not None


def visible(rows: Iterable[ProjectImage]) -> list[ProjectImage]:
    return [r for r in rows if not hidden(r)]
