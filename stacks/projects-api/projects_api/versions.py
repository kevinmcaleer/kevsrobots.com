"""Project versions (epic #433).

A ``projects`` row is the author's **working copy**: the editor autosaves
into it, and into the rows that hang off it (tags, BOM, videos, links,
wiring, diagrams, schematic, images, files). Readers never see it. Pressing
Publish takes a **snapshot** of all of that (:func:`snapshot_project`),
stores it as an immutable ``project_versions`` row and points
``projects.published_version_id`` at it. Everything public serves that
snapshot (:func:`resolve_view`, :class:`LiveView`).

Restoring copies a snapshot back into the working copy
(:func:`apply_snapshot`), after saving the working copy as an
``unpublished`` version so the author can always get back.

Pictures: the wiring SVG and the diagram SVG/PNG files are rewritten in
place on every save, so a version keeps its own copy of each
(``projects/versions/{project_id}/…``, named by content hash, so a picture
that didn't change between versions is stored once). Images and files are
a shared pool referenced by id; one a version still uses is only
soft-deleted (``removed_at``) when the author removes it (issue #439).
"""

from __future__ import annotations

import copy
import hashlib
import logging
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Iterable, Optional

from fastapi import HTTPException, status
from sqlalchemy import delete, func, select
from sqlalchemy.ext.asyncio import AsyncSession

from .models import (
    Part,
    Project,
    ProjectBOMItem,
    ProjectDiagram,
    ProjectFile,
    ProjectImage,
    ProjectLink,
    ProjectSchematic,
    ProjectTag,
    ProjectVersion,
    ProjectVideo,
    ProjectWiring,
)
from .graphics import referenced_ids
from .storage import delete_file, read_file, save_file
from .visibility import DRAFT, NO_STORE

logger = logging.getLogger(__name__)

SNAPSHOT_FORMAT = 1

PUBLISHED = "published"
UNPUBLISHED = "unpublished"

# The project's own fields that a version carries.
PROJECT_FIELDS = (
    "title",
    "short_description",
    "content_md",
    "content_mode",
    "difficulty",
    "estimated_minutes",
    "code_repo_url",
    "cover_image",
    "files_view",
)

BOM_FIELDS = (
    "id", "name", "quantity", "unit", "unit_cost", "currency_code", "supplier_url",
    "sort_order", "part_id", "part_revision_id", "supplier_id",
)
VIDEO_FIELDS = ("id", "youtube_id", "title", "sort_order")
LINK_FIELDS = ("id", "title", "url", "link_type")
IMAGE_FIELDS = ("id", "caption", "sort_order")
FILE_FIELDS = (
    "id", "description", "print_settings", "print_qty", "print_colour", "filament",
)

# Where a picture is stored, not what it shows.
_PICTURE_KEYS = {"svg_path", "png_path", "svg_hash", "png_hash"}


def _now() -> datetime:
    return datetime.now(timezone.utc).replace(tzinfo=None)


def _iso(value: Any) -> Any:
    return value.isoformat() if isinstance(value, datetime) else value


def _row(obj: Any, fields: Iterable[str]) -> dict:
    return {f: _iso(getattr(obj, f, None)) for f in fields}


# --- Snapshots ----------------------------------------------------------------


async def snapshot_project(session: AsyncSession, project: Project) -> dict:
    """Everything a reader sees of ``project``'s working copy, as JSON.

    Picture paths point at the working copy's files here; the version
    gets its own copies when it is stored (:func:`create_version`)."""
    pid = project.id
    snap: dict[str, Any] = {"format": SNAPSHOT_FORMAT}
    for f in PROJECT_FIELDS:
        snap[f] = getattr(project, f, None)
    snap["tags"] = sorted(
        set(
            (
                await session.scalars(select(ProjectTag.tag).where(ProjectTag.project_id == pid))
            ).all()
        )
    )
    snap["bom"] = [
        _row(i, BOM_FIELDS)
        for i in (
            await session.scalars(
                select(ProjectBOMItem)
                .where(ProjectBOMItem.project_id == pid)
                .order_by(ProjectBOMItem.sort_order, ProjectBOMItem.id)
            )
        ).all()
    ]
    snap["videos"] = [
        _row(v, VIDEO_FIELDS)
        for v in (
            await session.scalars(
                select(ProjectVideo)
                .where(ProjectVideo.project_id == pid)
                .order_by(ProjectVideo.sort_order, ProjectVideo.id)
            )
        ).all()
    ]
    snap["links"] = [
        _row(link, LINK_FIELDS)
        for link in (
            await session.scalars(
                select(ProjectLink).where(ProjectLink.project_id == pid).order_by(ProjectLink.id)
            )
        ).all()
    ]
    wiring = (
        await session.scalars(select(ProjectWiring).where(ProjectWiring.project_id == pid))
    ).first()
    snap["wiring"] = (
        {"data": wiring.data, "svg_path": wiring.svg_path} if wiring is not None else None
    )
    snap["diagrams"] = {
        str(d.id): {
            "title": d.title,
            "data": d.data,
            "svg_path": d.svg_path,
            "png_path": d.png_path,
        }
        for d in (
            await session.scalars(
                select(ProjectDiagram).where(ProjectDiagram.project_id == pid).order_by(ProjectDiagram.id)
            )
        ).all()
    }
    schematic = (
        await session.scalars(select(ProjectSchematic).where(ProjectSchematic.project_id == pid))
    ).first()
    snap["schematic"] = (
        {
            "name": schematic.name,
            "description": schematic.description,
            "schematic_data": schematic.schematic_data,
        }
        if schematic is not None
        else None
    )
    # A graphic's old renders (graphics.py) only while the text still shows them.
    shown = referenced_ids(pid, project.content_md, project.cover_image)
    snap["images"] = [
        _row(i, IMAGE_FIELDS)
        for i in (
            await session.scalars(
                select(ProjectImage)
                .where(ProjectImage.project_id == pid, ProjectImage.removed_at.is_(None))
                .order_by(ProjectImage.sort_order, ProjectImage.id)
            )
        ).all()
        if getattr(i, "superseded_at", None) is None or i.id in shown
    ]
    snap["files"] = [
        _row(f, FILE_FIELDS)
        for f in (
            await session.scalars(
                select(ProjectFile)
                .where(ProjectFile.project_id == pid, ProjectFile.removed_at.is_(None))
                .order_by(ProjectFile.id)
            )
        ).all()
    ]
    return snap


def comparable(snap: Optional[dict]) -> Any:
    """``snap`` without storage details, for "is anything different?".

    Row ids are dropped from the BOM / video / link rows (the editor's
    syncs recreate those rows) but kept for images and files, which the
    text refers to by id. Picture paths and hashes go too: the picture
    follows from the wiring / diagram data, which stays."""
    if snap is None:
        return None
    out = {k: v for k, v in snap.items() if k != "format"}
    for key in ("bom", "videos", "links"):
        out[key] = [{k: v for k, v in r.items() if k != "id"} for r in snap.get(key) or []]
    w = snap.get("wiring")
    out["wiring"] = {k: v for k, v in w.items() if k not in _PICTURE_KEYS} if w else None
    out["diagrams"] = {
        did: {k: v for k, v in d.items() if k not in _PICTURE_KEYS}
        for did, d in (snap.get("diagrams") or {}).items()
    }
    out["tags"] = sorted(set(snap.get("tags") or []))
    out["images"] = list(snap.get("images") or [])
    out["files"] = list(snap.get("files") or [])
    return out


def same_content(a: Optional[dict], b: Optional[dict]) -> bool:
    return comparable(a) == comparable(b)


# --- Version rows -------------------------------------------------------------


async def live_version(session: AsyncSession, project: Project) -> Optional[ProjectVersion]:
    vid = getattr(project, "published_version_id", None)
    if vid is None:
        return None
    v = await session.get(ProjectVersion, vid)
    return v if v is not None and v.project_id == project.id else None


async def get_version(session: AsyncSession, project_id: int, number: int) -> Optional[ProjectVersion]:
    return (
        await session.scalars(
            select(ProjectVersion).where(
                ProjectVersion.project_id == project_id, ProjectVersion.number == number
            )
        )
    ).first()


async def list_versions(session: AsyncSession, project_id: int) -> list[ProjectVersion]:
    return list(
        (
            await session.scalars(
                select(ProjectVersion)
                .where(ProjectVersion.project_id == project_id)
                .order_by(ProjectVersion.number.desc())
            )
        ).all()
    )


async def newest_version(session: AsyncSession, project_id: int) -> Optional[ProjectVersion]:
    return (
        await session.scalars(
            select(ProjectVersion)
            .where(ProjectVersion.project_id == project_id)
            .order_by(ProjectVersion.number.desc())
            .limit(1)
        )
    ).first()


async def first_published_at(session: AsyncSession, project_id: int) -> Optional[datetime]:
    return await session.scalar(
        select(func.min(ProjectVersion.published_at)).where(
            ProjectVersion.project_id == project_id,
            ProjectVersion.published_at.is_not(None),
        )
    )


async def has_unpublished_changes(
    session: AsyncSession, project: Project, live: Optional[ProjectVersion] = None
) -> bool:
    """True when the working copy differs from the live version. A project
    with nothing live has nothing to compare with: False."""
    if live is None:
        live = await live_version(session, project)
    if live is None:
        return False
    return not same_content(await snapshot_project(session, project), live.snapshot)


# --- Picture copies -----------------------------------------------------------


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _known_pictures(versions: Iterable[ProjectVersion]) -> dict[str, str]:
    """hash → stored path, for every picture the versions already keep."""
    known: dict[str, str] = {}
    for v in versions:
        snap = v.snapshot or {}
        w = snap.get("wiring") or {}
        if w.get("svg_hash") and w.get("svg_path"):
            known[w["svg_hash"]] = w["svg_path"]
        for d in (snap.get("diagrams") or {}).values():
            for kind in ("svg", "png"):
                h, p = d.get(f"{kind}_hash"), d.get(f"{kind}_path")
                if h and p:
                    known[h] = p
    return known


def _freeze(path: Optional[str], project_id: int, stem: str, ext: str, known: dict[str, str]):
    """A version's own copy of the picture at ``path``: ``(path, hash)``.

    Reuses a copy another version already has when the bytes are the same.
    When the file can't be read (storage down), the version keeps pointing
    at the working file rather than losing the picture."""
    if not path:
        return None, None
    data = read_file(path)
    if data is None:
        logger.warning("Couldn't read %s to keep it with a version", path)
        return path, None
    digest = _sha(data)
    if digest in known:
        return known[digest], digest
    saved = save_file(data, f"{stem}-{digest[:16]}.{ext}", project_id, "versions")
    if saved is None:
        logger.warning("Couldn't store a version's copy of %s", path)
        return path, None
    known[digest] = saved
    return saved, digest


def _freeze_pictures(snap: dict, project_id: int, known: dict[str, str]) -> None:
    w = snap.get("wiring")
    if w:
        w["svg_path"], w["svg_hash"] = _freeze(w.get("svg_path"), project_id, "wiring", "svg", known)
    for did, d in (snap.get("diagrams") or {}).items():
        d["svg_path"], d["svg_hash"] = _freeze(d.get("svg_path"), project_id, f"diagram-{did}", "svg", known)
        d["png_path"], d["png_hash"] = _freeze(d.get("png_path"), project_id, f"diagram-{did}", "png", known)


def _picture_paths(snap: Optional[dict]) -> set[str]:
    paths: set[str] = set()
    snap = snap or {}
    w = snap.get("wiring") or {}
    if w.get("svg_path"):
        paths.add(w["svg_path"])
    for d in (snap.get("diagrams") or {}).values():
        for k in ("svg_path", "png_path"):
            if d.get(k):
                paths.add(d[k])
    return paths


def _version_file(path: str, project_id: int) -> bool:
    """Only files under the versions folder belong to versions."""
    return f"projects/versions/{project_id}/" in path


async def create_version(
    session: AsyncSession,
    project: Project,
    *,
    kind: str,
    snapshot: Optional[dict] = None,
    note: Optional[str] = None,
    restored_from_number: Optional[int] = None,
    published_at: Optional[datetime] = None,
) -> ProjectVersion:
    """Store ``snapshot`` (default: the working copy now) as the project's
    next version, with its own copies of the pictures. Caller commits."""
    snap = copy.deepcopy(snapshot) if snapshot is not None else await snapshot_project(session, project)
    known = _known_pictures(await list_versions(session, project.id))
    _freeze_pictures(snap, project.id, known)
    newest = await session.scalar(
        select(func.max(ProjectVersion.number)).where(ProjectVersion.project_id == project.id)
    )
    number = max(int(project.last_version_number or 0), int(newest or 0)) + 1
    project.last_version_number = number
    version = ProjectVersion(
        project_id=project.id,
        number=number,
        kind=kind,
        snapshot=snap,
        note=(note or "").strip()[:200] or None,
        restored_from_number=restored_from_number,
        published_at=(published_at or _now()) if kind == PUBLISHED else None,
    )
    session.add(version)
    await session.flush()
    return version


class NoChanges(Exception):
    """Publish was asked for, but the working copy is the live version."""


async def publish(
    session: AsyncSession, project: Project, *, note: Optional[str] = None
) -> ProjectVersion:
    """Make the working copy the live version. Caller commits; the router
    adds the slug, activity and badges (:mod:`routers.versions`)."""
    snap = await snapshot_project(session, project)
    live = await live_version(session, project)
    if live is not None and same_content(snap, live.snapshot):
        raise NoChanges()
    version = await create_version(session, project, kind=PUBLISHED, snapshot=snap, note=note)
    project.published_version_id = version.id
    if project.status != "archived":
        project.status = "completed"
    return version


def unpublish(project: Project) -> None:
    project.published_version_id = None
    project.status = DRAFT


# --- Images and files used by versions (issue #439) ---------------------------


def _ids(snap: Optional[dict], key: str) -> set[int]:
    return {int(r["id"]) for r in (snap or {}).get(key) or [] if r.get("id") is not None}


async def versions_using(
    session: AsyncSession, project: Project, key: str, item_id: int
) -> tuple[bool, bool]:
    """Whether the live version, and whether any other version, uses image
    or file ``item_id`` (``key`` is ``images`` or ``files``)."""
    live_id = getattr(project, "published_version_id", None)
    in_live = in_other = False
    for v in await list_versions(session, project.id):
        if item_id in _ids(v.snapshot, key):
            if v.id == live_id:
                in_live = True
            else:
                in_other = True
    return in_live, in_other


async def delete_version(session: AsyncSession, project: Project, version: ProjectVersion) -> None:
    """Delete a non-live version and whatever only it kept: its picture
    copies, and removed images / files no other version uses. Caller
    commits."""
    others = [v for v in await list_versions(session, project.id) if v.id != version.id]
    kept_pictures = set().union(*(_picture_paths(v.snapshot) for v in others)) if others else set()
    for path in _picture_paths(version.snapshot) - kept_pictures:
        if _version_file(path, project.id):
            delete_file(path)
    for key, model in (("images", ProjectImage), ("files", ProjectFile)):
        still_used = set().union(*(_ids(v.snapshot, key) for v in others)) if others else set()
        for item_id in _ids(version.snapshot, key) - still_used:
            row = await session.get(model, item_id)
            if row is not None and row.project_id == project.id and row.removed_at is not None:
                delete_file(row.file_path)
                thumb = getattr(row, "thumbnail_path", None)
                if thumb:
                    delete_file(thumb)
                await session.delete(row)
    await session.delete(version)


# --- Restore ------------------------------------------------------------------


async def _refresh_usage(session: AsyncSession, part_ids: set[int]) -> None:
    for part_id in part_ids:
        part = await session.get(Part, part_id)
        if part is None:
            continue
        count = await session.scalar(
            select(func.count(func.distinct(ProjectBOMItem.project_id))).where(
                ProjectBOMItem.part_id == part_id
            )
        )
        part.usage_count = max(0, int(count or 0))


def _copy_to_working(path: Optional[str], project_id: int, name: str, folder: str) -> Optional[str]:
    """Copy a version's picture to the working copy's own file, so the next
    save (which replaces the working file) never touches the version's."""
    if not path:
        return None
    data = read_file(path)
    if data is None:
        return None
    return save_file(data, name, project_id, folder)


async def apply_snapshot(session: AsyncSession, project: Project, snap: dict) -> None:
    """Make the working copy what ``snap`` describes (issue #437).

    Does not touch ``published_version_id``, ``status`` or the slug.
    Images and files the snapshot uses come back to the gallery; ones it
    doesn't are removed from the working copy (soft, since the version
    saved before the restore still uses them). Caller commits."""
    pid = project.id
    for f in PROJECT_FIELDS:
        if f in snap:
            value = snap[f]
            if f == "content_mode" and not value:
                value = "markdown"
            setattr(project, f, value)
    project.updated_at = _now()

    await session.execute(delete(ProjectTag).where(ProjectTag.project_id == pid))
    for tag in snap.get("tags") or []:
        session.add(ProjectTag(project_id=pid, tag=tag))

    old_parts = set(
        (
            await session.scalars(
                select(ProjectBOMItem.part_id).where(
                    ProjectBOMItem.project_id == pid, ProjectBOMItem.part_id.is_not(None)
                )
            )
        ).all()
    )
    await session.execute(delete(ProjectBOMItem).where(ProjectBOMItem.project_id == pid))
    new_parts: set[int] = set()
    for r in snap.get("bom") or []:
        part_id = r.get("part_id")
        if part_id is not None and await session.get(Part, part_id) is None:
            part_id = None
        session.add(
            ProjectBOMItem(
                project_id=pid,
                name=r.get("name") or "",
                quantity=r.get("quantity") or 1,
                unit=r.get("unit") or "qty",
                unit_cost=r.get("unit_cost"),
                currency_code=r.get("currency_code"),
                supplier_url=r.get("supplier_url"),
                sort_order=r.get("sort_order") or 0,
                part_id=part_id,
                part_revision_id=r.get("part_revision_id") if part_id is not None else None,
                supplier_id=r.get("supplier_id") if part_id is not None else None,
            )
        )
        if part_id is not None:
            new_parts.add(part_id)
    await session.flush()
    await _refresh_usage(session, old_parts | new_parts)

    await session.execute(delete(ProjectVideo).where(ProjectVideo.project_id == pid))
    for r in snap.get("videos") or []:
        session.add(
            ProjectVideo(
                project_id=pid,
                youtube_id=r.get("youtube_id"),
                title=r.get("title"),
                sort_order=r.get("sort_order") or 0,
            )
        )
    await session.execute(delete(ProjectLink).where(ProjectLink.project_id == pid))
    for r in snap.get("links") or []:
        session.add(
            ProjectLink(
                project_id=pid,
                title=r.get("title") or "",
                url=r.get("url") or "",
                link_type=r.get("link_type") or "article",
            )
        )

    # Wiring: one row per project.
    wiring = (
        await session.scalars(select(ProjectWiring).where(ProjectWiring.project_id == pid))
    ).first()
    want = snap.get("wiring")
    if want is None:
        if wiring is not None:
            await session.delete(wiring)
    else:
        svg = _copy_to_working(want.get("svg_path"), pid, "wiring.svg", "wiring")
        if wiring is None:
            wiring = ProjectWiring(project_id=pid, data=want.get("data") or {}, svg_path=svg)
            session.add(wiring)
        else:
            wiring.data = want.get("data") or {}
            if svg:
                wiring.svg_path = svg
    # Diagrams keep their ids, since the text shows them by URL.
    wanted = {int(k): v for k, v in (snap.get("diagrams") or {}).items()}
    existing = {
        d.id: d
        for d in (
            await session.scalars(select(ProjectDiagram).where(ProjectDiagram.project_id == pid))
        ).all()
    }
    for did, d in existing.items():
        if did not in wanted:
            await session.delete(d)
    for did, want_d in wanted.items():
        row = existing.get(did)
        if row is None:
            other = await session.get(ProjectDiagram, did)
            if other is not None:  # an id another project now has; can't happen with serial ids
                continue
            row = ProjectDiagram(id=did, project_id=pid, title=want_d.get("title") or "Diagram", data={})
            session.add(row)
        row.title = want_d.get("title") or row.title
        row.data = want_d.get("data") or {}
        svg = _copy_to_working(want_d.get("svg_path"), pid, f"diagram-{did}.svg", "diagrams")
        png = _copy_to_working(want_d.get("png_path"), pid, f"diagram-{did}.png", "diagrams")
        if svg:
            row.svg_path = svg
        if png:
            row.png_path = png

    schematic = (
        await session.scalars(select(ProjectSchematic).where(ProjectSchematic.project_id == pid))
    ).first()
    want_s = snap.get("schematic")
    if want_s is None:
        if schematic is not None:
            await session.delete(schematic)
    else:
        if schematic is None:
            schematic = ProjectSchematic(project_id=pid)
            session.add(schematic)
        schematic.name = want_s.get("name")
        schematic.description = want_s.get("description")
        schematic.schematic_data = want_s.get("schematic_data")

    for key, model, fields in (
        ("images", ProjectImage, IMAGE_FIELDS),
        ("files", ProjectFile, FILE_FIELDS),
    ):
        rows_by_id = {r["id"]: r for r in snap.get(key) or [] if r.get("id") is not None}
        for row in (await session.scalars(select(model).where(model.project_id == pid))).all():
            if row.id in rows_by_id:
                row.removed_at = None
                for f in fields:
                    if f != "id" and f in rows_by_id[row.id]:
                        setattr(row, f, rows_by_id[row.id][f])
            elif row.removed_at is None:
                row.removed_at = _now()
    await session.flush()


# --- What a reader is shown ---------------------------------------------------


@dataclass
class View:
    """Which state of a project a request is shown.

    ``snapshot`` None means the working copy (the rows themselves);
    otherwise the version's snapshot. ``private`` means the response is
    the author's alone and must not be cached (drafts, previews)."""

    project: Optional[Project]
    snapshot: Optional[dict] = None
    version: Optional[ProjectVersion] = None
    private: bool = False
    is_author: bool = False
    label: str = "working"  # "working", "live" or the version number as text
    headers: dict = field(default_factory=dict)

    @property
    def working(self) -> bool:
        return self.snapshot is None

    def image_ids(self) -> Optional[set[int]]:
        return None if self.snapshot is None else _ids(self.snapshot, "images")

    def file_ids(self) -> Optional[set[int]]:
        return None if self.snapshot is None else _ids(self.snapshot, "files")

    def entry(self, key: str, item_id: int) -> Optional[dict]:
        for r in (self.snapshot or {}).get(key) or []:
            if r.get("id") == item_id:
                return r
        return None


def _not_found(no_store: bool) -> HTTPException:
    return HTTPException(
        status.HTTP_404_NOT_FOUND, detail="Project not found", headers=NO_STORE if no_store else None
    )


async def resolve_view(
    session: AsyncSession,
    project_id: int,
    user: Optional[str],
    version: Optional[str] = None,
    *,
    no_store: bool = False,
) -> View:
    """What ``user`` is shown of project ``project_id``.

    ``version`` is the ``?version=`` query value: ``working`` or a number
    (the author only; 404 for everyone else), or ``live``. Without it the
    author gets the working copy and everyone else the live version.

    * A missing project is left to the caller (``View(project=None)``), so
      the sub-resource routes keep their own 404s.
    * Someone else's project with nothing live: 404 when it's a draft.
      Otherwise (a public row that has never been snapshotted, which the
      startup backfill prevents) the working copy, as before #433."""
    project = await session.get(Project, project_id)
    if project is None:
        return View(project=None)
    is_author = user is not None and user == project.author_username
    # Handlers called directly (by-slug forwarding) pass FastAPI's Query
    # default object rather than a value.
    wanted = version.strip().lower() if isinstance(version, str) else ""
    live = await live_version(session, project)

    if wanted and wanted != "live":
        if not is_author:
            raise _not_found(True)
        if wanted == "working":
            return View(project, private=True, is_author=True, label="working", headers=dict(NO_STORE))
        try:
            number = int(wanted.lstrip("v"))
        except ValueError:
            raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Version not found", headers=NO_STORE)
        v = await get_version(session, project.id, number)
        if v is None:
            raise HTTPException(status.HTTP_404_NOT_FOUND, detail="Version not found", headers=NO_STORE)
        return View(project, v.snapshot, v, private=True, is_author=True, label=str(v.number), headers=dict(NO_STORE))

    if live is not None and (wanted == "live" or not is_author):
        # The author's own look at the live page isn't for caches either.
        private = is_author
        return View(
            project, live.snapshot, live, private=private, is_author=is_author, label="live",
            headers=dict(NO_STORE) if private else {},
        )
    if is_author:
        private = project.status == DRAFT
        return View(project, private=private, is_author=True, label="working",
                    headers=dict(NO_STORE) if private else {})
    if project.status == DRAFT:
        raise _not_found(no_store)
    return View(project, label="working")


# --- Live fields for listings -------------------------------------------------


class LiveView:
    """A project as its live version shows it, for cards and listings.

    Reads like the ORM row (``p.title``, ``p.cover_image`` …) with the
    snapshot's values in place of the working copy's; anything a version
    doesn't carry (slug, author, featured, status …) comes from the row."""

    def __init__(self, project: Project, version: ProjectVersion):
        self._project = project
        self._version = version
        snap = version.snapshot or {}
        for f in PROJECT_FIELDS:
            if f in snap:
                object.__setattr__(self, f, snap[f])
        object.__setattr__(self, "live_tags", list(snap.get("tags") or []))
        object.__setattr__(self, "live_has_bom", bool(snap.get("bom")))
        if version.published_at is not None:
            object.__setattr__(self, "updated_at", version.published_at)

    def __getattr__(self, name: str) -> Any:
        return getattr(self._project, name)

    @property
    def row(self) -> Project:
        return self._project


async def live_views(session: AsyncSession, projects: list[Project]) -> list[Any]:
    """Each project as readers see it: a :class:`LiveView` when it has a
    live version, the row itself otherwise (one query for all of them)."""
    ids = [p.published_version_id for p in projects if getattr(p, "published_version_id", None)]
    versions: dict[int, ProjectVersion] = {}
    if ids:
        versions = {
            v.id: v
            for v in (await session.scalars(select(ProjectVersion).where(ProjectVersion.id.in_(ids)))).all()
        }
    out = []
    for p in projects:
        v = versions.get(getattr(p, "published_version_id", None) or -1)
        out.append(LiveView(p, v) if v is not None and v.project_id == p.id else p)
    return out


async def live_view(session: AsyncSession, project: Project) -> Any:
    return (await live_views(session, [project]))[0]


def row_of(p: Any) -> Project:
    return p.row if isinstance(p, LiveView) else p


async def item_public(session: AsyncSession, project: Project, key: str, item_id: int) -> bool:
    """Whether image / file ``item_id`` is on the public page: the live
    version uses it (or, for a public project never snapshotted, it is in
    the working copy's gallery). Anything else is the author's alone and
    must never be cached for someone else."""
    live = await live_version(session, project)
    if live is not None:
        return item_id in _ids(live.snapshot, key)
    return project.status != DRAFT


async def guard_removal(session: AsyncSession, project: Project, key: str, row: Any) -> bool:
    """Issue #439, before the author deletes image / file ``row`` from the
    working copy. When any version uses it (the live one included) it is
    only soft-deleted (``removed_at``): it leaves the gallery, and the live
    page and older versions keep showing it. Returns True when no version
    uses it, so the caller deletes it for good.

    (The plan had a 409 for pictures on the live page, but every gallery
    picture is in the live version, so the author could never take one out
    of the working copy to publish a version without it.)"""
    in_live, in_other = await versions_using(session, project, key, row.id)
    if in_live or in_other:
        row.removed_at = _now()
        return False
    return True
