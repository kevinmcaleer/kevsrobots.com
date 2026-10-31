"""Seed the parts catalog from a Snakie parts library (issue #266).

    python -m projects_api.import_snakie_parts <path to a Snakie library>
    python -m projects_api.import_snakie_parts --github [ref]

e.g. ``examples/parts/snakie-standard`` in kevinmcaleer/Snakie: one folder
per part, each with ``parts.yml`` and optionally an image and a mesh.
Snakie's Standard library is the master copy of the catalog, so
``--github`` downloads it straight from GitHub (the ``master`` branch, or
the branch, tag or commit given) instead of needing a checkout on the
server.

Each part is matched to the catalog by slug (Snakie's ``id``), then by
name, then by part number, and created as a draft when nothing matches.
For a part that exists, the Snakie fields (manufacturer, package, voltage,
pin spacing, dimensions, definition) are refreshed, while name,
description, part number, category and tags are only filled in where the
catalog has none, so community edits survive. Each change is recorded as
a revision by ``snakie-import``. A run that changes nothing writes nothing,
so the import can be repeated.

The library's images carry no record of where they came from, so each
imported photo is credited to the library with no licence. That flags it
for review on the admin parts page.
"""

from __future__ import annotations

import argparse
import asyncio
import io
import logging
import sys
import tarfile
import tempfile
import urllib.request
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import yaml
from sqlalchemy import func, select
from sqlalchemy.ext.asyncio import AsyncSession

from .db import add_part_spec_columns_if_missing, create_all, get_sessionmaker
from .models import Part, PartPhoto, PartRevision, PartSupplier
from .parts_taxonomy import canonicalize
from .snakie_parts import spec_from_definition, stored_definition
from .storage import ALLOWED_IMAGE_EXTENSIONS, delete_file, generate_filename, read_file, save_file

logger = logging.getLogger(__name__)

AUTHOR = "snakie-import"
GITHUB_REPO = "kevinmcaleer/Snakie"
GITHUB_LIBRARY_PATH = "examples/parts/snakie-standard"
MESH_EXTENSIONS = {".stl", ".glb", ".gltf", ".obj"}


@dataclass
class ImportReport:
    created: list[str] = field(default_factory=list)
    updated: list[str] = field(default_factory=list)
    unchanged: list[str] = field(default_factory=list)
    skipped: list[str] = field(default_factory=list)

    def summary(self) -> str:
        return (
            f"{len(self.created)} created, {len(self.updated)} updated, "
            f"{len(self.unchanged)} unchanged, {len(self.skipped)} skipped"
        )


def _now() -> datetime:
    return datetime.now(timezone.utc).replace(tzinfo=None)


def _library_label(root: Path) -> str:
    manifest = root / "library.yml"
    if manifest.is_file():
        try:
            data = yaml.safe_load(manifest.read_text(encoding="utf-8")) or {}
            name = data.get("name") or data.get("id") or root.name
            version = data.get("version")
            return f"{name} {version}" if version else str(name)
        except yaml.YAMLError:
            pass
    return root.name


async def _find_part(session: AsyncSession, slug: str, name: str, mpn: Optional[str]) -> Optional[Part]:
    part = await session.scalar(select(Part).where(Part.slug == slug))
    if part is None and name:
        part = await session.scalar(
            select(Part).where(func.lower(Part.name) == name.lower()).limit(1)
        )
    if part is None and mpn:
        part = await session.scalar(
            select(Part).where(func.lower(Part.mpn) == mpn.lower()).limit(1)
        )
    return part


def _suppliers_json(suppliers: list[PartSupplier]) -> list[dict]:
    return [
        {
            "name": s.supplier_name, "url": s.url, "country_code": s.country_code,
            "unit_cost": s.unit_cost, "currency_code": s.currency_code,
        }
        for s in suppliers
    ]


def _first_paragraph(text: Optional[str]) -> Optional[str]:
    text = (text or "").strip()
    return text or None


async def _import_photo(session: AsyncSession, part: Part, folder: Path, raw: dict, source: str) -> bool:
    """Add the library's image as a photo, once. Returns whether it did."""
    image = raw.get("image")
    if not isinstance(image, str) or "/" in image:
        return False
    path = folder / image
    if not path.is_file() or path.suffix.lower() not in ALLOWED_IMAGE_EXTENSIONS:
        return False
    existing = await session.scalar(
        select(PartPhoto.id).where(PartPhoto.part_id == part.id, PartPhoto.source == source).limit(1)
    )
    if existing is not None:
        return False
    stored = save_file(path.read_bytes(), generate_filename(image, part.id), part.id, "part_images")
    if stored is None:
        logger.warning("Could not store the image for %s", part.slug)
        return False
    count = await session.scalar(select(func.count(PartPhoto.id)).where(PartPhoto.part_id == part.id))
    session.add(PartPhoto(
        part_id=part.id, title=None, file_path=stored, filename=image,
        sort_order=int(count or 0), created_by=AUTHOR, source=source, licence=None,
    ))
    return True


def _import_mesh(part: Part, folder: Path, raw: dict) -> Optional[str]:
    """The storage path for the library's mesh, re-saved only when its bytes
    changed. ``None`` leaves the part's mesh as it is."""
    mesh = raw.get("mesh")
    if not isinstance(mesh, str) or "/" in mesh:
        return None
    path = folder / mesh
    if not path.is_file() or path.suffix.lower() not in MESH_EXTENSIONS:
        return None
    content = path.read_bytes()
    if part.mesh_file_path and read_file(part.mesh_file_path) == content:
        return None
    stored = save_file(content, generate_filename(mesh, part.id), part.id, "part_meshes")
    if stored and part.mesh_file_path:
        delete_file(part.mesh_file_path)
    return stored


async def import_part(session: AsyncSession, folder: Path, report: ImportReport, library: str) -> None:
    try:
        raw = yaml.safe_load((folder / "parts.yml").read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError) as e:
        report.skipped.append(f"{folder.name}: {e}")
        return
    slug = str(raw.get("id") or folder.name).strip().lower()
    name = str(raw.get("name") or "").strip()
    if not slug or len(name) < 2:
        report.skipped.append(f"{folder.name}: no id or name")
        return
    mpn = (str(raw["partNumber"]).strip() or None) if raw.get("partNumber") else None
    category = None
    if raw.get("family"):
        existing = (await session.scalars(select(Part.category).where(Part.category.is_not(None)).distinct())).all()
        category = canonicalize(str(raw["family"]), list(existing))
    tags = [str(t).strip() for t in (raw.get("tags") or []) if str(t).strip()]
    spec = spec_from_definition(raw)
    spec["definition"] = stored_definition(raw)
    source = f"{library} (Snakie)"

    part = await _find_part(session, slug, name, mpn)
    now = _now()
    created = part is None
    if created:
        part = Part(
            slug=slug, name=name[:200], mpn=mpn, description_md=_first_paragraph(raw.get("description")),
            tags=tags, status="draft", category=category, created_by=AUTHOR,
            created_at=now, updated_at=now, usage_count=0,
        )
        session.add(part)
        await session.flush()

    before = (
        part.description_md, part.mpn, part.category, list(part.tags or []),
        {f: getattr(part, f) for f in ("manufacturer", "package", "voltage", "pin_spacing",
                                       "dimensions_mm", "definition", "mesh_file_path")},
    )
    # Snakie owns the spec; the catalog keeps whatever people wrote.
    for column, value in spec.items():
        setattr(part, column, value)
    if not part.description_md:
        part.description_md = _first_paragraph(raw.get("description"))
    if not part.mpn and mpn:
        part.mpn = mpn
    if not part.category and category:
        part.category = category
    merged = list(part.tags or [])
    merged += [t for t in tags if t.lower() not in {m.lower() for m in merged}]
    part.tags = merged
    mesh_path = _import_mesh(part, folder, raw)
    if mesh_path:
        part.mesh_file_path = mesh_path
    photo_added = await _import_photo(session, part, folder, raw, source)

    after = (
        part.description_md, part.mpn, part.category, list(part.tags or []),
        {f: getattr(part, f) for f in ("manufacturer", "package", "voltage", "pin_spacing",
                                       "dimensions_mm", "definition", "mesh_file_path")},
    )
    if not created and before == after:
        if photo_added:
            report.updated.append(slug)
        else:
            report.unchanged.append(slug)
        await session.commit()
        return

    suppliers = (await session.scalars(select(PartSupplier).where(PartSupplier.part_id == part.id))).all()
    rev = PartRevision(
        part_id=part.id, author=AUTHOR,
        change_summary=("Imported from " if created else "Updated from ") + library,
        name=part.name, sku=part.sku, mpn=part.mpn, description_md=part.description_md,
        image_url=part.image_url, tags=list(part.tags or []), category=part.category,
        family=part.family, symbol_id=part.symbol_id,
        suppliers_json=_suppliers_json(list(suppliers)), created_at=now,
        manufacturer=part.manufacturer, package=part.package, voltage=part.voltage,
        pin_spacing=part.pin_spacing, dimensions_mm=part.dimensions_mm,
        definition=part.definition, mesh_file_path=part.mesh_file_path,
        datasheet_url=part.datasheet_url,
    )
    session.add(rev)
    await session.flush()
    part.current_revision_id = rev.id
    part.updated_at = now
    await session.commit()
    (report.created if created else report.updated).append(slug)


async def import_library(root: Path, session: AsyncSession) -> ImportReport:
    report = ImportReport()
    library = _library_label(root)
    for folder in sorted(p for p in root.iterdir() if p.is_dir() and not p.name.startswith(".")):
        if (folder / "parts.yml").is_file():
            await import_part(session, folder, report, library)
    return report


async def _main(path: Path) -> int:
    await create_all()
    await add_part_spec_columns_if_missing()
    async with get_sessionmaker()() as session:
        report = await import_library(path, session)
    print(report.summary())
    for line in report.skipped:
        print("  skipped", line)
    return 0


def github_tarball_url(ref: str) -> str:
    return f"https://codeload.github.com/{GITHUB_REPO}/tar.gz/{ref}"


def extract_library(tarball: bytes, dest: Path, library_path: str = GITHUB_LIBRARY_PATH) -> Path:
    """Unpack only ``library_path`` from a GitHub source tarball into ``dest``
    and return the library folder. GitHub wraps everything in one top-level
    ``<repo>-<ref>/`` folder, which is dropped. Links and anything that would
    land outside ``dest`` are skipped."""
    root = dest.resolve()
    prefix = library_path.strip("/") + "/"
    found = False
    with tarfile.open(fileobj=io.BytesIO(tarball), mode="r:gz") as tar:
        for member in tar.getmembers():
            parts = member.name.split("/", 1)
            if len(parts) < 2 or not parts[1].startswith(prefix):
                continue
            if not (member.isfile() or member.isdir()):
                continue
            rel = parts[1][len(prefix):]
            target = (root / rel).resolve()
            if target != root and root not in target.parents:
                continue
            found = True
            if member.isdir():
                target.mkdir(parents=True, exist_ok=True)
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            src = tar.extractfile(member)
            if src is not None:
                target.write_bytes(src.read())
    if not found:
        raise ValueError(f"{library_path} is not in the downloaded tarball")
    return root


def download_github_library(ref: str, dest: Path) -> Path:
    with urllib.request.urlopen(github_tarball_url(ref), timeout=120) as resp:
        data = resp.read()
    return extract_library(data, dest)


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("path", type=Path, nargs="?", help="a Snakie library folder (one folder per part)")
    parser.add_argument(
        "--github", nargs="?", const="master", metavar="REF",
        help=f"download {GITHUB_LIBRARY_PATH} from github.com/{GITHUB_REPO} (default branch: master)",
    )
    args = parser.parse_args(argv)
    if args.github:
        with tempfile.TemporaryDirectory() as tmp:
            try:
                library = download_github_library(args.github, Path(tmp) / "library")
            except Exception as exc:  # network, 404, bad tarball
                print(f"Could not download the Snakie library ({args.github}): {exc}", file=sys.stderr)
                return 2
            return asyncio.run(_main(library))
    if args.path is None:
        parser.error("give a library folder or --github")
    if not args.path.is_dir():
        print(f"{args.path} is not a folder", file=sys.stderr)
        return 2
    return asyncio.run(_main(args.path))


if __name__ == "__main__":
    sys.exit(main())
