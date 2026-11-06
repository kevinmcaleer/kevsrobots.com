"""Snakie ``parts.yml`` interop for the parts catalog (issue #266).

Snakie's part model (``docs/parts-library.md`` in kevinmcaleer/Snakie) is
the richer one. A catalog part keeps its searchable, editable fields in
columns (name, manufacturer, package, voltage, …) and the rest of the
Snakie ``PartDefinition`` (headers of pins, geometry, electrical model,
symbol) as JSON in ``parts.definition``. Converting is a matter of mapping
names, which lives here so the router and the importer agree:

==================  ==================
Snakie              catalog column
==================  ==================
``id``              ``slug``
``name``            ``name``
``description``     ``description_md`` (first line, on create)
``manufacturer``    ``manufacturer``
``family``          ``category`` (Snakie's family is a category: Sensor…)
``partNumber``      ``mpn``
``package``         ``package``
``pinSpacing``      ``pin_spacing``
``voltage``         ``voltage``
``dimensions``      ``dimensions_mm``
==================  ==================
"""

from __future__ import annotations

from typing import Any, Optional

import yaml

# Keys of a Snakie definition that point at files beside parts.yml. They
# mean nothing once the definition is stored here; the export swaps in URLs.
FILE_KEYS = ("image", "help", "mesh")
# Runtime keys the Snakie editor (and the embedded bundle) put on a part:
# ``imageData`` is the photo inlined as a data URL. Never stored.
RUNTIME_KEYS = ("imageData",)
# Ours, not Snakie's (issue #269): the PartPhoto the pins were placed over.
# Kept in the stored definition, left out of parts.yml.
HOST_KEYS = ("photo_id",)


def spec_from_definition(definition: dict) -> dict:
    """The catalog column values a Snakie definition carries. Keys are only
    present when the definition has a usable value for them."""
    out: dict[str, Any] = {}
    for key, limit in (("manufacturer", 120), ("voltage", 60)):
        value = definition.get(key)
        if isinstance(value, str) and value.strip():
            out[key] = value.strip()[:limit]
    if definition.get("package") in ("THT", "SMD"):
        out["package"] = definition["package"]
    spacing = definition.get("pinSpacing")
    if isinstance(spacing, (int, float)) and not isinstance(spacing, bool) and spacing > 0:
        out["pin_spacing"] = float(spacing)
    dims = definition.get("dimensions")
    if isinstance(dims, dict):
        try:
            out["dimensions_mm"] = {"width": float(dims["width"]), "height": float(dims["height"])}
        except (KeyError, TypeError, ValueError):
            pass
    return out


def stored_definition(definition: dict) -> dict:
    """What to keep in ``parts.definition``: everything except the file
    references, which only make sense inside a Snakie library folder, and
    the inlined image data the editor carries at runtime (the rear face's
    too)."""
    out = {k: v for k, v in definition.items() if k not in FILE_KEYS and k not in RUNTIME_KEYS}
    rear = out.get("rear")
    if isinstance(rear, dict):
        out["rear"] = {k: v for k, v in rear.items() if k not in FILE_KEYS and k not in RUNTIME_KEYS}
    return out


def to_parts_yml(
    part: Any,
    *,
    image_url: Optional[str] = None,
    mesh_url: Optional[str] = None,
) -> str:
    """Serialise a catalog part as Snakie ``parts.yml``. The catalog columns
    win over the stored definition, since they're what people edit here."""
    out: dict[str, Any] = {"id": part.slug, "name": part.name}
    definition = dict(part.definition or {})
    description = (part.description_md or "").strip().split("\n\n")[0].strip()
    if description:
        out["description"] = description
    elif definition.get("description"):
        out["description"] = definition["description"]
    for key, value in (
        ("manufacturer", part.manufacturer),
        ("family", part.category),
        ("tags", list(part.tags or []) or None),
        ("package", part.package),
        ("pinSpacing", part.pin_spacing),
        ("voltage", part.voltage),
        ("partNumber", part.mpn),
        ("dimensions", part.dimensions_mm),
    ):
        if value not in (None, "", []):
            out[key] = value
    for key, value in definition.items():
        if (key not in out and key not in FILE_KEYS and key not in HOST_KEYS
                and key not in ("id", "name", "description")):
            out[key] = value
    # Snakie requires a headers list, even an empty one.
    out.setdefault("headers", [])
    if image_url:
        out["image"] = image_url
    if mesh_url:
        out["mesh"] = mesh_url
    return yaml.safe_dump(out, sort_keys=False, allow_unicode=True, width=1000)


# --- 3-D pin maps (``definition.sockets3d``) --------------------------------
#
# Where a part's pins are on its 3-D model, for wiring in project 3D scenes
# (markdown-studio Layers' CatalogSockets: ``{size, sockets: [...]}``, each
# socket a plug's place on a port or a bare pin). Saved from a scene with
# "Save pins to the catalog part", or shipped with a Snakie part.

PIN_MAP_SUMMARIES = ("3D sockets for its model", "3D pin map for its model")


def pin_ports(definition: Optional[dict]) -> dict[str, list[str]]:
    """A definition's ports as 3D scenes name them (project-wiring.js
    ``portsOf``): each pin group, each header with ungrouped pins
    (``header-<n>``), each connector (``connector-<n>``), its pin names."""
    d = definition or {}
    ports: dict[str, list[str]] = {}
    for i, h in enumerate(d.get("headers") or []):
        if not isinstance(h, dict):
            continue
        pins = [p for p in (h.get("pins") or []) if isinstance(p, dict) and p.get("name")]
        for p in pins:
            if p.get("group"):
                ports.setdefault(str(p["group"]), []).append(str(p["name"]))
        loose = [str(p["name"]) for p in pins if not p.get("group")]
        if loose:
            ports[f"header-{i + 1}"] = loose
    for i, c in enumerate(d.get("connectors") or []):
        if not isinstance(c, dict):
            continue
        names = [str(p["name"]) for p in (c.get("pins") or []) if isinstance(p, dict) and p.get("name")]
        if names:
            ports[f"connector-{i + 1}"] = names
    return ports


def pin_map_stats(definition: Optional[dict]) -> tuple[int, int]:
    """(pins placed on the 3-D model, pins in the 2-D pinout)."""
    ports = pin_ports(definition)
    every = {n for names in ports.values() for n in names}
    sockets = ((definition or {}).get("sockets3d") or {}).get("sockets") or []
    placed: set[str] = set()
    for s in sockets:
        if not isinstance(s, dict):
            continue
        if s.get("connector") == "pin":
            if s.get("pin") in every:
                placed.add(str(s["pin"]))
        elif s.get("port") in ports:
            placed.update(ports[str(s["port"])])
    return len(placed), len(every)
