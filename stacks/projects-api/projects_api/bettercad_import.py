"""BetterCad build steps: reading the .zip BetterCad sends and turning it
into a section of the project's text.

The zip is BetterCad's "Build steps" export (kevinmcaleer/BetterCad#347):

    README.md            front matter (title, steps, generator: BetterCad),
                         "# How to build …", an intro paragraph, the
                         finished picture, "## Parameters", "## Step N: …"
    images/*.png|jpg     a picture per step and images/finished.png
    <model>.bcad         the model, joints and all
    models/*.stl …       printable parts (optional)
    assembly.json        where each part sits in the assembled model (optional)

The parts' STLs also go into the project's 3D model (``<project>.model.json``,
the parts every 3D scene picks from), each where it sits in BetterCad.

Image and file links in README.md are relative. In the project they point
at the stored pictures and files instead, and the whole thing sits between
two markers so a later send from BetterCad (an update) replaces just that
part of the text, keeping everything the author wrote around it.
"""

from __future__ import annotations

import io
import json
import math
import posixpath
import re
import struct
import zipfile
from dataclasses import dataclass, field
from typing import Callable, Optional
from urllib.parse import unquote

from .storage import ALLOWED_FILE_EXTENSIONS

START = "<!-- bettercad:start -->"
END = "<!-- bettercad:end -->"
HEADING = "## Build steps"

IMAGE_EXTENSIONS = {".png", ".jpg", ".jpeg"}
MODEL_FILE_EXTENSIONS = (ALLOWED_FILE_EXTENSIONS | {".bcad", ".urdf", ".glb", ".gltf"}) - {
    ".zip", ".tar", ".gz", ".7z", ".rar",
}
MAX_ENTRIES = 400
MAX_UNPACKED = 150 * 1024 * 1024
ASSEMBLY = "assembly.json"
PART_PREFIX = "bc-"
PNG = b"\x89PNG\r\n\x1a\n"
JPEG = b"\xff\xd8\xff"


class BundleError(ValueError):
    """The zip isn't BetterCad build steps we can take; the message says why."""


@dataclass
class Placed:
    """A part as BetterCad placed it: its STL (``path`` in the zip), its
    middle at home (mm, world) and its turn from the STL (roll, pitch, yaw in
    degrees, R = Rz Ry Rx, about its middle). Copies of one shape share an
    STL, each placed and turned its own way."""
    path: str
    name: str
    position: list[float]
    rotation: Optional[list[float]] = None
    link: Optional[str] = None


@dataclass
class Joint:
    """A BetterCad joint, as URDF has it: the child component hangs from the
    parent (None for Base) about / along ``axis`` through ``origin`` (world,
    mm, at home). Limits are degrees for a hinge, mm for a slider."""
    child: str
    parent: Optional[str]
    kind: str
    origin: list[float]
    axis: list[float]
    lower: float = 0
    upper: float = 0


@dataclass
class Assembly:
    # Every part, in order; several can share an STL.
    parts: list[Placed] = field(default_factory=list)
    joints: list[Joint] = field(default_factory=list)
    # Component -> the component it sits inside, if any.
    link_parent: dict[str, Optional[str]] = field(default_factory=dict)


@dataclass
class Bundle:
    title: str
    steps: int
    readme: str
    # Path in the zip (e.g. ``images/step-01.png``) -> bytes, in zip order
    # with ``images/finished.png`` first, so it can be the cover.
    images: dict[str, bytes] = field(default_factory=dict)
    # Path in the zip -> bytes, for the model and its parts.
    files: dict[str, bytes] = field(default_factory=dict)
    # What ``assembly.json`` says.
    assembly: "Assembly" = field(default_factory=lambda: Assembly())

    @property
    def size(self) -> int:
        return sum(map(len, self.images.values())) + sum(map(len, self.files.values()))


def _ext(path: str) -> str:
    return posixpath.splitext(path)[1].lower()


def _safe_path(name: str) -> str:
    if "\\" in name or name.startswith("/") or re.match(r"^[A-Za-z]:", name):
        raise BundleError(f"The zip has a path that isn't allowed: {name}")
    norm = posixpath.normpath(name)
    if norm.startswith("..") or "/../" in f"/{norm}/":
        raise BundleError(f"The zip has a path that isn't allowed: {name}")
    return norm


def front_matter(readme: str) -> dict[str, str]:
    """The ``key: value`` lines between the first two ``---`` lines."""
    lines = readme.split("\n")
    if not lines or lines[0].strip() != "---":
        return {}
    out: dict[str, str] = {}
    for line in lines[1:]:
        if line.strip() == "---":
            return out
        key, sep, value = line.partition(":")
        if not sep:
            continue
        value = value.strip()
        if value.startswith('"'):
            try:
                value = json.loads(value)
            except ValueError:
                value = value.strip('"')
        out[key.strip()] = str(value)
    return {}


def read_bundle(data: bytes, *, max_image: int, max_file: int) -> Bundle:
    """Checks the zip and reads what the project needs from it."""
    try:
        archive = zipfile.ZipFile(io.BytesIO(data))
    except zipfile.BadZipFile:
        raise BundleError("That isn't a .zip file.")
    infos = [i for i in archive.infolist() if not i.is_dir()]
    if len(infos) > MAX_ENTRIES:
        raise BundleError("The zip has too many files.")
    if sum(i.file_size for i in infos) > MAX_UNPACKED:
        raise BundleError("The zip is too big once unpacked.")

    readme: Optional[str] = None
    assembly: Optional[bytes] = None
    images: dict[str, bytes] = {}
    files: dict[str, bytes] = {}
    for info in infos:
        path = _safe_path(info.filename)
        base = posixpath.basename(path)
        if base.startswith(".") or path.startswith("__MACOSX/"):
            continue
        ext = _ext(path)
        folder = posixpath.dirname(path)
        if path == "README.md":
            readme = archive.read(info).decode("utf-8", errors="replace")
        elif path == ASSEMBLY:
            if info.file_size > max_file:
                raise BundleError(f"A file is too big: {path}")
            assembly = archive.read(info)
        elif folder == "images":
            if ext not in IMAGE_EXTENSIONS:
                raise BundleError(f"Only PNG and JPEG pictures go under images/: {path}")
            if info.file_size > max_image:
                raise BundleError(f"A picture is too big: {path}")
            content = archive.read(info)
            if not (content.startswith(PNG) or content.startswith(JPEG)):
                raise BundleError(f"A picture isn't a PNG or JPEG: {path}")
            images[path] = content
        elif folder in ("", "models"):
            if ext not in MODEL_FILE_EXTENSIONS:
                raise BundleError(f"That kind of file can't go in a project: {path}")
            if info.file_size > max_file:
                raise BundleError(f"A file is too big: {path}")
            files[path] = archive.read(info)
        else:
            raise BundleError(f"The zip has a file in a folder it doesn't use: {path}")

    if readme is None:
        raise BundleError("The zip has no README.md, so it isn't BetterCad build steps.")
    meta = front_matter(readme)
    if meta.get("generator") != "BetterCad":
        raise BundleError("The zip's README.md wasn't made by BetterCad.")
    if "images/finished.png" in images:
        images = {"images/finished.png": images["images/finished.png"], **images}
    try:
        steps = int(meta.get("steps", "0"))
    except ValueError:
        steps = 0
    title = (meta.get("title") or "Untitled model").strip()[:200]
    placed = _assembly(assembly, files) if assembly is not None else Assembly()
    return Bundle(title=title, steps=steps, readme=readme, images=images, files=files, assembly=placed)


def _vec(v: object) -> Optional[list[float]]:
    if not isinstance(v, list) or len(v) != 3:
        return None
    if not all(isinstance(n, (int, float)) and not isinstance(n, bool) and math.isfinite(n) for n in v):
        return None
    return [float(n) for n in v]


def _text(v: object, size: int = 120) -> str:
    return v.strip()[:size] if isinstance(v, str) else ""


def _assembly(raw: bytes, files: dict[str, bytes]) -> Assembly:
    """``assembly.json``: where each STL sits, its component, and the joints.
    Anything odd in it is left out: a part then goes where its STL says, and
    a joint that doesn't make sense isn't made."""
    out = Assembly()
    try:
        data = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, ValueError):
        return out
    if not isinstance(data, dict):
        return out
    parts = data.get("parts")
    for p in parts if isinstance(parts, list) else []:
        if not isinstance(p, dict) or not isinstance(p.get("file"), str):
            continue
        path = posixpath.normpath(p["file"])
        pos = _vec(p.get("position"))
        if path not in files or pos is None:
            continue
        out.parts.append(Placed(
            path=path, name=_text(p.get("name")), position=pos,
            rotation=_vec(p.get("rotation")), link=_text(p.get("link"), 80) or None,
        ))
    joints = data.get("joints")
    for j in joints if isinstance(joints, list) else []:
        if not isinstance(j, dict):
            continue
        origin, axis = _vec(j.get("origin")), _vec(j.get("axis"))
        child = _text(j.get("child"), 80)
        if origin is None or axis is None or not child or math.hypot(*axis) < 1e-9:
            continue
        kind = j.get("kind") if j.get("kind") in ("revolute", "continuous", "prismatic", "fixed") else "fixed"
        lower = j.get("lower") if isinstance(j.get("lower"), (int, float)) else 0
        upper = j.get("upper") if isinstance(j.get("upper"), (int, float)) else 0
        out.joints.append(Joint(
            child=child, parent=_text(j.get("parent"), 80) or None, kind=kind,
            origin=origin, axis=axis, lower=float(lower), upper=float(upper),
        ))
    links = data.get("links")
    for link in links if isinstance(links, list) else []:
        if isinstance(link, dict) and _text(link.get("id"), 80):
            out.link_parent[_text(link.get("id"), 80)] = _text(link.get("parent"), 80) or None
    return out


# --- the project's 3D model ---


def stl_centre(data: bytes) -> Optional[list[float]]:
    """The middle of an STL's bounding box (binary or ASCII), or None."""
    box = stl_box(data)
    return box[0] if box else None


def stl_box(data: bytes) -> Optional[tuple[list[float], list[float]]]:
    """The middle and size of an STL's bounding box, or None."""
    lo = [math.inf] * 3
    hi = [-math.inf] * 3

    def add(x: float, y: float, z: float) -> None:
        for k, v in enumerate((x, y, z)):
            if v < lo[k]:
                lo[k] = v
            if v > hi[k]:
                hi[k] = v

    n = struct.unpack_from("<I", data, 80)[0] if len(data) >= 84 else -1
    if n >= 0 and len(data) == 84 + 50 * n:
        for i in range(n):
            v = struct.unpack_from("<9f", data, 84 + 50 * i + 12)
            add(*v[0:3])
            add(*v[3:6])
            add(*v[6:9])
    else:
        for m in re.finditer(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", data):
            try:
                add(*(float(g) for g in m.groups()))
            except ValueError:
                continue
    if not math.isfinite(lo[0]):
        return None
    return [round((lo[k] + hi[k]) / 2, 3) for k in range(3)], [round(hi[k] - lo[k], 3) for k in range(3)]


def _slug(text: str) -> str:
    return re.sub(r"[^a-z0-9]+", "-", text.lower()).strip("-")


def _r(n: float) -> float:
    return round(n, 3) + 0.0


# --- turns (3×3, row-major lists), as the 3D scene has them ---

Mat = list[float]


def _rot(rpy: Optional[list[float]]) -> Mat:
    """R = Rz(yaw) Ry(pitch) Rx(roll), from degrees."""
    r, p, y = (math.radians(v) for v in (rpy or [0, 0, 0]))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
        -sp, cp * sr, cp * cr,
    ]


def _rpy(m: Mat) -> list[float]:
    pitch = math.asin(max(-1.0, min(1.0, -m[6])))
    if abs(m[6]) < 0.9999999:
        roll, yaw = math.atan2(m[7], m[8]), math.atan2(m[3], m[0])
    else:
        roll, yaw = 0.0, math.atan2(-m[1], m[4])
    return [_r(math.degrees(v)) for v in (roll, pitch, yaw)]


def _t(m: Mat) -> Mat:
    return [m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]]


def _mul(a: Mat, b: Mat) -> Mat:
    return [sum(a[r * 3 + k] * b[k * 3 + c] for k in range(3)) for r in range(3) for c in range(3)]


def _app(m: Mat, v: list[float]) -> list[float]:
    return [m[r * 3] * v[0] + m[r * 3 + 1] * v[1] + m[r * 3 + 2] * v[2] for r in range(3)]


def _cross(a: list[float], b: list[float]) -> list[float]:
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def _dot(a: list[float], b: list[float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _axis_angle(axis: list[float], angle: float) -> Mat:
    x, y, z = axis
    c, s, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return [
        t * x * x + c, t * x * y - s * z, t * x * z + s * y,
        t * x * y + s * z, t * y * y + c, t * y * z - s * x,
        t * x * z - s * y, t * y * z + s * x, t * z * z + c,
    ]


def _from_to(a: list[float], b: list[float]) -> Mat:
    """The shortest turn taking unit ``a`` to unit ``b``, as three.js's
    ``Quaternion.setFromUnitVectors`` picks it (the scene's joint maths)."""
    d = _dot(a, b)
    if d < -1 + 1e-6:
        axis = [-a[1], a[0], 0.0] if abs(a[0]) > abs(a[2]) else [0.0, -a[2], a[1]]
        n = math.hypot(*axis)
        return _axis_angle([v / n for v in axis], math.pi)
    axis = _cross(a, b)
    n = math.hypot(*axis)
    if n < 1e-12:
        return [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
    return _axis_angle([v / n for v in axis], math.atan2(n, d))


def _spin(turn: Mat, child_normal: list[float], normal: list[float]) -> float:
    """The spin (degrees about ``normal``) the scene adds after lining
    ``child_normal`` up against ``normal`` so the child ends up turned ``turn``."""
    base = _from_to(child_normal, [-v for v in normal])
    # A direction square to the child's normal, where the line-up and the turn each take it.
    helper = [1.0, 0, 0] if abs(child_normal[0]) < 0.9 else [0, 1.0, 0]
    side = _cross(child_normal, helper)
    a, b = _app(base, side), _app(turn, side)
    return math.degrees(math.atan2(_dot(_cross(a, b), normal), _dot(a, b)))


def model_parts(bundle: Bundle, url_for: Callable[[str], Optional[str]]) -> list[dict]:
    """A part of the project's 3D model for each part BetterCad placed
    (``assembly.json``; copies of one shape share an STL), or one per STL
    where its STL says, joined as BetterCad joins them.

    The first part of each component leads it: the component's other parts
    hang from it, fixed, and it hangs from the lead part of the component its
    joint hangs from (Base: the first part in no component) with that
    joint's axis as the joined faces' normal, so a hinge turns about the same
    line and a slider slides along it. A child's place is in its parent's
    frame, turns and all.
    """
    asm = bundle.assembly
    placed = [p for p in asm.parts if _ext(p.path) == ".stl" and url_for(p.path)]
    seen = {p.path for p in placed}
    for path, content in bundle.files.items():
        if _ext(path) == ".stl" and path not in seen and url_for(path):
            box = stl_box(content)
            placed.append(Placed(path=path, name="", position=box[0] if box else [0.0, 0.0, 0.0]))
    sizes = {path: (max(box[1]) if (box := stl_box(content)) else 20.0) for path, content in bundle.files.items() if _ext(path) == ".stl"}

    out: list[dict] = []
    used: set[str] = set()
    middle: dict[str, list[float]] = {}
    turn: dict[str, Mat] = {}
    size: dict[str, float] = {}
    lead: dict[Optional[str], str] = {}
    from_assembly = set(id(p) for p in asm.parts)
    for pl in placed:
        file = posixpath.basename(pl.path)
        name = pl.name or posixpath.splitext(file)[0]
        stem = (PART_PREFIX + (_slug(name) or "part"))[:36]
        part_id, n = stem, 2
        while part_id in used:
            part_id, n = f"{stem}-{n}", n + 1
        used.add(part_id)
        middle[part_id], turn[part_id], size[part_id] = pl.position, _rot(pl.rotation), sizes.get(pl.path, 20.0)
        part = {
            "id": part_id, "name": name, "url": url_for(pl.path), "file": file,
            "position": [_r(v) for v in pl.position], "rotation": _rpy(turn[part_id]),
        }
        if pl.link is not None and pl.link in lead:
            _hang(part, lead[pl.link], middle, turn)
        elif id(pl) in from_assembly and pl.link not in lead:
            lead[pl.link] = part_id
        out.append(part)

    by_id = {p["id"]: p for p in out}
    joined = {j.child: j for j in asm.joints}
    for link, child_id in lead.items():
        if link is None:
            continue
        j = joined.get(link)
        # A component with no joint of its own moves with the one it sits in.
        parent_link = j.parent if j else asm.link_parent.get(link)
        parent_id = lead.get(parent_link)
        if parent_id is None or parent_id == child_id:
            continue
        child = by_id[child_id]
        _hang(child, parent_id, middle, turn)
        if j is None:
            continue
        rp, rc = turn[parent_id], turn[child_id]
        length = math.hypot(*j.axis)
        axis = [v / length for v in j.axis]
        normal = _app(_t(rp), axis)
        child_normal = _app(_t(rc), [-v for v in axis])
        rel = _mul(_t(rp), rc)
        at, c = middle[parent_id], middle[child_id]
        joint = {
            "normal": [_r(v) for v in normal],
            "point": [_r(v) for v in _app(_t(rp), [j.origin[k] - at[k] for k in range(3)])],
            "childNormal": [_r(v) for v in child_normal],
            "childPoint": [_r(v) for v in _app(_t(rc), [j.origin[k] - c[k] for k in range(3)])],
            "spin": _r(_spin(rel, child_normal, normal)),
            "gap": 0,
            "distance": max(10, round(size[child_id] * 0.6)),
        }
        if j.kind == "revolute":
            joint.update(type="revolute", lower=max(-180.0, j.lower), upper=min(180.0, j.upper))
        elif j.kind == "continuous":
            joint.update(type="revolute", lower=-180.0, upper=180.0)
        elif j.kind == "prismatic":
            joint.update(type="prismatic", lower=j.lower, upper=j.upper)
        child["joint"] = joint
    return out


def _hang(part: dict, parent_id: str, middle: dict[str, list[float]], turn: dict[str, Mat]) -> None:
    """Puts ``part`` in ``parent_id``'s frame, where it is now."""
    rp, at, c = turn[parent_id], middle[parent_id], middle[part["id"]]
    part["parent"] = parent_id
    part["position"] = [_r(v) for v in _app(_t(rp), [c[k] - at[k] for k in range(3)])]
    part["rotation"] = _rpy(_mul(_t(rp), turn[part["id"]]))


def print_counts(bundle: Bundle) -> dict[str, int]:
    """How many of each STL to print: one per part BetterCad placed from it."""
    counts: dict[str, int] = {}
    for p in bundle.assembly.parts:
        counts[p.path] = counts.get(p.path, 0) + 1
    return counts


# What a part keeps from the model when BetterCad sends it again: how the
# author showed it in the 3D scene.
_SHOWN = ("hidden", "highlight", "explode", "arrow", "xray", "sockets", "wiringPart")


def merge_model(old: Optional[dict], parts: list[dict], now_ms: int) -> dict:
    """The project's 3D model with BetterCad's parts in it: the last send's
    parts (ids starting ``bc-``) give way to these, and the author's own
    parts, views and steps stay. BetterCad's joints win; a part the author
    joined in the 3D scene, that BetterCad doesn't join, keeps that joint."""
    model = dict(old) if isinstance(old, dict) and isinstance(old.get("parts"), list) else {"version": 1, "stamp": 0, "parts": []}
    before = {p.get("id"): p for p in model["parts"] if isinstance(p, dict)}
    mine = [p for p in model["parts"] if not (isinstance(p, dict) and str(p.get("id", "")).startswith(PART_PREFIX))]
    sent = []
    for p in parts:
        was = before.get(p["id"])
        if isinstance(was, dict) and "parent" in p:
            # BetterCad joins it: its joint wins, the author's look stays,
            # and so does how it comes apart in an exploded view.
            p = {**p, **{k: was[k] for k in _SHOWN if k in was}}
            if isinstance(was.get("joint"), dict) and isinstance(p.get("joint"), dict):
                for k in ("distance", "explodeDir", "explodeAuto"):
                    if k in was["joint"]:
                        p["joint"][k] = was["joint"][k]
        elif isinstance(was, dict):
            # BetterCad doesn't join it: a joint the author made stays.
            p = {**p, **{k: was[k] for k in _SHOWN + ("parent", "joint") if k in was}}
            if "parent" in was:
                p["position"], p["rotation"] = was.get("position", p["position"]), was.get("rotation", p["rotation"])
        sent.append(p)
    stamp = model.get("stamp")
    stamp = stamp if isinstance(stamp, (int, float)) and not isinstance(stamp, bool) else 0
    model.update(version=1, stamp=max(now_ms, int(stamp) + 1), parts=sent + mine)
    return model


_LINK = re.compile(r"(\]\()([^)\s]+)(\))")


def _strip_front_matter(readme: str) -> str:
    lines = readme.split("\n")
    if lines and lines[0].strip() == "---":
        for i, line in enumerate(lines[1:], start=1):
            if line.strip() == "---":
                return "\n".join(lines[i + 1:])
    return readme


def section(readme: str, url_for: Callable[[str], Optional[str]]) -> str:
    """README.md as the project's "Build steps" section, between the markers.

    The ``# How to build …`` title goes (the project has its own), every
    other heading moves down a level under ``## Build steps``, and each
    relative link ``url_for`` knows becomes that URL.
    """
    body = _strip_front_matter(readme).strip("\n").split("\n")
    out: list[str] = []
    fence = False
    for line in body:
        if line.lstrip().startswith("```"):
            fence = not fence
        if not fence and re.match(r"^# ", line):
            continue
        if not fence and re.match(r"^#{2,5} ", line):
            line = "#" + line
        out.append(line)

    def rewrite(m: re.Match[str]) -> str:
        url = url_for(posixpath.normpath(unquote(m.group(2))))
        return f"{m.group(1)}{url}{m.group(3)}" if url else m.group(0)

    text = _LINK.sub(rewrite, "\n".join(out).strip("\n"))
    return f"{START}\n{HEADING}\n\n{text}\n{END}"


def splice(content_md: Optional[str], new_section: str) -> str:
    """The text with ``new_section`` in place of the last import's, or added
    at the end when there wasn't one."""
    text = content_md or ""
    start = text.find(START)
    end = text.find(END, start + 1) if start >= 0 else -1
    if start >= 0 and end >= 0:
        return text[:start] + new_section + text[end + len(END):]
    if not text.strip():
        return new_section + "\n"
    lead = "" if text.endswith("\n\n") else "\n" if text.endswith("\n") else "\n\n"
    return text + lead + new_section + "\n"
