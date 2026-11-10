"""BetterCad build steps: reading the .zip BetterCad sends and turning it
into a section of the project's text.

The zip is BetterCad's "Build steps" export (kevinmcaleer/BetterCad#347):

    README.md            front matter (title, steps, generator: BetterCad),
                         "# How to build …", an intro paragraph, the
                         finished picture, "## Parameters", "## Step N: …"
    images/*.png|jpg     a picture per step and images/finished.png
    <model>.bcad         the model, joints and all
    models/*.stl …       printable parts (optional)

Image and file links in README.md are relative. In the project they point
at the stored pictures and files instead, and the whole thing sits between
two markers so a later send from BetterCad (an update) replaces just that
part of the text, keeping everything the author wrote around it.
"""

from __future__ import annotations

import io
import json
import posixpath
import re
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
PNG = b"\x89PNG\r\n\x1a\n"
JPEG = b"\xff\xd8\xff"


class BundleError(ValueError):
    """The zip isn't BetterCad build steps we can take; the message says why."""


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
    return Bundle(title=title, steps=steps, readme=readme, images=images, files=files)


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
