"""Clean an SVG the browser drew before we store and serve it (issues
#268, #270).

The wiring workspace and the diagram tool export their canvas as SVG in
the browser and upload it, and the API serves it back as an image for
the project page and the Markdown that links it. An SVG opened on its
own is a document that can run script, so what comes in is parsed and
rebuilt with only drawing in it: no script, no foreignObject, no event
handlers, and links only to fragments or inlined raster images. It is
also served with a CSP that forbids script, as a second fence.
"""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from typing import Optional

SVG_NS = "http://www.w3.org/2000/svg"
XLINK_NS = "http://www.w3.org/1999/xlink"

# Elements that never belong in a picture we serve.
_BLOCKED = {"script", "foreignobject", "iframe", "object", "embed", "audio", "video", "handler", "listener"}
_SAFE_HREF = re.compile(r"^(#|data:image/(png|jpe?g|gif|webp);base64,)", re.I)
# A url( that isn't a fragment or an inline image. The quote is inside the
# lookahead: as `['"]?` before it, the regex backtracked past the quote and
# rejected `url("#id")`, which is how browsers write computed styles.
_BAD_STYLE = re.compile(r"javascript:|expression\s*\(|url\(\s*(?!['\"]?\s*(?:#|data:image/))", re.I)

# Served with every stored SVG: drawing only, images inline.
SVG_CSP = "default-src 'none'; style-src 'unsafe-inline'; img-src data:; font-src data:"


class UnsafeSvg(ValueError):
    pass


def _local(tag: str) -> str:
    return tag.rsplit("}", 1)[-1].lower()


def clean_svg(text: str, max_bytes: int = 15 * 1024 * 1024) -> str:
    """The SVG in ``text`` with everything but drawing taken out. Raises
    ``UnsafeSvg`` when it isn't an SVG we can read."""
    if not isinstance(text, str) or not text.strip():
        raise UnsafeSvg("The SVG is empty")
    if len(text.encode("utf-8")) > max_bytes:
        raise UnsafeSvg("The SVG is too large")
    # No DTDs: entity tricks are the classic XML attack, and a drawing
    # doesn't need one.
    if re.search(r"<!DOCTYPE|<!ENTITY", text, re.I):
        raise UnsafeSvg("The SVG can't carry a DOCTYPE")
    try:
        root = ET.fromstring(text)
    except ET.ParseError as exc:
        raise UnsafeSvg(f"The SVG doesn't parse: {exc}") from exc
    if _local(root.tag) != "svg":
        raise UnsafeSvg("That isn't an SVG")

    def scrub(el: ET.Element) -> None:
        for child in list(el):
            if not isinstance(child.tag, str) or _local(child.tag) in _BLOCKED:
                el.remove(child)
                continue
            scrub(child)
        for name in list(el.attrib):
            local = _local(name)
            value = el.attrib[name]
            if local.startswith("on"):
                del el.attrib[name]
            elif local == "href" and not _SAFE_HREF.match(value.strip()):
                del el.attrib[name]
            elif local == "style" and _BAD_STYLE.search(value):
                del el.attrib[name]
            elif re.match(r"\s*javascript:", value, re.I):
                del el.attrib[name]
        # <style> text: drop rules that fetch anything.
        if _local(el.tag) == "style" and el.text and _BAD_STYLE.search(el.text):
            el.text = ""

    scrub(root)
    ET.register_namespace("", SVG_NS)
    ET.register_namespace("xlink", XLINK_NS)
    return ET.tostring(root, encoding="unicode")


def safe_svg_or_none(text: Optional[str]) -> Optional[str]:
    try:
        return clean_svg(text) if text else None
    except UnsafeSvg:
        return None
