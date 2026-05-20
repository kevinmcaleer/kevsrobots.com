"""Branded "Made with kevsrobots.com/projects" slide renderer.

Issue #178, Phase 3b/3c: every instruction export (PDF / GIF / MP4)
ends with the same branded slide so projects shared from kevsrobots.com
carry attribution back to the platform. The slide is generated
server-side as a PNG via Pillow rather than hand-authored as an asset
so it can include the per-project title (and so a future polish PR
can layer locale / theme / accent tweaks without touching frontend
build).

Pure text on a dark background — deliberately no logo file dependency.
Robust against missing fonts (falls back to ``ImageFont.load_default()``
if DejaVuSans-Bold isn't installed) so it works in CI / dev machines
even when the Dockerfile's ``fonts-dejavu-core`` apt package isn't
present.
"""

from __future__ import annotations

import io

from PIL import Image, ImageDraw, ImageFont

# Brand palette — deliberately kept inline so this module has no
# config dependency. Dark background reads well against the kevsrobots
# orange/red accent without needing the accent itself; the headline and
# subtext stay white / light grey for high contrast.
_BG_COLOR = (13, 27, 42)          # #0d1b2a — dark navy
_HEADLINE_COLOR = (255, 255, 255) # white
_SUBTEXT_COLOR = (192, 200, 209)  # light grey
_FOOTNOTE_COLOR = (148, 163, 184) # softer grey
_TOPNOTE_COLOR = (148, 163, 184)

# Font sizes are derived from the canvas width so the slide reads well
# at any resolution the caller picks.
_HEADLINE_SCALE = 0.055   # ~5.5% of width — fits "Made with kevsrobots.com/projects" on one line
_SUBTEXT_SCALE = 0.030    # ~3.0% of width
_FOOTNOTE_SCALE = 0.024   # smaller wordmark / footer
_TOPNOTE_SCALE = 0.024    # italic top line for project title

_FONT_BOLD_NAMES = ("DejaVuSans-Bold.ttf", "DejaVuSansBold.ttf", "DejaVuSans-Bold")
_FONT_REGULAR_NAMES = ("DejaVuSans.ttf", "DejaVuSans")
_FONT_ITALIC_NAMES = ("DejaVuSans-Oblique.ttf", "DejaVuSans-Italic.ttf")


def _try_truetype(names: tuple[str, ...], size: int) -> ImageFont.ImageFont:
    """Best-effort truetype load with a graceful fallback.

    Pillow's ``ImageFont.truetype`` raises ``OSError`` when the font
    isn't found via its lookup paths; we cycle through each candidate
    and fall back to ``load_default()`` so the slide still renders on
    machines without the DejaVu package (e.g. CI runners, fresh dev
    boxes). The default font is bitmap and small — but it's only a
    fallback, the Dockerfile installs DejaVu for production.
    """
    for name in names:
        try:
            return ImageFont.truetype(name, size)
        except (OSError, IOError):  # noqa: PERF203 — at most 3 attempts
            continue
    return ImageFont.load_default()


def _measure(draw: ImageDraw.ImageDraw, text: str, font: ImageFont.ImageFont) -> tuple[int, int]:
    """Return (width, height) of the rendered text in pixels.

    Uses ``textbbox`` (modern Pillow) when available, falling back to
    ``textsize`` for older versions. The bbox is (x0, y0, x1, y1) at
    the supplied anchor; the size is just the diff.
    """
    if hasattr(draw, "textbbox"):
        bbox = draw.textbbox((0, 0), text, font=font)
        return bbox[2] - bbox[0], bbox[3] - bbox[1]
    # Very old Pillow — keep the fallback for completeness.
    return draw.textsize(text, font=font)  # type: ignore[attr-defined]


def render_made_with_slide(
    width: int = 1200,
    height: int = 900,
    project_title: str | None = None,
) -> bytes:
    """Return PNG bytes for the 'Made with kevsrobots.com/projects' slide.

    The dimensions default to the canvas size the Instruction Builder
    uses (1200x900), so the slide composes well alongside step PNGs
    without aspect-ratio surprises in the GIF / MP4 pipeline.

    ``project_title`` shows up as a small italic line at the top
    ("Instructions for <title>") when supplied. The headline + subtitle
    + wordmark are unconditional.
    """
    img = Image.new("RGB", (width, height), _BG_COLOR)
    draw = ImageDraw.Draw(img)

    headline_font = _try_truetype(_FONT_BOLD_NAMES, max(16, int(width * _HEADLINE_SCALE)))
    subtext_font = _try_truetype(_FONT_REGULAR_NAMES, max(12, int(width * _SUBTEXT_SCALE)))
    footnote_font = _try_truetype(_FONT_REGULAR_NAMES, max(10, int(width * _FOOTNOTE_SCALE)))
    topnote_font = _try_truetype(_FONT_ITALIC_NAMES, max(10, int(width * _TOPNOTE_SCALE)))

    # Top: optional "Instructions for <title>" line.
    if project_title:
        # Trim aggressively — anything longer than ~60 chars at the
        # default width starts to wrap weirdly.
        title_clean = project_title.strip()
        if len(title_clean) > 60:
            title_clean = title_clean[:57].rstrip() + "…"
        top_line = f"Instructions for {title_clean}"
        tw, th = _measure(draw, top_line, topnote_font)
        # Top padding ~7% of height.
        ty = int(height * 0.07)
        draw.text(
            ((width - tw) // 2, ty),
            top_line,
            font=topnote_font,
            fill=_TOPNOTE_COLOR,
        )

    # Centre: headline + subtitle, vertically centred.
    headline = "Made with kevsrobots.com/projects"
    subtext = "Build your own at kevsrobots.com/projects"

    hw, hh = _measure(draw, headline, headline_font)
    sw, sh = _measure(draw, subtext, subtext_font)
    gap = int(height * 0.035)
    block_h = hh + gap + sh
    block_top = (height - block_h) // 2

    draw.text(
        ((width - hw) // 2, block_top),
        headline,
        font=headline_font,
        fill=_HEADLINE_COLOR,
    )
    draw.text(
        ((width - sw) // 2, block_top + hh + gap),
        subtext,
        font=subtext_font,
        fill=_SUBTEXT_COLOR,
    )

    # Footer wordmark — small, centred near the bottom.
    wordmark = "kevsrobots"
    ww, wh = _measure(draw, wordmark, footnote_font)
    wy = height - int(height * 0.08) - wh
    draw.text(
        ((width - ww) // 2, wy),
        wordmark,
        font=footnote_font,
        fill=_FOOTNOTE_COLOR,
    )

    buf = io.BytesIO()
    img.save(buf, format="PNG")
    return buf.getvalue()
