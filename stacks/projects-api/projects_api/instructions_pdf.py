"""PDF rendering for the build-instructions export (issue #178, Phase 2b).

Renders an :class:`InstructionExportRequest` to an in-memory PDF using
ReportLab's Platypus high-level API. The browser supplies each step's
canvas as a PNG dataURL (rendered client-side with Fabric.js at 2× for
print quality); this module decodes them, lays them out per the
``steps_per_page`` knob (1 / 2 / 4), and returns the binary.

Kept deliberately small and route-agnostic so the FastAPI handler can
stay focused on auth + validation. The single public entry point is
:func:`render_instructions_pdf`.
"""

from __future__ import annotations

import base64
import io
from datetime import datetime, timezone

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.platypus import (
    BaseDocTemplate,
    Frame,
    Image,
    KeepInFrame,
    PageBreak,
    PageTemplate,
    Paragraph,
    Spacer,
)

from .schemas import InstructionExportRequest, InstructionExportStep

# A4 minus 15mm margins on every side. Recomputed once here so the
# layout functions can lay things out against a known content box.
_PAGE_W, _PAGE_H = A4
_MARGIN = 15 * mm
_CONTENT_W = _PAGE_W - 2 * _MARGIN
_CONTENT_H = _PAGE_H - 2 * _MARGIN

# Data-URL prefix the route enforces; we re-check here as defence in
# depth in case the helper is ever called from somewhere else.
_PNG_DATAURL_PREFIX = "data:image/png;base64,"


def _decode_image(image_data_url: str) -> io.BytesIO:
    """Strip the dataURL prefix and base64-decode into an in-memory file."""
    if not image_data_url.startswith(_PNG_DATAURL_PREFIX):
        raise ValueError("image_data_url must start with 'data:image/png;base64,'")
    raw = image_data_url[len(_PNG_DATAURL_PREFIX) :]
    return io.BytesIO(base64.b64decode(raw))


def _build_styles() -> dict[str, ParagraphStyle]:
    """ReportLab paragraph styles used across the doc.

    We start from ``getSampleStyleSheet`` then layer on our own tweaks so
    we keep the library's sensible defaults (font registration, leading
    calc, …) without re-implementing them.
    """
    base = getSampleStyleSheet()
    styles: dict[str, ParagraphStyle] = {}

    styles["title"] = ParagraphStyle(
        "ib_title",
        parent=base["Title"],
        fontSize=28,
        leading=34,
        alignment=TA_CENTER,
        spaceAfter=12,
    )
    styles["subtitle"] = ParagraphStyle(
        "ib_subtitle",
        parent=base["Normal"],
        fontSize=14,
        leading=18,
        alignment=TA_CENTER,
        textColor=colors.HexColor("#6c757d"),
    )
    styles["cover_footer"] = ParagraphStyle(
        "ib_cover_footer",
        parent=base["Normal"],
        fontSize=10,
        leading=14,
        alignment=TA_CENTER,
        textColor=colors.HexColor("#adb5bd"),
    )
    styles["step_heading"] = ParagraphStyle(
        "ib_step_heading",
        parent=base["Heading2"],
        fontSize=16,
        leading=20,
        spaceAfter=6,
    )
    styles["step_heading_sm"] = ParagraphStyle(
        "ib_step_heading_sm",
        parent=base["Heading3"],
        fontSize=12,
        leading=15,
        spaceAfter=4,
    )
    styles["step_body"] = ParagraphStyle(
        "ib_step_body",
        parent=base["Normal"],
        fontSize=10,
        leading=13,
    )
    styles["step_body_sm"] = ParagraphStyle(
        "ib_step_body_sm",
        parent=base["Normal"],
        fontSize=9,
        leading=11,
    )
    return styles


def _escape(text: str | None) -> str:
    """Minimal escape for ReportLab Paragraph content (XML-ish)."""
    if text is None:
        return ""
    return (
        str(text)
        .replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
    )


def _step_image(step: InstructionExportStep, *, max_w: float, max_h: float) -> Image:
    """Build a ReportLab ``Image`` flowable scaled to fit the cell.

    ReportLab's ``Image`` accepts an explicit width / height — we
    compute the aspect-preserving fit ourselves rather than rely on the
    library's autosize because the source PNG dimensions aren't known
    until ReportLab opens the bytes, and we want a deterministic layout.
    """
    buf = _decode_image(step.image_data_url)
    img = Image(buf, width=max_w, height=max_h, kind="proportional")
    # KeepInFrame around the Image would change wrapping; the proportional
    # kind handles the aspect-preserve case so we just return it.
    return img


def _heading_and_body(
    step: InstructionExportStep,
    styles: dict[str, ParagraphStyle],
    *,
    small: bool = False,
) -> list:
    """Title (with step number prefix) + optional description paragraph."""
    heading_style = styles["step_heading_sm"] if small else styles["step_heading"]
    body_style = styles["step_body_sm"] if small else styles["step_body"]
    label = step.title.strip() if step.title and step.title.strip() else "Untitled step"
    heading = Paragraph(
        f"<b>Step {step.step_number}:</b> {_escape(label)}",
        heading_style,
    )
    out = [heading]
    if step.description and step.description.strip():
        out.append(Paragraph(_escape(step.description), body_style))
    return out


def _cover_flowables(
    request: InstructionExportRequest, styles: dict[str, ParagraphStyle]
) -> list:
    """Title page contents.

    UTC date stamp matches what the rest of the API uses for timestamps,
    so re-exports across the date line don't drift between viewer
    timezones.
    """
    title = request.project_title.strip() if request.project_title and request.project_title.strip() else "Instructions"
    n = len(request.steps)
    today = datetime.now(timezone.utc).strftime("%Y-%m-%d")
    return [
        Spacer(1, _CONTENT_H * 0.25),
        Paragraph(_escape(title), styles["title"]),
        Spacer(1, 8),
        Paragraph(f"{n} step{'s' if n != 1 else ''} &middot; {today}", styles["subtitle"]),
        Spacer(1, _CONTENT_H * 0.35),
        Paragraph(
            "Built with kevsrobots.com Instruction Builder",
            styles["cover_footer"],
        ),
        PageBreak(),
    ]


# --- Layout flowables: one block per step page slot ----------------------


def _one_per_page(step: InstructionExportStep, styles: dict[str, ParagraphStyle]) -> list:
    """One large step per page. Image fills most of the width."""
    out: list = []
    out.extend(_heading_and_body(step, styles, small=False))
    out.append(Spacer(1, 6))
    # Image: cap to 170mm wide and the remaining content height after the
    # heading + description (estimate generously; KeepInFrame will clip
    # if the body description is huge).
    img = _step_image(step, max_w=170 * mm, max_h=_CONTENT_H - 50 * mm)
    out.append(img)
    out.append(PageBreak())
    return out


def _two_per_page(steps: list[InstructionExportStep], styles: dict[str, ParagraphStyle]) -> list:
    """Two stacked half-page cells per A4 page.

    Each cell gets ~half the available height. We wrap each cell in
    ``KeepInFrame`` so a verbose description doesn't push the image to
    the next page mid-cell — instead it gets clipped (Phase 2c could
    refine this if it turns out to matter in practice).
    """
    out: list = []
    cell_h = (_CONTENT_H - 6 * mm) / 2  # 6mm gap between cells
    for i in range(0, len(steps), 2):
        chunk = steps[i : i + 2]
        for idx, step in enumerate(chunk):
            cell_content: list = []
            cell_content.extend(_heading_and_body(step, styles, small=False))
            cell_content.append(Spacer(1, 4))
            cell_content.append(
                _step_image(step, max_w=_CONTENT_W, max_h=cell_h - 30 * mm),
            )
            out.append(KeepInFrame(_CONTENT_W, cell_h, cell_content, mode="truncate"))
            if idx == 0 and len(chunk) > 1:
                out.append(Spacer(1, 6 * mm))
        out.append(PageBreak())
    return out


def _four_per_page(steps: list[InstructionExportStep], styles: dict[str, ParagraphStyle]) -> list:
    """2x2 grid per page. Tighter typography and smaller images.

    Implemented as a sequence of KeepInFrame quadrants flowed into the
    document; the BaseDocTemplate uses four equal Frames per page so
    each KeepInFrame lands in its own quadrant.
    """
    out: list = []
    # Quadrant size matches the Frame layout below. Leave a small inner
    # padding so the image doesn't kiss the frame edge.
    cell_w = (_CONTENT_W - 6 * mm) / 2
    cell_h = (_CONTENT_H - 6 * mm) / 2
    for i in range(0, len(steps), 4):
        chunk = steps[i : i + 4]
        for step in chunk:
            cell_content: list = []
            cell_content.extend(_heading_and_body(step, styles, small=True))
            cell_content.append(Spacer(1, 2))
            cell_content.append(
                _step_image(step, max_w=cell_w - 4 * mm, max_h=cell_h - 22 * mm),
            )
            out.append(KeepInFrame(cell_w, cell_h, cell_content, mode="truncate"))
        # If the final page has fewer than 4 steps the trailing Frames
        # stay empty — ReportLab is happy with that.
        out.append(PageBreak())
    return out


# --- Page templates -------------------------------------------------------


def _make_doc(buffer: io.BytesIO, steps_per_page: int) -> BaseDocTemplate:
    """Build a doc with two page templates: ``Cover`` and ``Content``.

    ``Cover`` is a single full-width frame. ``Content`` differs by
    ``steps_per_page``: a single frame (1), two stacked frames (2), or a
    2x2 grid of frames (4). The 1- and 2-step cases actually reuse a
    single full-width frame and rely on KeepInFrame to subdivide; only
    the 4-up case wants discrete frames because Platypus's flow
    algorithm needs a hint that the cells are independent.
    """
    doc = BaseDocTemplate(
        buffer,
        pagesize=A4,
        leftMargin=_MARGIN,
        rightMargin=_MARGIN,
        topMargin=_MARGIN,
        bottomMargin=_MARGIN,
        title="Instructions",
        author="kevsrobots.com",
    )

    cover_frame = Frame(
        _MARGIN, _MARGIN, _CONTENT_W, _CONTENT_H, id="cover", showBoundary=0
    )
    cover_template = PageTemplate(id="Cover", frames=[cover_frame])

    if steps_per_page == 4:
        cell_w = (_CONTENT_W - 6 * mm) / 2
        cell_h = (_CONTENT_H - 6 * mm) / 2
        # Origin at bottom-left in ReportLab.
        x_left = _MARGIN
        x_right = _MARGIN + cell_w + 6 * mm
        y_bottom = _MARGIN
        y_top = _MARGIN + cell_h + 6 * mm
        content_frames = [
            Frame(x_left, y_top, cell_w, cell_h, id="tl"),
            Frame(x_right, y_top, cell_w, cell_h, id="tr"),
            Frame(x_left, y_bottom, cell_w, cell_h, id="bl"),
            Frame(x_right, y_bottom, cell_w, cell_h, id="br"),
        ]
    else:
        # Single full-width frame; 1- and 2-up rely on KeepInFrame inside.
        content_frames = [
            Frame(_MARGIN, _MARGIN, _CONTENT_W, _CONTENT_H, id="content"),
        ]

    content_template = PageTemplate(
        id="Content", frames=content_frames, onPage=_draw_page_number
    )
    doc.addPageTemplates([cover_template, content_template])
    return doc


def _draw_page_number(canvas, doc) -> None:
    """Footer page number on content pages only.

    The cover template doesn't include this callback so the title page
    stays clean.
    """
    canvas.saveState()
    canvas.setFont("Helvetica", 8)
    canvas.setFillColor(colors.HexColor("#adb5bd"))
    # Page count isn't known to a single page callback in ReportLab
    # without a multi-pass build, so we just show "{i}" — adding the
    # total would require switching to a Total-pages approach (e.g.
    # _doPageNumber from the cookbook). Simple is fine for now.
    canvas.drawCentredString(_PAGE_W / 2, _MARGIN / 2, f"{doc.page}")
    canvas.restoreState()


# --- Entry point ----------------------------------------------------------


def render_instructions_pdf(
    request: InstructionExportRequest,
    project_id: int,  # noqa: ARG001 — accepted for parity / future filename use
) -> bytes:
    """Return the assembled PDF as bytes.

    Caller is responsible for validating ``request.steps_per_page`` is
    one of {1, 2, 4} and that ``request.steps`` is non-empty; this
    helper trusts those constraints (and will raise on bad input
    anyway).
    """
    if request.steps_per_page not in (1, 2, 4):
        raise ValueError("steps_per_page must be 1, 2, or 4")
    if not request.steps:
        raise ValueError("steps must be non-empty")

    styles = _build_styles()
    buffer = io.BytesIO()
    doc = _make_doc(buffer, request.steps_per_page)

    flowables: list = []
    if request.include_title_page:
        flowables.append(_NextPageTemplate("Cover"))
        flowables.extend(_cover_flowables(request, styles))

    flowables.append(_NextPageTemplate("Content"))

    if request.steps_per_page == 1:
        for step in request.steps:
            flowables.extend(_one_per_page(step, styles))
    elif request.steps_per_page == 2:
        flowables.extend(_two_per_page(request.steps, styles))
    else:  # 4
        flowables.extend(_four_per_page(request.steps, styles))

    doc.build(flowables)
    return buffer.getvalue()


# ReportLab's NextPageTemplate is in platypus.flowables; we import lazily
# at module bottom to keep the top-of-file imports readable.
from reportlab.platypus.doctemplate import NextPageTemplate as _NextPageTemplate  # noqa: E402
