"""Animated GIF / H.264 MP4 rendering for instruction exports.

Issue #178, Phase 3b/3c: takes the same array of PNG dataURLs the PDF
export already accepts and assembles them into a video format via
FFmpeg. The branded "Made with kevsrobots.com/projects" slide is
appended as the closing frame, held for ~3s so it lingers on the
viewer's screen at the end.

**Why subprocess + FFmpeg rather than pyav / imageio?**

FFmpeg is one apt-install (already shipped in the projects-api
Dockerfile alongside ``fonts-dejavu-core``), the binary is rock-solid,
and the CLI is well-documented for both palettegen GIFs and yuv420p
H.264. The Python wrappers add a dependency and a bug surface for
something that's already trivially scripted as ``subprocess.run`` with
list-form arguments. List-form is mandatory — no ``shell=True`` — so
even though we don't accept arbitrary paths from clients, there is
zero shell-injection or path-traversal surface.

Heavy lifting (subprocess.run + decode loop) is synchronous; the
FastAPI route wraps the call in ``asyncio.to_thread`` so the event
loop stays responsive while FFmpeg runs.
"""

from __future__ import annotations

import base64
import io
import logging
import subprocess
import tempfile
from pathlib import Path

from PIL import Image

from .instructions_branding import render_made_with_slide
from .schemas import InstructionExportRequest

logger = logging.getLogger(__name__)

# Cap each FFmpeg subprocess at 120s wall-clock. A 50-step build with
# the default 2s/frame settles at ~100s of video; encode time is a
# small multiple of that on a Pi 5. The cap is well above the realistic
# worst case and well below the load-balancer's 60s proxy timeout × 2.
_FFMPEG_TIMEOUT_S = 120

_PNG_DATAURL_PREFIX = "data:image/png;base64,"

# Target canvas the video frames render at. The browser's Fabric.js
# canvas is 1200x900 and ``renderAllStepsToDataUrls`` uses multiplier:2,
# so step PNGs arrive at 2400x1800. We resize them down (or up) to a
# consistent 1200x900 for the encoder — varying frame sizes crash
# libavfilter's palettegen/paletteuse pair and libx264 alike. The slide
# is rendered at the same dimensions to match.
_VIDEO_W = 1200
_VIDEO_H = 900


def _decode_data_url(data_url: str) -> bytes:
    """Strip the dataURL prefix and base64-decode the payload."""
    if not data_url.startswith(_PNG_DATAURL_PREFIX):
        raise ValueError("image_data_url must start with 'data:image/png;base64,'")
    raw = data_url[len(_PNG_DATAURL_PREFIX):]
    return base64.b64decode(raw)


def _normalise_frame(png_bytes: bytes) -> bytes:
    """Resize / letterbox the supplied PNG to ``(_VIDEO_W, _VIDEO_H)``.

    Aspect-preserving fit on a white canvas — same colour the browser's
    Fabric canvas uses as its background — so a step screenshot from
    an unusual aspect ratio doesn't get stretched. The output is always
    exact-sized RGB so libx264's yuv420p / paletteuse don't trip on a
    mid-stream parameter change.
    """
    try:
        src = Image.open(io.BytesIO(png_bytes))
        src.load()
    except Exception as exc:  # noqa: BLE001 — Pillow's exceptions are varied
        raise ValueError("frame is not a valid PNG") from exc

    # Convert to RGB on a white canvas so any RGBA inputs flatten
    # cleanly. ``thumbnail`` would mutate in-place and not pad to the
    # target dimensions; we want exact dimensions every time.
    canvas = Image.new("RGB", (_VIDEO_W, _VIDEO_H), (255, 255, 255))
    src_w, src_h = src.size
    if src_w == 0 or src_h == 0:
        raise ValueError("frame has zero width or height")

    scale = min(_VIDEO_W / src_w, _VIDEO_H / src_h)
    new_w = max(1, int(src_w * scale))
    new_h = max(1, int(src_h * scale))
    resized = src.convert("RGBA").resize((new_w, new_h), Image.Resampling.LANCZOS)

    x = (_VIDEO_W - new_w) // 2
    y = (_VIDEO_H - new_h) // 2
    # Paste with the alpha channel as the mask so any transparency
    # blends onto white rather than appearing as black.
    canvas.paste(resized, (x, y), resized)
    buf = io.BytesIO()
    canvas.save(buf, format="PNG")
    return buf.getvalue()


def _write_frames(
    request: InstructionExportRequest,
    workdir: Path,
    final_slide_holds: int,
) -> int:
    """Write step PNGs + the duplicated Made-with slide to ``workdir``.

    Each frame is normalised to ``_VIDEO_W x _VIDEO_H`` so FFmpeg can
    treat the directory as a uniform image sequence — varying frame
    sizes crash both palettegen/paletteuse (GIF) and libx264 (MP4).

    Returns the total number of frames written. The frames are named
    ``frame-001.png``, ``frame-002.png``, … so FFmpeg's ``-i frame-%03d.png``
    image-sequence demuxer picks them up in order.
    """
    n = 0
    for step in request.steps:
        n += 1
        png_bytes = _normalise_frame(_decode_data_url(step.image_data_url))
        (workdir / f"frame-{n:03d}.png").write_bytes(png_bytes)

    # Made-with slide — generate once at the same canvas size as the
    # normalised step frames, duplicate for the hold.
    slide_bytes = render_made_with_slide(
        width=_VIDEO_W, height=_VIDEO_H, project_title=request.project_title
    )
    for _ in range(max(1, final_slide_holds)):
        n += 1
        (workdir / f"frame-{n:03d}.png").write_bytes(slide_bytes)
    return n


def _run_ffmpeg(args: list[str], stage: str) -> None:
    """Invoke FFmpeg with list-form arguments + timeout + error logging.

    The caller surfaces failures as HTTP 500s without leaking the
    FFmpeg stderr to the client — we log it here for ops debugging.
    """
    try:
        subprocess.run(
            args,
            check=True,
            capture_output=True,
            timeout=_FFMPEG_TIMEOUT_S,
        )
    except subprocess.TimeoutExpired as exc:
        logger.error("ffmpeg %s timed out after %ss: %s", stage, _FFMPEG_TIMEOUT_S, exc)
        raise RuntimeError(f"ffmpeg {stage} timed out") from exc
    except subprocess.CalledProcessError as exc:
        stderr = exc.stderr.decode("utf-8", errors="replace") if exc.stderr else ""
        logger.error("ffmpeg %s failed (rc=%s): %s", stage, exc.returncode, stderr)
        raise RuntimeError(f"ffmpeg {stage} failed") from exc
    except FileNotFoundError as exc:
        # FFmpeg not on PATH — production has it via the Dockerfile;
        # local dev / CI without it gets a clear error rather than a
        # cryptic missing-binary trace.
        logger.error("ffmpeg binary not found on PATH")
        raise RuntimeError("ffmpeg is not installed") from exc


def render_instructions_video(
    request: InstructionExportRequest,
    project_id: int,  # noqa: ARG001 — accepted for parity / future filename use
    output_format: str,
    frame_seconds: float | None = None,
    final_slide_seconds: float | None = None,
) -> bytes:
    """Return GIF or MP4 binary for the supplied instruction steps.

    ``output_format`` must be ``'gif'`` or ``'mp4'``. The branded
    final-slide is appended unconditionally — Phase 3 doesn't expose
    an "include_branding" toggle.

    The ``frame_seconds`` / ``final_slide_seconds`` arguments override
    the values on ``request`` if supplied; otherwise the request values
    (themselves defaulting to 2.0 / 3.0) drive timing.
    """
    if output_format not in ("gif", "mp4"):
        raise ValueError(f"output_format must be 'gif' or 'mp4', got {output_format!r}")
    if not request.steps:
        raise ValueError("steps must be non-empty")

    fs = frame_seconds if frame_seconds is not None else request.frame_seconds
    ss = final_slide_seconds if final_slide_seconds is not None else request.final_slide_seconds
    if fs <= 0:
        raise ValueError("frame_seconds must be positive")
    if ss <= 0:
        raise ValueError("final_slide_seconds must be positive")

    # Number of times to duplicate the closing slide so the hold is
    # ``ss`` seconds at the step framerate of ``1/fs`` fps. Always at
    # least 1 — even with a tiny ss/fs ratio the slide should appear
    # on screen for at least one frame slot.
    holds = max(1, round(ss / fs))

    with tempfile.TemporaryDirectory(prefix="ib-export-") as tmp:
        workdir = Path(tmp)
        total_frames = _write_frames(request, workdir, holds)
        assert total_frames >= 2  # at least one step + the slide

        framerate = f"1/{fs}" if fs >= 1 else f"{1.0 / fs:.6f}"

        if output_format == "gif":
            palette = workdir / "palette.png"
            # Two-pass: palettegen then paletteuse. The default GIF
            # global palette is 256 colours; this pair gives noticeably
            # better quality than a single-pass encode on the kinds of
            # screenshots Fabric.js produces (flat-colour backgrounds +
            # antialiased text). ``-update 1`` + ``-frames:v 1`` tell
            # FFmpeg 6+/8 that ``palette.png`` is a single image rather
            # than a misnamed image sequence — otherwise the encoder
            # warns "filename doesn't contain a pattern" on FFmpeg 8.
            _run_ffmpeg(
                [
                    "ffmpeg", "-y",
                    "-framerate", framerate,
                    "-i", str(workdir / "frame-%03d.png"),
                    "-vf", "palettegen",
                    "-update", "1",
                    "-frames:v", "1",
                    str(palette),
                ],
                stage="palettegen",
            )
            out_path = workdir / "out.gif"
            _run_ffmpeg(
                [
                    "ffmpeg", "-y",
                    "-framerate", framerate,
                    "-i", str(workdir / "frame-%03d.png"),
                    "-i", str(palette),
                    "-lavfi", "paletteuse",
                    "-loop", "0",
                    str(out_path),
                ],
                stage="gif-encode",
            )
            return out_path.read_bytes()

        # MP4 — single-pass H.264. ``yuv420p`` is required for broad
        # player compatibility (QuickTime, mobile browsers); ``faststart``
        # moves the moov atom to the front so the file plays without a
        # full download. ``-vf scale`` pads odd-pixel inputs to an
        # even-pixel grid, which libx264 requires under yuv420p.
        out_path = workdir / "out.mp4"
        _run_ffmpeg(
            [
                "ffmpeg", "-y",
                "-framerate", framerate,
                "-i", str(workdir / "frame-%03d.png"),
                "-c:v", "libx264",
                "-pix_fmt", "yuv420p",
                "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2",
                "-movflags", "+faststart",
                str(out_path),
            ],
            stage="mp4-encode",
        )
        return out_path.read_bytes()
