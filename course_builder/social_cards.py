"""Social share cards (Open Graph / Twitter images) for every course.

For each course in web/_data/courses.yml this writes two 1200x630 JPEGs to
web/assets/img/learn/cards/:

    <slug>.jpg            the course card, used as og:image on every lesson
    <slug>-completed.jpg  the "I completed this course" card, used as og:image
                          on /learn/<slug>/completed.html, the page learners
                          share when they finish a course

<slug> is the course folder under /learn/ (e.g. micropython).

build.py runs this after it writes courses.yml, so a new or renamed course
gets its cards with the normal course build. It can also be run on its own:

    python3 generate_course_cards.py

The fonts are vendored in course_builder/fonts (Inter, SIL OFL 1.1) so the
cards look the same on a Mac and on the build Pi. Output is deterministic,
so re-running on unchanged courses leaves git clean.
"""
from __future__ import annotations

import os
import re

import yaml
from PIL import Image, ImageDraw, ImageFilter, ImageFont, ImageOps

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEB = os.path.join(ROOT, "web")
COURSES_YML = os.path.join(WEB, "_data", "courses.yml")
OUT = os.path.join(WEB, "assets", "img", "learn", "cards")
FONTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fonts")
LOGO = os.path.join(WEB, "assets", "img", "logo_red.png")

W, H = 1200, 630
PAD = 64

INK = (20, 23, 31)          # card background
INK_2 = (32, 36, 48)        # panels
WHITE = (255, 255, 255)
MUTED = (178, 186, 204)
KEVRED = (194, 32, 50)      # $kevred
OCRE = (255, 202, 41)       # $ocre
GREEN = (46, 160, 90)


def _font(name: str, size: int) -> ImageFont.FreeTypeFont:
    return ImageFont.truetype(os.path.join(FONTS, name), size)


def course_slug(course: dict) -> str | None:
    """The course folder, from its link: /learn/<slug>/00_intro.html."""
    m = re.match(r"^/learn/([^/]+)/", course.get("link") or "")
    return m.group(1) if m else None


def _text_w(draw, text, font):
    box = draw.textbbox((0, 0), text, font=font)
    return box[2] - box[0]


def _wrap(draw, text, font, max_w):
    lines, cur = [], ""
    for word in text.split():
        trial = (cur + " " + word).strip()
        if _text_w(draw, trial, font) <= max_w or not cur:
            cur = trial
        else:
            lines.append(cur)
            cur = word
    if cur:
        lines.append(cur)
    return lines


def _fit_title(draw, title, max_w, max_lines):
    """Largest title size (72 down to 40) that fits in max_lines lines."""
    for size in range(72, 38, -4):
        font = _font("InterDisplay-Bold.otf", size)
        lines = _wrap(draw, title, font, max_w)
        if len(lines) <= max_lines:
            return font, lines
    font = _font("InterDisplay-Bold.otf", 40)
    lines = _wrap(draw, title, font, max_w)[:max_lines]
    lines[-1] = lines[-1].rstrip(" -:,") + "…"
    return font, lines


def _cover(course: dict, size: tuple[int, int]) -> Image.Image | None:
    cover = course.get("cover") or ""
    path = os.path.join(WEB, cover.lstrip("/"))
    if not cover or not os.path.isfile(path):
        return None
    try:
        img = Image.open(path).convert("RGB")
    except OSError:
        return None
    return ImageOps.fit(img, size, Image.LANCZOS, centering=(0.5, 0.5))


def _rounded(img: Image.Image, radius: int) -> Image.Image:
    mask = Image.new("L", img.size, 0)
    ImageDraw.Draw(mask).rounded_rectangle([0, 0, img.size[0], img.size[1]], radius, fill=255)
    out = Image.new("RGBA", img.size)
    out.paste(img, (0, 0), mask)
    return out


def _pill(draw, x, y, text, font, fill, fg, icon=None):
    """Rounded label; returns its right edge."""
    pad_x, h = 20, 46
    w = _text_w(draw, text, font) + pad_x * 2 + (34 if icon else 0)
    draw.rounded_rectangle([x, y, x + w, y + h], radius=h // 2, fill=fill)
    tx = x + pad_x
    if icon == "tick":
        cx, cy = tx + 12, y + h // 2
        draw.line([(cx - 9, cy), (cx - 2, cy + 8), (cx + 11, cy - 9)], fill=fg, width=5, joint="curve")
        tx += 34
    draw.text((tx, y + h // 2), text, font=font, fill=fg, anchor="lm")
    return x + w


def _meta_line(course: dict) -> str:
    parts = []
    if course.get("lessons"):
        parts.append(f"{course['lessons']} lessons")
    if course.get("duration"):
        parts.append(str(course["duration"]))
    if course.get("level"):
        parts.append(str(course["level"]).capitalize())
    return "  ·  ".join(parts)


def render(course: dict, completed: bool = False) -> Image.Image:
    img = Image.new("RGB", (W, H), INK)
    d = ImageDraw.Draw(img)

    # Right: the course cover, 4:3, with a soft shadow. Covers are 4:3 already.
    cov_w, cov_h = 480, 360
    cov_x, cov_y = W - PAD - cov_w, (H - cov_h) // 2 - 10
    cover = _cover(course, (cov_w, cov_h))
    shadow = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    ImageDraw.Draw(shadow).rounded_rectangle(
        [cov_x + 8, cov_y + 14, cov_x + cov_w + 8, cov_y + cov_h + 14], 22, fill=(0, 0, 0, 150))
    img.paste(shadow.filter(ImageFilter.GaussianBlur(14)), (0, 0), shadow.filter(ImageFilter.GaussianBlur(14)))
    if cover is not None:
        img.paste(_rounded(cover, 22), (cov_x, cov_y), _rounded(cover, 22))
    else:
        d.rounded_rectangle([cov_x, cov_y, cov_x + cov_w, cov_y + cov_h], 22, fill=INK_2)

    if completed:
        # A ribbon across the cover's top-right corner
        badge_r = 74
        bx, by = cov_x + cov_w - 18, cov_y + 18
        d.ellipse([bx - badge_r, by - badge_r, bx + badge_r, by + badge_r], fill=OCRE, outline=INK, width=6)
        d.line([(bx - 34, by + 2), (bx - 10, by + 28), (bx + 36, by - 26)], fill=INK, width=14, joint="curve")

    # Top band in brand red, full width
    d.rectangle([0, 0, W, 10], fill=KEVRED)

    # Left column
    left_w = cov_x - PAD - 48
    y = PAD + 6
    f_pill = _font("Inter-SemiBold.otf", 24)
    if completed:
        _pill(d, PAD, y, "COURSE COMPLETE", f_pill, GREEN, WHITE, icon="tick")
    else:
        _pill(d, PAD, y, "FREE COURSE", f_pill, KEVRED, WHITE)
    y += 46 + 34

    if completed:
        f_kicker = _font("Inter-SemiBold.otf", 30)
        d.text((PAD, y), "I completed", font=f_kicker, fill=OCRE)
        y += 48

    font, lines = _fit_title(d, course.get("name", ""), left_w, 3 if completed else 4)
    line_h = int(font.size * 1.12)
    for line in lines:
        d.text((PAD, y), line, font=font, fill=WHITE)
        y += line_h

    meta = _meta_line(course)
    if meta:
        y += 18
        d.text((PAD, y), meta, font=_font("Inter-Regular.otf", 28), fill=MUTED)

    # Bottom: logo + site
    logo = Image.open(LOGO).convert("RGBA").resize((64, 64), Image.LANCZOS)
    ly = H - PAD - 64 + 6
    img.paste(logo, (PAD, ly), logo)
    f_brand = _font("Inter-SemiBold.otf", 30)
    f_url = _font("Inter-Regular.otf", 22)
    d.text((PAD + 82, ly + 6), "KevsRobots", font=f_brand, fill=WHITE)
    d.text((PAD + 82, ly + 42), "kevsrobots.com/learn", font=f_url, fill=MUTED)

    if course.get("video"):
        # A small "has a video" hint under the cover: the landing page leads with it
        f_v = _font("Inter-SemiBold.otf", 22)
        vx, vy = cov_x, cov_y + cov_h + 28
        d.rounded_rectangle([vx, vy, vx + 40, vy + 28], 8, fill=KEVRED)
        d.polygon([(vx + 15, vy + 7), (vx + 15, vy + 21), (vx + 27, vy + 14)], fill=WHITE)
        d.text((vx + 52, vy + 14), "Video course on YouTube", font=f_v, fill=MUTED, anchor="lm")

    return img


def generate(courses_yml: str = COURSES_YML, out_dir: str = OUT, verbose: bool = True) -> int:
    with open(courses_yml) as f:
        courses = yaml.safe_load(f) or []
    os.makedirs(out_dir, exist_ok=True)

    written = 0
    for course in courses:
        slug = course_slug(course)
        if not slug:
            continue
        for completed in (False, True):
            name = f"{slug}-completed.jpg" if completed else f"{slug}.jpg"
            render(course, completed).save(
                os.path.join(out_dir, name), "JPEG", quality=84, optimize=True, progressive=True)
            written += 1
    if verbose:
        print(f"Wrote {written} social cards for {written // 2} courses to {os.path.relpath(out_dir, ROOT)}")
    return written
