#!/usr/bin/env python3
"""Generate subtle geometric header banners for course lessons.

Each lesson in a course gets its own banner image, so pages feel distinct
rather than repeating one cover. Banners share a palette derived from the
course name, and vary their pattern and hue per lesson, so a course reads
as one visual family.

Usage:
    python3 generate_banners.py <course-slug>            # generate + wire up
    python3 generate_banners.py <course-slug> --force    # overwrite existing
    python3 generate_banners.py <course-slug> --images-only
    python3 generate_banners.py --contact-sheet out.jpg  # preview all styles
"""

import argparse
import colorsys
import hashlib
import math
import random
import re
import sys
from pathlib import Path

import yaml
from PIL import Image, ImageDraw

# The .cover class renders at 200px tall and the full width of the lesson
# column (roughly 1050-1450px), so the visible box is 5:1 to 7:1. We author
# at 4:1 and let object-fit crop a little off the top and bottom. Because the
# patterns are abstract and vertically uniform, that crop is invisible.
WIDTH, HEIGHT = 1600, 400
SUPERSAMPLE = 2          # render big, downscale - PIL does not antialias polygons

REPO = Path(__file__).resolve().parent
SOURCE = REPO / "source"

BANNER_DIR_NAME = "banners"
COVER_LINE = '![Course Cover Image]({{page.cover}}){:class="cover"}'
# Matches whatever cover line a lesson currently has - the Liquid form or a
# hard-coded assets/ path - so we can normalise it rather than duplicate it.
# Note [ \t]* not \s*: \s would swallow the blank line that follows.
COVER_LINE_RE = re.compile(
    r'^!\[[^\]]*\]\((?:\{\{\s*page\.cover\s*\}\}|assets/[^)]+)\)\{:class="cover"\}[ \t]*$',
    re.MULTILINE,
)


# --------------------------------------------------------------------------
# Colour
# --------------------------------------------------------------------------

def seeded_random(*parts: str) -> random.Random:
    """A Random seeded from strings, so output is stable across runs."""
    digest = hashlib.sha256("::".join(parts).encode("utf-8")).hexdigest()
    return random.Random(int(digest[:16], 16))


def hsv(h: float, s: float, v: float, a: int = 255) -> tuple:
    r, g, b = colorsys.hsv_to_rgb(h % 1.0, s, v)
    return (int(r * 255), int(g * 255), int(b * 255), a)


WHITE = (255, 255, 255, 255)
BLACK = (0, 0, 0, 255)


def vivid_value(hue: float, value: float) -> float:
    """Lift the value of hues that need it to look clean.

    HSV treats every hue as equally bright at the same V, but our eyes do
    not. Yellow at V=0.75 is olive - a dirty colour. Blue at V=0.75 is a
    perfectly rich blue. So lift V toward the maximum as the hue approaches
    yellow, and leave the blues and reds alone.
    """
    lift = max(0.0, math.cos((hue - 0.15) * math.tau))   # 1 at yellow, 0 at blue
    return min(0.99, value + 0.26 * lift)


def mix(c1: tuple, c2: tuple, t: float) -> tuple:
    """Blend two colours, keeping c1's alpha."""
    return tuple(int(c1[i] * (1 - t) + c2[i] * t) for i in range(3)) + (c1[3],)


# How far a lesson may drift from its course's base hue, as a fraction of the
# colour wheel. 0.10 is about 36 degrees each way. Wider than this and a
# course wanders across a perceptual boundary - green to brick red reads as
# two different courses, even though it is a small step on the wheel.
HUE_SPREAD = 0.10


def build_palette(rng: random.Random, course_hue: float, hue_offset: float,
                  allow_ink: bool = True) -> dict:
    """A confident background plus tone-on-tone shape colours.

    The house style on kevsrobots is bold, saturated colour - the BurgerBot
    red, the DuckDB yellow, the MeshCore navy - with texture that sits very
    close to the background tone. That is what makes a banner read as
    "subtle" while still being genuinely colourful: the *colour* is loud, the
    *geometry* is quiet. Desaturated mid-tones just look muddy, so we never
    generate them.
    """
    base_hue = (course_hue + hue_offset) % 1.0
    family = "ink" if (allow_ink and rng.random() < 0.30) else "vivid"

    if family == "ink":
        # Deep near-black colour; texture slightly lighter than the ground
        bg = hsv(base_hue, rng.uniform(0.42, 0.62), rng.uniform(0.14, 0.21))
        tones = [mix(bg, WHITE, f) for f in (0.14, 0.24, 0.36)]
        pop = hsv(base_hue + rng.choice([0.40, -0.40]), 0.55, 0.95)
    else:
        # Saturated bright ground; texture a touch darker and a touch lighter
        bg = hsv(base_hue, rng.uniform(0.68, 0.88),
                 vivid_value(base_hue, rng.uniform(0.70, 0.90)))
        tones = [mix(bg, BLACK, 0.14), mix(bg, WHITE, 0.16), mix(bg, BLACK, 0.24)]
        pop = hsv(base_hue + rng.choice([0.10, -0.10]), 0.55, 0.98)

    # Weighted so the contrasting pop turns up in roughly one shape in six
    accents = [tones[0], tones[0], tones[1], tones[1], tones[2], pop]

    return {"bg": bg, "tones": tones, "pop": pop,
            "accents": accents, "family": family}


def gradient_background(draw: ImageDraw.ImageDraw, size: tuple, palette: dict,
                        rng: random.Random) -> None:
    """A soft vertical shift, so the ground is never a flat slab."""
    w, h = size
    top = palette["bg"]
    bottom = mix(top, BLACK if palette["family"] == "vivid" else WHITE, 0.16)
    for y in range(h):
        t = y / h
        draw.line([(0, y), (w, y)],
                  fill=tuple(int(top[i] * (1 - t) + bottom[i] * t)
                             for i in range(3)) + (255,))


# --------------------------------------------------------------------------
# Patterns - each draws onto an RGBA overlay at supersampled size
# --------------------------------------------------------------------------

def pattern_triangles(d, w, h, palette, rng):
    """Low-poly mesh of soft triangles."""
    cols, rows = rng.randint(10, 16), rng.randint(3, 5)
    cw, ch = w / cols, h / rows
    points = [[(c * cw + rng.uniform(-cw * .3, cw * .3),
                r * ch + rng.uniform(-ch * .3, ch * .3))
               for c in range(cols + 1)] for r in range(rows + 1)]
    for r in range(rows):
        for c in range(cols):
            a, b = points[r][c], points[r][c + 1]
            e, f = points[r + 1][c], points[r + 1][c + 1]
            for tri in ((a, b, e), (b, f, e)):
                colour = rng.choice(palette["accents"])
                d.polygon(tri, fill=colour[:3] + (rng.randint(18, 62),))


def pattern_arcs(d, w, h, palette, rng):
    """Concentric rings radiating from just off one edge."""
    cx = rng.choice([w * 0.08, w * 0.92])
    cy = h * rng.uniform(0.3, 0.7)
    step = rng.randint(44, 70)
    width = rng.randint(6, 12)
    for i in range(1, 34):
        r = i * step
        colour = palette["accents"][i % len(palette["accents"])]
        alpha = max(14, 92 - i * 3)
        d.ellipse([cx - r, cy - r, cx + r, cy + r],
                  outline=colour[:3] + (alpha,), width=width)


def pattern_hexes(d, w, h, palette, rng):
    """Honeycomb tiling, a few cells filled."""
    size = rng.randint(38, 58)
    dx, dy = size * 1.5, size * math.sqrt(3)
    for row in range(-1, int(h / dy) + 2):
        for col in range(-1, int(w / dx) + 2):
            cx = col * dx
            cy = row * dy + (dy / 2 if col % 2 else 0)
            pts = [(cx + size * math.cos(math.pi / 3 * k),
                    cy + size * math.sin(math.pi / 3 * k)) for k in range(6)]
            colour = rng.choice(palette["accents"])
            if rng.random() < 0.30:
                d.polygon(pts, fill=colour[:3] + (rng.randint(22, 58),))
            else:
                d.polygon(pts, outline=colour[:3] + (rng.randint(18, 46),), width=3)


def pattern_stripes(d, w, h, palette, rng):
    """Diagonal bands of varying width."""
    angle = rng.choice([28, 34, 40, -28, -34])
    slope = math.tan(math.radians(angle))
    x = -h * abs(slope) - 200
    while x < w + 200:
        band = rng.randint(20, 90)
        colour = rng.choice(palette["accents"])
        d.polygon([(x, 0), (x + band, 0),
                   (x + band + h * slope, h), (x + h * slope, h)],
                  fill=colour[:3] + (rng.randint(16, 54),))
        x += band + rng.randint(18, 70)


def pattern_dots(d, w, h, palette, rng):
    """Dot grid whose radius swells toward one side."""
    step = rng.randint(34, 50)
    flip = rng.random() < 0.5
    for gy in range(0, h + step, step):
        for gx in range(0, w + step, step):
            t = (gx / w) if flip else 1 - (gx / w)
            r = step * (0.10 + 0.34 * t) * rng.uniform(0.7, 1.25)
            colour = rng.choice(palette["accents"])
            d.ellipse([gx - r, gy - r, gx + r, gy + r],
                      fill=colour[:3] + (rng.randint(24, 78),))


def pattern_isocubes(d, w, h, palette, rng):
    """Isometric cube lattice - three faces per cube at different alphas."""
    s = rng.randint(34, 50)
    hx, hy = s * math.cos(math.radians(30)), s * math.sin(math.radians(30))
    for row in range(-2, int(h / hy) + 3):
        for col in range(-2, int(w / (hx * 2)) + 3):
            cx = col * hx * 2 + (hx if row % 2 else 0)
            cy = row * (s + hy) / 2
            top = [(cx, cy - s), (cx + hx, cy - hy), (cx, cy), (cx - hx, cy - hy)]
            left = [(cx - hx, cy - hy), (cx, cy), (cx, cy + s), (cx - hx, cy + hy)]
            right = [(cx + hx, cy - hy), (cx, cy), (cx, cy + s), (cx + hx, cy + hy)]
            colour = rng.choice(palette["accents"])
            for face, alpha in ((top, 62), (left, 34), (right, 20)):
                d.polygon(face, fill=colour[:3] + (int(alpha * rng.uniform(.6, 1.2)),))


def pattern_circuit(d, w, h, palette, rng):
    """Orthogonal traces with solder pads - a nod to the maker theme."""
    lanes = rng.randint(7, 11)
    for i in range(lanes):
        y = h * (i + 0.5) / lanes + rng.uniform(-8, 8)
        colour = rng.choice(palette["accents"])
        alpha = rng.randint(34, 84)
        x = rng.uniform(-60, 120)
        pts = [(x, y)]
        while x < w + 60:
            run = rng.uniform(70, 220)
            x += run
            pts.append((x, y))
            if rng.random() < 0.45 and x < w:
                jog = rng.choice([-1, 1]) * rng.uniform(18, 40)
                x += abs(jog)
                y += jog
                pts.append((x, y))
        d.line(pts, fill=colour[:3] + (alpha,), width=rng.randint(3, 6), joint="curve")
        for px, py in pts[::3]:
            r = rng.uniform(5, 10)
            d.ellipse([px - r, py - r, px + r, py + r],
                      fill=colour[:3] + (min(255, alpha + 40),))


def pattern_waves(d, w, h, palette, rng):
    """Layered sine bands sweeping across the full height."""
    layers = rng.randint(7, 10)
    for i in range(layers):
        colour = palette["accents"][i % len(palette["accents"])]
        amp = rng.uniform(30, 78)
        freq = rng.uniform(0.8, 2.2)
        phase = rng.uniform(0, math.tau)
        base = h * (-0.10 + 1.25 * i / layers)
        pts = [(x, base + amp * math.sin(freq * math.tau * x / w + phase))
               for x in range(0, w + 12, 12)]
        d.polygon(pts + [(w, h + 40), (0, h + 40)],
                  fill=colour[:3] + (rng.randint(40, 82),))


def pattern_shards(d, w, h, palette, rng):
    """Overlapping angular quads, like cut glass."""
    for _ in range(rng.randint(45, 70)):
        cx, cy = rng.uniform(-60, w + 60), rng.uniform(-60, h + 60)
        size = rng.uniform(110, 320)
        rot = rng.uniform(0, math.tau)
        pts = []
        for k in range(4):
            a = rot + k * math.pi / 2 + rng.uniform(-0.35, 0.35)
            r = size * rng.uniform(0.5, 1.0)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a) * 0.55))
        colour = rng.choice(palette["accents"])
        d.polygon(pts, fill=colour[:3] + (rng.randint(30, 68),))


def pattern_nested(d, w, h, palette, rng):
    """Rows of nested squares, each row rotated a little."""
    step = rng.randint(110, 170)
    for cx in range(0, w + step, step):
        for cy in range(0, h + step, step):
            rot = rng.uniform(0, math.pi / 2)
            colour = rng.choice(palette["accents"])
            for k in range(rng.randint(2, 5)):
                r = step * 0.46 * (1 - k * 0.22)
                pts = [(cx + r * math.cos(rot + a * math.pi / 2),
                        cy + r * math.sin(rot + a * math.pi / 2)) for a in range(4)]
                d.polygon(pts, outline=colour[:3] + (rng.randint(24, 66),), width=3)


def pattern_rays(d, w, h, palette, rng):
    """A fan of thin wedges from one corner."""
    ox = rng.choice([-40, w + 40])
    oy = rng.choice([-40, h + 40])
    count = rng.randint(46, 68)
    for i in range(count):
        a0 = math.tau * (i / count)
        a1 = a0 + math.tau / count * rng.uniform(0.35, 0.62)
        span = max(w, h) * 2.4
        colour = rng.choice(palette["accents"])
        d.polygon([(ox, oy),
                   (ox + span * math.cos(a0), oy + span * math.sin(a0)),
                   (ox + span * math.cos(a1), oy + span * math.sin(a1))],
                  fill=colour[:3] + (rng.randint(34, 74),))


# Sparse patterns need a bright ground to read at all - on a near-black
# "ink" palette they disappear. Only the dense ones get to go dark.
INK_SAFE = {"triangles", "hexes", "dots", "isocubes", "circuit", "nested", "stripes"}

PATTERNS = {
    "triangles": pattern_triangles,
    "arcs": pattern_arcs,
    "hexes": pattern_hexes,
    "stripes": pattern_stripes,
    "dots": pattern_dots,
    "isocubes": pattern_isocubes,
    "circuit": pattern_circuit,
    "waves": pattern_waves,
    "shards": pattern_shards,
    "nested": pattern_nested,
    "rays": pattern_rays,
}


# --------------------------------------------------------------------------
# Rendering
# --------------------------------------------------------------------------

def pattern_deck(course_name: str) -> list[str]:
    """A per-course shuffle of every pattern.

    Lessons deal from this in order, so a course works through all eleven
    patterns before any repeats. Choosing independently per lesson clusters
    duplicates - a twenty lesson course would typically show `waves` four
    times and never show `arcs` at all.
    """
    deck = sorted(PATTERNS)
    seeded_random(course_name, "deck").shuffle(deck)
    return deck


def render_banner(course_name: str, lesson_stem: str, style: str | None = None,
                  index: int | None = None) -> Image.Image:
    """Render one banner. Same inputs always produce the same image."""
    # The course picks the base hue; the lesson only rotates around it, so a
    # course looks like a set rather than a random assortment.
    course_hue = seeded_random(course_name).random()

    rng = seeded_random(course_name, lesson_stem)

    if style:
        name = style
    elif index is None:
        name = rng.choice(sorted(PATTERNS))
    else:
        name = pattern_deck(course_name)[index % len(PATTERNS)]

    if index is None:
        hue_offset = rng.uniform(-HUE_SPREAD, HUE_SPREAD)
    else:
        # An irrational step walks the band evenly instead of clumping
        hue_offset = (((index * 0.3819660) % 1.0) - 0.5) * 2 * HUE_SPREAD

    palette = build_palette(rng, course_hue, hue_offset, allow_ink=name in INK_SAFE)

    w, h = WIDTH * SUPERSAMPLE, HEIGHT * SUPERSAMPLE
    base = Image.new("RGB", (w, h))
    gradient_background(ImageDraw.Draw(base), (w, h), palette, rng)

    overlay = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    PATTERNS[name](ImageDraw.Draw(overlay, "RGBA"), w, h, palette, rng)

    # The patterns pick low alphas so shapes layer without turning to soup.
    # Because the shape colours are tone-on-tone, those alphas end up almost
    # invisible - so lift the whole overlay in one place rather than tuning
    # eleven pattern functions. Ink palettes need less, their tones already
    # contrast more against a near-black ground.
    gain = 2.6 if palette["family"] == "ink" else 3.4
    overlay.putalpha(overlay.getchannel("A").point(
        lambda v: min(255, int(v * gain))))

    base = Image.alpha_composite(base.convert("RGBA"), overlay)
    return base.convert("RGB").resize((WIDTH, HEIGHT), Image.LANCZOS)


# --------------------------------------------------------------------------
# Wiring banners into the lessons
# --------------------------------------------------------------------------

def lesson_files(course_dir: Path) -> tuple[str, list[str]]:
    """Course name and the ordered lesson filenames from course.yml."""
    data = yaml.safe_load((course_dir / "course.yml").read_text())
    course = data[0]
    files = [f for section in course["content"] for f in section["section"]["content"]]
    return course["name"], files


def set_frontmatter_cover(text: str, cover: str) -> str:
    """Replace or insert the `cover:` key in a lesson's frontmatter."""
    head, sep, body = text.partition("\n---\n")
    if not sep or not head.startswith("---"):
        raise ValueError("lesson has no frontmatter")
    if re.search(r"^cover:.*$", head, re.MULTILINE):
        head = re.sub(r"^cover:.*$", f"cover: {cover}", head, count=1, flags=re.MULTILINE)
    else:
        head = head.rstrip("\n") + f"\ncover: {cover}"
    return head + sep + body


def set_cover_line(text: str) -> str:
    """Normalise the top of the body to: cover image, blank line, rule.

    Rebuilt from scratch each time rather than substituted in place, so a
    lesson ends up identical whether it had no cover line, an old
    hard-coded one, or an already-correct one.
    """
    head, sep, body = text.partition("\n---\n")

    # Drop any cover line already sitting at the top, plus the rule under it
    body = body.lstrip("\n")
    match = COVER_LINE_RE.match(body)
    if match:
        body = body[match.end():].lstrip("\n")
        if body.startswith("---"):
            body = body[3:].lstrip("\n")

    return f"{head}{sep}\n{COVER_LINE}\n\n---\n\n{body}"


def generate_course(slug: str, force: bool = False, images_only: bool = False,
                    style: str | None = None) -> int:
    course_dir = SOURCE / slug
    if not (course_dir / "course.yml").exists():
        print(f"No course.yml in {course_dir}", file=sys.stderr)
        return 1

    course_name, files = lesson_files(course_dir)
    banner_dir = course_dir / "assets" / BANNER_DIR_NAME
    banner_dir.mkdir(parents=True, exist_ok=True)

    made = skipped = 0
    for position, filename in enumerate(files):
        lesson_path = course_dir / filename
        if not lesson_path.exists():
            print(f"  missing lesson {filename}", file=sys.stderr)
            continue

        stem = Path(filename).stem
        banner_path = banner_dir / f"{stem}.jpg"

        if banner_path.exists() and not force:
            skipped += 1
        else:
            render_banner(course_name, stem, style, index=position).save(
                banner_path, "JPEG", quality=82, optimize=True)
            made += 1

        if not images_only:
            cover = f"/learn/{slug}/assets/{BANNER_DIR_NAME}/{stem}.jpg"
            text = lesson_path.read_text()
            text = set_frontmatter_cover(text, cover)
            text = set_cover_line(text)
            lesson_path.write_text(text)

    total_kb = sum(p.stat().st_size for p in banner_dir.glob("*.jpg")) / 1024
    print(f"{slug}: {made} generated, {skipped} kept, "
          f"{total_kb:.0f}KB total ({total_kb / max(1, len(files)):.0f}KB each)")
    return 0


def contact_sheet(out: Path) -> int:
    """One banner per style, stacked, for eyeballing the look."""
    styles = sorted(PATTERNS)
    sheet = Image.new("RGB", (WIDTH, HEIGHT * len(styles)), (0, 0, 0))
    for i, name in enumerate(styles):
        # Vary the course name per row so the sheet samples the whole wheel
        sheet.paste(render_banner(f"Sheet {i}", name, style=name), (0, i * HEIGHT))
    sheet.save(out, "JPEG", quality=80, optimize=True)
    print(f"{out} ({len(styles)} styles)")
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("slug", nargs="?", help="Course folder name under /source")
    parser.add_argument("--force", action="store_true", help="Regenerate existing banners")
    parser.add_argument("--images-only", action="store_true",
                        help="Do not touch the lesson frontmatter")
    parser.add_argument("--style", choices=sorted(PATTERNS),
                        help="Force one pattern instead of varying per lesson")
    parser.add_argument("--list-styles", action="store_true")
    parser.add_argument("--contact-sheet", type=Path,
                        help="Write a preview of every style to this path")
    args = parser.parse_args(argv)

    if args.list_styles:
        print("\n".join(sorted(PATTERNS)))
        return 0
    if args.contact_sheet:
        return contact_sheet(args.contact_sheet)
    if not args.slug:
        parser.error("a course slug is required")
    return generate_course(args.slug, args.force, args.images_only, args.style)


if __name__ == "__main__":
    raise SystemExit(main())
