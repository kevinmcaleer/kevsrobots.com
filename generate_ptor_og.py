#!/usr/bin/env python3
"""Generate social-share (Open Graph) images for the Periodic Table of Robotics.

Produces one 1200x630 PNG per element in web/assets/img/ptor/og/<slug>.png,
styled like the element's table tile, plus table.png for the index page.

Re-run after adding or changing elements:
    python3 generate_ptor_og.py
"""
from __future__ import annotations

import os
import yaml
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(ROOT, "web", "_data", "periodic_table.yml")
OUT = os.path.join(ROOT, "web", "assets", "img", "ptor", "og")

W, H = 1200, 630

# Mac font paths; fall back to PIL default if unavailable.
FONT_DIRS = ["/System/Library/Fonts/Supplemental", "/Library/Fonts", "/usr/share/fonts"]


def _font(names, size):
    for d in FONT_DIRS:
        for n in names:
            p = os.path.join(d, n)
            if os.path.exists(p):
                return ImageFont.truetype(p, size)
    return ImageFont.load_default()


def hex_to_rgb(h):
    h = h.lstrip("#")
    return tuple(int(h[i:i + 2], 16) for i in (0, 2, 4))


def text_colour(bg):
    # Perceived luminance — dark text on light tiles, white on dark.
    r, g, b = bg
    lum = (0.299 * r + 0.587 * g + 0.114 * b)
    return (33, 37, 41) if lum > 150 else (255, 255, 255)


def centre(draw, cx, y, text, font, fill):
    bbox = draw.textbbox((0, 0), text, font=font)
    w = bbox[2] - bbox[0]
    draw.text((cx - w / 2, y), text, font=font, fill=fill)


def wrap(draw, text, font, max_w):
    words = text.split()
    lines, cur = [], ""
    for word in words:
        trial = (cur + " " + word).strip()
        if draw.textbbox((0, 0), trial, font=font)[2] <= max_w:
            cur = trial
        else:
            if cur:
                lines.append(cur)
            cur = word
    if cur:
        lines.append(cur)
    return lines


def build_element(el, cat_colour, cat_label):
    bg = hex_to_rgb(cat_colour)
    fg = text_colour(bg)
    img = Image.new("RGB", (W, H), bg)
    d = ImageDraw.Draw(img)

    f_num = _font(["Arial.ttf"], 40)
    f_code = _font(["Arial Black.ttf", "Arial Bold.ttf"], 240)
    f_name = _font(["Arial Bold.ttf"], 70)
    f_blurb = _font(["Arial.ttf"], 32)
    f_brand = _font(["Arial Bold.ttf"], 30)

    # Atomic number, top-left
    d.text((70, 55), str(el["number"]), font=f_num, fill=fg)
    # Brand, top-right
    brand = "The Periodic Table of Robotics"
    bb = d.textbbox((0, 0), brand, font=f_brand)
    d.text((W - 70 - (bb[2] - bb[0]), 62), brand, font=f_brand, fill=fg)

    # Big code, vertically centred in the upper area using its real glyph box
    cbox = d.textbbox((0, 0), el["code"], font=f_code)
    code_w = cbox[2] - cbox[0]
    code_h = cbox[3] - cbox[1]
    code_y = 150
    d.text((W / 2 - code_w / 2 - cbox[0], code_y - cbox[1]), el["code"], font=f_code, fill=fg)

    # Name, below the code glyph box with a clear gap
    name_y = code_y + code_h + 34
    centre(d, W / 2, name_y, el["name"], f_name, fg)

    # Blurb (wrapped, up to 2 lines)
    blurb = el.get("blurb", "")
    lines = wrap(d, blurb, f_blurb, W - 220)[:2]
    y = name_y + 92
    for ln in lines:
        centre(d, W / 2, y, ln, f_blurb, fg)
        y += 40

    # Category pill, bottom-left
    cat_text = cat_label
    cb = d.textbbox((0, 0), cat_text, font=f_brand)
    pad = 18
    pill_w = (cb[2] - cb[0]) + pad * 2
    d.rounded_rectangle([70, H - 70, 70 + pill_w, H - 22], radius=24,
                        outline=fg, width=2)
    d.text((70 + pad, H - 64), cat_text, font=f_brand, fill=fg)

    return img


def build_table(elements, categories):
    """A simple poster of the full grid for the index page OG image."""
    bg = (18, 22, 30)
    img = Image.new("RGB", (W, H), bg)
    d = ImageDraw.Draw(img)
    cmap = {c["key"]: hex_to_rgb(c["color"]) for c in categories}

    f_title = _font(["Arial Black.ttf", "Arial Bold.ttf"], 56)
    f_code = _font(["Arial Bold.ttf"], 22)

    centre(d, W / 2, 28, "The Periodic Table of Robotics", f_title, (255, 255, 255))

    cols, gap = 18, 6
    grid_top, grid_left = 130, 60
    cell = (W - grid_left * 2 - (cols - 1) * gap) / cols
    for el in elements:
        row, col = el["row"], el["col"]
        x = grid_left + (col - 1) * (cell + gap)
        y = grid_top + (row - 1) * (cell + gap)
        bgc = cmap.get(el["category"], (120, 120, 120))
        d.rounded_rectangle([x, y, x + cell, y + cell], radius=6, fill=bgc)
        fg = text_colour(bgc)
        centre(d, x + cell / 2, y + cell / 2 - 14, el["code"], f_code, fg)
    return img


def main():
    os.makedirs(OUT, exist_ok=True)
    doc = yaml.safe_load(open(DATA))
    cats = {c["key"]: c for c in doc["categories"]}
    elements = doc["elements"]

    for el in elements:
        cat = cats[el["category"]]
        img = build_element(el, cat["color"], cat["label"])
        img.save(os.path.join(OUT, f"{el['slug']}.png"), optimize=True)

    table = build_table(elements, doc["categories"])
    table.save(os.path.join(OUT, "table.png"), optimize=True)

    print(f"Wrote {len(elements)} element images + table.png to {OUT}")


if __name__ == "__main__":
    main()
