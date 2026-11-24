---
title: Drawing shapes and icons
description: Draw pixels, lines, rectangles and ellipses on the OLED, then use them to build a door icon with an open and a closed state
layout: lesson
type: page
cover: /learn/micropython_projects/assets/banners/07_drawing_shapes_and_icons.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Text is useful, but a picture says "open" faster than a word does. The SSD1306 driver is built on MicroPython's `framebuf`, which gives us a small toolbox of drawing commands. In this lesson we learn them, then use them to draw a door.

---

## The toolbox

Every command takes a colour as its last argument: `1` draws, `0` erases.

| Command | What it draws |
|---|---|
| `oled.pixel(x, y, 1)` | One dot |
| `oled.line(x1, y1, x2, y2, 1)` | A line between two points |
| `oled.rect(x, y, w, h, 1)` | A rectangle outline |
| `oled.fill_rect(x, y, w, h, 1)` | A solid rectangle |
| `oled.ellipse(x, y, rx, ry, 1)` | An ellipse outline, centred on (x, y) |
| `oled.ellipse(x, y, rx, ry, 1, True)` | A solid ellipse |
{:class="table table-single"}

There is no separate `fill_ellipse()`. In MicroPython's `framebuf`, a solid ellipse is the same `ellipse()` call with `True` as an extra last argument. (Kev's OLED post writes `fill_ellipse()`, which isn't in `framebuf`, so use the `True` version.) Ellipses need a reasonably recent MicroPython, so update the firmware if you get an `AttributeError`.

(0, 0) is the **top-left** corner. x goes right up to 127, and y goes **down** up to 63.

## A smiley to practise

```python
from machine import Pin, I2C
from ssd1306 import SSD1306_I2C

i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=400000)
oled = SSD1306_I2C(128, 64, i2c)

oled.fill(0)

oled.ellipse(64, 32, 30, 30, 1)          # face
oled.ellipse(54, 25, 3, 3, 1, True)       # left eye
oled.ellipse(74, 25, 3, 3, 1, True)       # right eye
oled.line(50, 42, 64, 50, 1)             # smile, left half
oled.line(64, 50, 78, 42, 1)             # smile, right half

oled.show()
```

## Drawing a door

A door is just a tall rectangle with a handle. When it is **closed** we fill it in. When it is **open** we draw only the outline, with a line to show it swinging out.

```python
from machine import Pin, I2C
from ssd1306 import SSD1306_I2C
import time

i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=400000)
oled = SSD1306_I2C(128, 64, i2c)


def draw_door(oled, x, y, is_open):
    """Draw a 30 x 44 door with its top-left corner at (x, y)."""
    if is_open:
        oled.rect(x, y, 30, 44, 1)              # empty doorway
        oled.line(x, y, x + 20, y + 6, 1)       # door swung open,
        oled.line(x + 20, y + 6, x + 20, y + 50, 1)   # drawn in a slanted view
        oled.line(x, y + 44, x + 20, y + 50, 1)
    else:
        oled.fill_rect(x, y, 30, 44, 1)         # solid door
        oled.ellipse(x + 24, y + 22, 2, 2, 0, True)    # handle, cut out in black


# Flip between the two so we can see both
is_open = False
for _ in range(6):
    oled.fill(0)
    draw_door(oled, 49, 10, is_open)
    oled.show()
    is_open = not is_open
    time.sleep(1)
```

Notice `draw_door()` is a **function**. We write the drawing code once and call it with different arguments. In the next lesson the buddy will call it with whatever the reed switch says.

> **Tip:** to rub something out, draw over it with colour `0`, like the handle above. Or call `oled.fill(0)` and redraw the whole screen, which is simplest.

## Try it Yourself

1. Add a doorknob to the open door.
2. Change the door's width and height, and see what breaks. What would you have to change to make them parameters?
3. Challenge: draw a padlock, or a lid, as a second icon for a different gadget.

## Common Issues

**Problem**: My shape appears in the wrong place.

**Solution**: Remember y counts down from the top. The bottom-right pixel is (127, 63), not (127, 0).

**Why**: Screens count from the top-left, unlike graphs on paper.

**Problem**: Old drawings stay behind when I draw a new one.

**Solution**: Call `oled.fill(0)` at the start of each new frame.

**Why**: The buffer remembers everything until you clear it.
