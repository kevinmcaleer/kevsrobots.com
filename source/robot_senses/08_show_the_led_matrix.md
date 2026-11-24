---
title: Show with the LED Matrix
description: Draw a face on the Modulino LED Matrix, understand why nothing appears until you call show, and fix an upside-down picture
layout: lesson
type: page
cover: /learn/robot_senses/assets/banners/08_show_the_led_matrix.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Sense four is expression. The **Modulino LED Matrix** is a panel of 96 blue LEDs, laid out 12 across and 8 down. It's the robot's face. A face tells you what the robot is thinking at a glance, with no laptop attached.

## What it does

You draw pixels into a **buffer** in the module's memory, then push the buffer to the panel. Kev's code uses these calls:

- `set_pixel(x, y)` turns one pixel on. `x` is the column, 0 to 11. `y` is the row, 0 to 7.
- `clear()` empties the buffer.
- `show()` pushes the buffer to the panel.

The LED Matrix needs its address given explicitly on a non-Arduino board: `address=0x39`. The library's auto-discovery doesn't find it on every board.

## The show() rule

Every drawing call writes to the buffer, **not** the panel. Nothing lights up until you call `show()`. If your matrix stays dark and your code looks right, this is almost always why.

## Real-world robot example

A face is a picture made of rows of characters. A `#` is a lit pixel and a `.` is dark. This complete program draws a happy face.

```python
from time import sleep_ms
from machine import I2C, Pin
from modulino import ModulinoLEDMatrix

i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400_000)
matrix = ModulinoLEDMatrix(i2c_bus=i2c, address=0x39)

WIDTH = 12
HEIGHT = 8

HAPPY = (
    "............",
    "..##....##..",
    "..##....##..",
    "............",
    ".#........#.",
    "..########..",
    "............",
    "............",
)

def draw(face):
    matrix.clear()
    for y, row in enumerate(face):
        for x, cell in enumerate(row):
            if cell == "#":
                matrix.set_pixel(x, y)
    matrix.show()          # nothing lights up until this line

draw(HAPPY)
sleep_ms(2000)
matrix.clear().show()      # clear, then push the empty buffer
```
{: .no-snakie }

Drawing a face as a tuple of strings means you can see the picture right in your source code. That's how Kev's `faces.py` does it, and it makes redesigning a face a drawing job, not a coding job.

Here is the same idea, runnable anywhere. It prints each face as text, so you can design faces before the robot is built:

```python
SURPRISED = (
    "............",
    ".######.....",
    ".#....#.....",
    ".#....#.....",
    ".######.....",
    "............",
    "............",
    "............",
)

ALERT = (
    "............",
    ".#########..",
    "..#######...",
    "...#####....",
    "............",
    "............",
    "............",
    "............",
)

for name, face in (("SURPRISED", SURPRISED), ("ALERT", ALERT)):
    print(name)
    for row in face:
        print(row.replace(".", " "))
```

## When the face is upside down

The module can't know which way up it was seated in the holder, so the correction is in software. If your picture is upside down, flip it on both axes, because a 180 degree rotation is a flip on each axis. Kev's config has two flags for this and sets both to `True` on his robot.

| Symptom | Fix |
|---|---|
| Upside down | Flip both x and y |
| Mirrored left to right | Flip x only |
| Upside down but text reads correctly | Flip y only |
{:class="table table-single"}

This function flips an ASCII face. It's complete and runs anywhere:

```python
def reverse(seq):
    # MicroPython can't always slice with [::-1], so count backwards instead
    return [seq[i] for i in range(len(seq) - 1, -1, -1)]

def orient(face, flip_x, flip_y):
    rows = reverse(face) if flip_y else list(face)
    if flip_x:
        rows = ["".join(reverse(row)) for row in rows]
    return rows

LOOK_LEFT = (
    "............",
    ".##..##.....",
    ".#....#.....",
    "............",
    "............",
    "............",
    "............",
    "............",
)

for row in orient(LOOK_LEFT, flip_x=True, flip_y=True):
    print(row)
```

Call `orient()` on the face before you draw it. Notice that `reverse()` counts backwards with `range()` instead of slicing with `[::-1]`. Kev's own code does this too, because slicing with a negative step isn't available on every MicroPython build.

**Why this matters:** a face turns a robot from a box of parts into something that reacts. Kev's robot pulls a startled face when you pick it up, and a happy one when you put it down.

## Try it Yourself

1. Redraw `HAPPY` as a sad face by changing the mouth rows.
2. Draw a face with the eyes looking to the right.
3. Challenge: make the robot blink by drawing an open-eyed face, a closed-eyed face, then the open one again, with a short pause between.

## Common Issues

**Problem**: The matrix stays dark but the code looks right.
**Solution**: Add `matrix.show()` after you draw.
**Why**: Drawing writes to a buffer. Nothing reaches the panel until `show()`.

**Problem**: The face is upside down.
**Solution**: Flip both axes in your drawing code.
**Why**: The module doesn't know how it was mounted.

**Problem**: Faces smear into each other.
**Solution**: Call `matrix.clear()` before you draw the next one.
**Why**: Pixels from the last face stay in the buffer until you clear them.
