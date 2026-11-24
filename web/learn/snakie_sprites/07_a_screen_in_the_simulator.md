---
layout: lesson
title: A Screen in the Simulator
author: Kevin McAleer
type: page
cover: /learn/snakie_sprites/assets/banners/07_a_screen_in_the_simulator.jpg
date: 2026-11-21
previous: 06_pixels_into_bytes.html
next: 08_putting_a_sprite_on_screen.html
description: Use MicroPython framebuf to draw shapes and text, and show them on the
  Display instrument of the simulated device
percent: 56
duration: 6
quiz_total: 39
date_updated: 2026-11-24
navigation:
- name: Pixel Art and Animation in Snakie
- content:
  - section: Overview
    content:
    - name: Pixel Art and Animation in Snakie
      link: 00_intro.html
    - name: Tiny Screens and Pixels
      link: 01_tiny_screens_and_pixels.html
  - section: Drawing
    content:
    - name: The Sprite Editor
      link: 02_the_sprite_editor.html
    - name: Drawing Your First Sprite
      link: 03_drawing_your_first_sprite.html
    - name: Frames and Animation
      link: 04_frames_and_animation.html
    - name: Saving and Exporting
      link: 05_saving_and_exporting.html
  - section: Sprites in Code
    content:
    - name: Pixels into Bytes
      link: 06_pixels_into_bytes.html
    - name: A Screen in the Simulator
      link: 07_a_screen_in_the_simulator.html
    - name: Putting a Sprite on Screen
      link: 08_putting_a_sprite_on_screen.html
    - name: Animating the Face
      link: 09_animating_the_face.html
    - name: Bigger and Smarter
      link: 10_bigger_and_smarter.html
  - section: Going Further
    content:
    - name: Sprite Files on the Board
      link: 11_sprite_files_on_the_board.html
    - name: A Real OLED
      link: 12_a_real_oled.html
    - name: Summary and What's Next
      link: 13_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to use framebuf to draw shapes and text and show them on the simulated Display.

## Meet framebuf

MicroPython comes with a module called `framebuf`. It gives you a **frame buffer**: a block of memory that stands for a screen, plus functions to draw on it.

Nothing appears anywhere while you draw. You're changing bytes in memory. When the picture is ready, you send the whole buffer to the screen in one go. That's why a display driver always has a `show()` function.

Try it here. This box draws into a small 32×12 buffer and prints the result as text:

{% capture snakie_code %}import framebuf

W, H = 32, 12
buf = bytearray(W * H // 8)
fb = framebuf.FrameBuffer(buf, W, H, framebuf.MONO_HLSB)

fb.rect(0, 0, 32, 12, 1)
fb.text("Hi!", 4, 2, 1)
fb.pixel(30, 10, 1)

for y in range(H):
    print("".join("#" if fb.pixel(x, y) else "." for x in range(W)))
{% endcapture %}
{% include snakie.html code=snakie_code name="framebuf_demo.py" view="code" %}

---

## The drawing functions

The ones you'll use most:

| Call | Draws |
|---|---|
| `fb.fill(0)` | Clears everything. `fill(1)` lights everything |
| `fb.pixel(x, y, 1)` | One pixel. Leave off the last number to **read** a pixel instead |
| `fb.hline(x, y, w, 1)` | A horizontal line `w` pixels long |
| `fb.vline(x, y, h, 1)` | A vertical line `h` pixels long |
| `fb.line(x1, y1, x2, y2, 1)` | A line between two points |
| `fb.rect(x, y, w, h, 1)` | The outline of a rectangle |
| `fb.fill_rect(x, y, w, h, 1)` | A solid rectangle |
| `fb.text("Hi", x, y, 1)` | Text. Each character is 8×8 pixels |
| `fb.blit(other, x, y)` | Copies another frame buffer onto this one |
{:class="table table-single"}

The last number is the colour: 1 for on, 0 for off. Drawing with 0 is how you rub things out.

Go back to the box and add `fb.fill_rect(20, 3, 6, 6, 1)`. Then add `fb.line(0, 0, 31, 11, 1)`.

---

## A pretend OLED

Text art is fine for a quick test, but Snakie can do better. Its **Display** instrument will show any frame buffer your program sends it, even on the simulated device.

A real display driver gives you an object with drawing functions and a `show()`. We'll make our own that behaves the same way, so that swapping in a real screen later takes one line.

In the Snakie app, in your `robot-face` folder, make a new file called `simscreen.py`:

```python
"""A pretend 128x64 OLED that shows up in Snakie's Display instrument."""

import binascii
import framebuf
import instruments as inst


class SimScreen(framebuf.FrameBuffer):
    def __init__(self, width=128, height=64):
        self.width = width
        self.height = height
        self.buffer = bytearray(width * height // 8)
        super().__init__(self.buffer, width, height, framebuf.MONO_HLSB)

    def show(self):
        data = binascii.b2a_base64(self.buffer).decode().strip()
        inst.screen_fb(data, self.width, self.height)
```

It's short, and it does two things:

- `SimScreen` **is** a frame buffer, so it has `fill`, `text`, `rect`, `blit` and all the rest
- `show()` packs the buffer into text and hands it to Snakie's `instruments` module, which passes it to the Display instrument

You don't need to understand every line. This file is a tool you write once and then import.

---

## Put it on the board

Your program runs on the device, so `simscreen.py` has to be there too.

**Right-click** `simscreen.py` in the Local Files list and choose **Upload to board**.

![Right-clicking a file to upload it to the board](/learn/snakie_sprites/assets/context_menu.jpg){:class="img-fluid w-100"}

The status bar confirms the upload. Press the refresh button in **Device Files** and you'll see it listed.

---

## Hello, screen

Make another file, `hello.py`:

```python
from simscreen import SimScreen

screen = SimScreen()

screen.fill(0)
screen.rect(0, 0, 128, 64, 1)
screen.text("Hello, robot!", 12, 20, 1)
screen.hline(12, 32, 104, 1)
screen.show()
```

Make sure the Display instrument is showing, then press **Run**.

![Text and shapes on the simulated display](/learn/snakie_sprites/assets/hello.jpg){:class="img-fluid w-100"}

There's your screen. The line under it reads **PROGRAM LIVE** while frames are arriving.

Try taking out `screen.show()` and running it again. Nothing changes on the display, because the drawing never left the board's memory.

---

## Try it Yourself

1. Draw a border two pixels thick. (Two `rect` calls, one inside the other)
2. Put your name in the centre of the screen. A character is 8 pixels wide, so a 5-letter name is 40 pixels, and it starts at x = (128 - 40) // 2
3. Draw a row of ten small squares along the bottom using a `for` loop and `fill_rect`
4. Add `import time`, then make a square slide across the screen by clearing, drawing and showing inside a loop with `time.sleep(0.05)`

---

## Common Issues

**Problem**: `ImportError: no module named 'simscreen'`.
**Solution**: Upload `simscreen.py` to the board with right-click, **Upload to board**.
**Why**: `import` looks for files on the device, and files in your project folder aren't copied there automatically.

**Problem**: The Display shows AWAITING FRAMEBUFFER.
**Solution**: Check your program calls `screen.show()`, and run it again.
**Why**: The instrument only draws what a program sends it.

**Problem**: I changed `simscreen.py` and nothing is different.
**Solution**: Save it, upload it again, press **Reset**, then Run.
**Why**: The board runs its own copy, and it keeps an imported module in memory until it's reset.

**Problem**: A long line of letters appears in the console.
**Solution**: Ignore it and carry on. Clear the console if it bothers you.
**Why**: Each frame travels to Snakie as a line of text, which is normally hidden. Now and then one slips through.

---

## Check your understanding

{% quiz %}
? When does a drawing actually appear on the screen?
- [ ] As soon as you call rect or text
- [x] When you call show()
- [ ] When the program ends
> Drawing changes the buffer in memory. show() sends the buffer to the display.

? What does fb.pixel(3, 4) do, with no third number?
- [ ] Turns the pixel on
- [ ] Turns the pixel off
- [x] Tells you whether the pixel is on
> With two arguments pixel() reads. With three it writes.

? Why do you upload simscreen.py to the board?
- [x] The program runs on the board, and import looks for files there
- [ ] To make it run faster
- [ ] Snakie can't open it otherwise
> Modules you import have to be on the device that runs the code.
{% endquiz %}
