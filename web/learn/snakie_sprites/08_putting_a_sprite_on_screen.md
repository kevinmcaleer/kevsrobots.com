---
layout: lesson
title: Putting a Sprite on Screen
author: Kevin McAleer
type: page
cover: /learn/snakie_sprites/assets/banners/08_putting_a_sprite_on_screen.jpg
date: 2026-11-21
previous: 07_a_screen_in_the_simulator.html
next: 09_animating_the_face.html
description: Import your exported sprite module and blit it onto the simulated display
  to make a pair of robot eyes
percent: 63
duration: 5
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

By the end of this lesson you will be able to import your exported sprite module and blit it onto the simulated display.

## What the module gives you

Open the `eye.py` you exported in lesson 5. Besides the frame data, it has three helpers:

| Name | Gives you |
|---|---|
| `eye.WIDTH`, `eye.HEIGHT`, `eye.FPS` | The sprite's size and speed |
| `eye.FRAMES` | The raw bytes of every frame |
| `eye.frame(i)` | Frame `i` as a small frame buffer, ready to blit |
| `eye.pixel(i, x, y)` | `True` if that pixel of frame `i` is lit |
| `eye.play(display, x, y, loops)` | Plays the whole animation on a display |
{:class="table table-single"}

Frames are numbered from 0, so your five are 0 to 4.

---

## Blit

**Blit** means "copy this small picture onto that big one". It's the heart of sprite graphics:

```python
screen.blit(eye.frame(0), 40, 24)
```
{: .no-snakie }

That copies frame 0 so its top-left corner lands at x = 40, y = 24.

See it in miniature. This box makes a tiny 7×5 sprite and blits it twice onto a 32×8 buffer:

{% capture snakie_code %}import framebuf

SHAPE = b'\x7c\xc6\xd6\xc6\x7c'
sprite = framebuf.FrameBuffer(bytearray(SHAPE), 7, 5, framebuf.MONO_HLSB)

W, H = 32, 8
canvas = framebuf.FrameBuffer(bytearray(W * H // 8), W, H, framebuf.MONO_HLSB)

canvas.blit(sprite, 4, 1)
canvas.blit(sprite, 18, 1)

for y in range(H):
    print("".join("#" if canvas.pixel(x, y) else "." for x in range(W)))
{% endcapture %}
{% include snakie.html code=snakie_code name="blit.py" view="code" %}

One drawing, two eyes. Change the `18` to `12` and run it. What happens when the sprites overlap? Try `40`, which is off the right-hand edge.

---

## Get the sprite onto the board

Right-click `eye.py` in Local Files and choose **Upload to board**, the same way you uploaded `simscreen.py`.

Refresh **Device Files** to check it has arrived.

![Uploaded files listed in Device Files](/learn/snakie_sprites/assets/device_files.jpg){:class="img-fluid w-100"}

---

## A face

Make a new file, `face.py`:

```python
from simscreen import SimScreen
import eye

screen = SimScreen()


def face(i):
    screen.fill(0)
    screen.blit(eye.frame(i), 40, 24)
    screen.blit(eye.frame(i), 72, 24)
    screen.show()


face(0)
```

Press **Run**.

![Two eyes on the simulated display](/learn/snakie_sprites/assets/face.jpg){:class="img-fluid w-100"}

Two eyes, looking back at you.

---

## Where do 40, 72 and 24 come from?

Placing things on a screen is mostly small sums. The screen is 128 wide and 64 tall, and each eye is 16 square.

- **Down**: to centre a 16-pixel sprite in 64 rows, leave (64 - 16) // 2 = 24 above it
- **Across**: the middle of the screen is x = 64. Put one eye 24 pixels left of the middle and one 8 pixels right of it, at 40 and 72, and there's a 16-pixel gap between them

You could write those sums into the code:

```python
GAP = 16
left = 64 - GAP // 2 - eye.WIDTH
right = 64 + GAP // 2
top = (64 - eye.HEIGHT) // 2
```

Now changing `GAP` moves both eyes and keeps the face centred.

---

## Try the other frames

Change the last line to `face(2)` and run it. Then `face(3)` and `face(4)`.

Numbers like those are hard to remember. Give them names at the top of the file:

```python
OPEN, HALF, CLOSED, LEFT, RIGHT = 0, 1, 2, 3, 4
```

Now `face(CLOSED)` says what it means.

---

## Try it Yourself

1. Use the `GAP` sums, then try a gap of 4 and a gap of 40
2. Make the robot go cross-eyed: show `RIGHT` in the left eye and `LEFT` in the right eye. You'll need `face` to take two frame numbers
3. Add a mouth with `screen.hline`, 8 pixels below the eyes
4. Add a caption under the face with `screen.text`

---

## Common Issues

**Problem**: `ImportError: no module named 'eye'`.
**Solution**: Upload `eye.py` to the board.
**Why**: Like `simscreen.py`, it has to be on the device.

**Problem**: `IndexError` when I ask for a frame.
**Solution**: Use 0 to 4. `eye.frame(5)` doesn't exist.
**Why**: Five frames are numbered 0, 1, 2, 3 and 4.

**Problem**: Old eyes are left behind when I draw new ones.
**Solution**: Call `screen.fill(0)` before you blit.
**Why**: The buffer keeps whatever was drawn until you clear it.

**Problem**: The frames are in the wrong order, so `CLOSED` shows a different picture.
**Solution**: Either reorder the frames in the Sprite editor and export again, or change the numbers after `OPEN, HALF, CLOSED, LEFT, RIGHT =` to match your sprite.
**Why**: The names are only labels for positions in your filmstrip.

---

## Check your understanding

{% quiz %}
? What does screen.blit(eye.frame(0), 40, 24) do?
- [ ] Draws a 40 by 24 rectangle
- [x] Copies frame 0 onto the screen with its top-left corner at (40, 24)
- [ ] Plays the animation 40 times
> Blit copies a small frame buffer onto a larger one at the position you give.

? You drew five frames. What number is the last one?
- [ ] 5
- [x] 4
- [ ] 6
> Counting starts at 0, so five frames run from 0 to 4.

? Why give the frame numbers names such as CLOSED?
- [ ] The board runs named frames faster
- [ ] MicroPython requires it
- [x] The code is easier to read and harder to get wrong
> face(CLOSED) explains itself. face(2) makes you go and check.
{% endquiz %}
