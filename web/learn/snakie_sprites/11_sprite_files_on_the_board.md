---
layout: lesson
title: Sprite Files on the Board
author: Kevin McAleer
type: page
cover: /learn/snakie_sprites/assets/banners/11_sprite_files_on_the_board.jpg
date: 2026-11-21
previous: 10_bigger_and_smarter.html
next: 12_a_real_oled.html
description: Load .spr sprite files straight from the board, and use Snakie desktop
  features that link sprites and code
percent: 84
duration: 7
quiz_total: 39
date_updated: 2026-11-21
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

## Two ways to ship a sprite

So far you've used the exported **module**, `eye.py`. There's a second way: put the `.spr` **file** itself on the board and read it when the program runs.

| | Module (`eye.py`) | File (`eye.spr`) |
|---|---|---|
| Size on the board | About 1.7 KB | 176 bytes |
| Memory while running | All frames at once | One frame at a time |
| Changing the art | Export again, upload again | Upload the new `.spr` |
| Code needed | `import eye` | A small reader |
{:class="table table-single"}

For five small frames it makes no difference. For a 40-frame full-screen animation, the file wins easily: 40 KB of frames would crowd a Pico's memory as a module, but streams comfortably one frame at a time.

---

## What's inside a .spr file

The format is simple on purpose. It's a 16-byte **header** followed by the frames, each in the same `MONO_HLSB` layout you decoded in lesson 6.

| Bytes | Holds |
|---|---|
| 0 to 3 | The letters `SNKS`, so a program can check it's a Snakie sprite |
| 4 | Version, currently 1 |
| 5 | Flags |
| 6 | Pixel format, 0 for 1-bit |
| 7 | Unused |
| 8 and 9 | Width |
| 10 and 11 | Height |
| 12 and 13 | Number of frames |
| 14 and 15 | Milliseconds per frame |
{:class="table table-single"}

Do the sum for your eye: 16 bytes of header plus 5 frames of 32 bytes is 176. That's exactly the size Snakie shows next to the file.

---

## A reader

Make a new file, `spr.py`:

```python
"""Read Snakie .spr sprite files on the board."""

import struct
import framebuf


class Spr:
    """A .spr animation, read one frame at a time from the board's files."""

    def __init__(self, path):
        self.f = open(path, "rb")
        header = struct.unpack("<4sBBBBHHHH", self.f.read(16))
        magic, version, flags, fmt, _, w, h, count, ms = header
        if magic != b"SNKS":
            raise ValueError("not a Snakie sprite")
        self.width, self.height, self.count, self.ms = w, h, count, ms
        self.size = (w + 7) // 8 * h
        self.buf = bytearray(self.size)
        self.fb = framebuf.FrameBuffer(self.buf, w, h, framebuf.MONO_HLSB)

    def frame(self, i):
        self.f.seek(16 + i * self.size)
        self.f.readinto(self.buf)
        return self.fb
```

`struct.unpack` reads the header table above in one line. The odd-looking `"<4sBBBBHHHH"` is a recipe: four characters, four single bytes, then four two-byte numbers.

`frame(i)` jumps to the right place in the file and reads one frame into a buffer that's reused every time. However long the animation, it only ever holds one frame in memory.

---

## Play it

Make `player.py`:

```python
import time
from simscreen import SimScreen
from spr import Spr

screen = SimScreen()
eye = Spr("eye.spr")
print(eye.width, "x", eye.height, "-", eye.count, "frames,", eye.ms, "ms each")

for loop in range(3):
    for i in range(eye.count):
        screen.fill(0)
        screen.blit(eye.frame(i), 56, 24)
        screen.show()
        time.sleep_ms(eye.ms)
print("done")
```

Now upload **three** files with right-click, **Upload to board**: `eye.spr`, `spr.py` and, if it isn't there already, `simscreen.py`.

Then run `player.py`. The console reports what it found:

```
16 x 16 - 5 frames, 167 ms each
```

and the eye plays through its frames three times on the Display. The 167 ms comes from the 6 FPS you set in the editor: 1000 divided by 6.

![A sprite file playing on the simulated display](/learn/snakie_sprites/assets/player.jpg){:class="img-fluid w-100"}

---

## Upload sprite files with right-click

There are two upload controls in the Files panel, and for `.spr` files it matters which you use.

- **Right-click, Upload to board** copies the file exactly, byte for byte. Use this one
- The **upload arrow** under the Local Files list sends whatever is in the open editor tab as text. That's fine for Python, but a sprite isn't text, and it arrives damaged

If you click `eye.spr` in the file list, Snakie opens it in a tab as a jumble of characters. Close that tab without saving. To edit a sprite, use **Import** in the Sprite editor.

---

## Extras in the desktop app

The Snakie desktop app knows about sprite files in your project and joins them up with your code.

**Sprite names complete as you type.** Inside a string, Snakie offers the project's `.spr` files, so a typing mistake can't sneak through to the board.

![The editor offering a sprite file name](/learn/snakie_sprites/assets/complete.jpg){:class="img-fluid w-100"}

**The Sprite editor tells you where a sprite is used.** Under the top bar, a line lists the Python files that name the sprite you have open. Click one to jump to it.

![The Sprite editor showing which file uses the sprite](/learn/snakie_sprites/assets/usedin.jpg){:class="img-fluid w-100"}

If it says nothing references the sprite, it's safe to delete.

In the browser version these two don't apply. A sprite saved from the browser isn't tied to your project folder, so the editor reports that nothing references it even when your code does.

Both of these look for names written out in full, such as `"eye.spr"`. A name built while the program runs, such as `name + ".spr"`, can't be found this way.

---

## Try it Yourself

1. Change `range(3)` to `while True:` so the sprite loops until you press Stop
2. Draw a second sprite, a 16×16 heart that beats over two frames, save it as `heart.spr`, upload it and play it
3. Play both at once: make two `Spr` objects and blit them side by side
4. Print `eye.count * eye.size + 16` and compare it with the file size shown in Device Files

---

## Common Issues

**Problem**: `OSError: [Errno 2] ENOENT`.
**Solution**: Upload `eye.spr` to the board and check the name in your code matches exactly.
**Why**: The program opens the file on the device, and that error means there's no file by that name.

**Problem**: `ValueError: not a Snakie sprite`, or the animation plays with very long pauses.
**Solution**: Delete the file from the board and upload it again with right-click, **Upload to board**.
**Why**: The file was sent as text and its bytes were changed on the way. A damaged header gives nonsense for the size or the frame time.

**Problem**: The picture is scrambled.
**Solution**: Check `self.size` is `(w + 7) // 8 * h`.
**Why**: If the frame size is wrong, every frame after the first is read from the wrong place.

---

## Check your understanding

{% quiz %}
? Why does reading a .spr file use less memory than importing a module?
- [ ] The file is compressed
- [x] Only one frame is held in memory at a time
- [ ] Modules can't hold pictures
> The reader reuses a single buffer, so memory use doesn't grow with the number of frames.

? What are the first four bytes of a .spr file for?
- [ ] The width and height
- [x] The letters SNKS, which identify the file type
- [ ] The first row of pixels
> A marker at the start lets a program check it has the right kind of file before trusting the rest.

? How should you copy a .spr file to the board?
- [x] Right-click it and choose Upload to board
- [ ] Open it in a tab and use the upload arrow
- [ ] Paste it into the console
> Right-click upload copies the bytes exactly. Sending the open tab treats the sprite as text and damages it.
{% endquiz %}
