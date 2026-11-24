---
layout: lesson
title: Pixel Art and Animation in Snakie
author: Kevin McAleer
type: page
cover: /learn/snakie_sprites/assets/banners/00_intro.jpg
date: 2026-11-21
next: 01_tiny_screens_and_pixels.html
description: Draw sprites and animations in Snakie and bring a blinking robot face
  to life on an OLED screen with MicroPython
percent: 7
duration: 4
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

## Overview

A robot with a face feels alive. Two eyes that blink, glance sideways and droop when the lights go out will get more smiles than any amount of clever motor code.

In this course you'll draw that face yourself, one pixel at a time, in Snakie's **Sprite editor**. Then you'll write the MicroPython that puts it on a small OLED screen and brings it to life. You don't need the screen to start. Snakie has a simulated one built in.

![The eye animation you will draw](/learn/snakie_sprites/assets/eye.gif){:style="width:192px;image-rendering:pixelated"}

That's the animation you'll make in the first half of the course: five frames, 16 pixels square.

{% include youtubeplayer.html id="QVe9meIHZtk" %}

New to Snakie? Watch Kev's introduction above first, then come back.

---

## Course Content

- How tiny screens work, and what a sprite is
- A tour of the Sprite editor
- Drawing an eye with the draw, erase and fill tools
- Adding frames to make it blink and look around
- Saving your work and exporting it as code, a GIF or a picture
- How a picture becomes a handful of bytes
- Drawing on a screen with MicroPython's `framebuf`
- Showing your sprite, then animating it
- Scaling the eyes up and making them react to a light sensor
- Loading sprite files straight from the board
- Moving it all to a real SSD1306 OLED

---

## Key Results

After this course you will be able to:

- Draw 1-bit sprites and frame animations in Snakie
- Export a sprite as a MicroPython module and use it in a program
- Explain how rows of pixels are packed into bytes
- Use `framebuf` to draw shapes, text and sprites
- Build an animation loop that feels alive
- Run the same face on a real OLED display

---

## What you'll need

- **Snakie**, in your browser at [app.snakie.org](https://app.snakie.org) or the desktop app. Everything up to the last two lessons runs on the simulated device
- A little MicroPython: variables, loops and functions. [Learn MicroPython - The basics](/learn/micropython/) covers all of it
- Optional, for the final lesson: a Raspberry Pi Pico and an SSD1306 OLED display (128×64, I2C). They cost a few pounds

If Snakie itself is new to you, take [Getting Started with Snakie](/learn/snakie_getting_started/) first. If you haven't used the Instrument Dock yet, [Snakie Instruments](/learn/snakie_instruments/) is a good companion.

---

## How the course works

Each lesson is short and ends with something to try and a quick quiz. Several lessons have a code box you can run right on the page, like this one:

{% capture snakie_code %}rows = [
    "..####..",
    ".#....#.",
    "#.#..#.#",
    "#......#",
    "#.#..#.#",
    "#..##..#",
    ".#....#.",
    "..####..",
]

for row in rows:
    print(row.replace("#", "██").replace(".", "  "))
{% endcapture %}
{% include snakie.html code=snakie_code name="smiley.py" view="code" %}

Press **Run**. Then change some `.` to `#` and run it again. You've just edited a sprite by hand. The Sprite editor makes that a lot more comfortable.

The code boxes show text. To see sprites on the simulated screen you'll use the full Snakie app, and we'll walk through that step by step.

> **Teachers**: This course suits beginners who have met a little MicroPython. Each lesson is roughly 30 to 45 minutes of drawing or coding. Snakie runs in the browser at app.snakie.org with nothing to install (or as the desktop app), and everything up to the last two lessons runs on the simulated device, so no hardware is needed. A Raspberry Pi Pico and an SSD1306 OLED are optional for the final lesson and cost a few pounds, so one set per group is enough. Sprites are saved into a project folder, so on shared computers give each learner or pair their own folder.
