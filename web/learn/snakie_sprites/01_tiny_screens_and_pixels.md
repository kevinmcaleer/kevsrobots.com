---
layout: lesson
title: Tiny Screens and Pixels
author: Kevin McAleer
type: page
cover: /learn/snakie_sprites/assets/banners/01_tiny_screens_and_pixels.jpg
date: 2026-11-21
previous: 00_intro.html
next: 02_the_sprite_editor.html
description: How small OLED screens and LED matrices work, what a sprite is and why
  one bit per pixel is enough
percent: 14
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

By the end of this lesson you will be able to explain how a small screen is a grid of pixels, what a sprite is and why one bit per pixel is enough.

## A screen is a grid of lights

Look closely at any screen and you'll find a grid of tiny lights called **pixels**. Your phone has millions. The screens we put on small robots have far fewer, and that's what makes them fun: you can see and place every single one.

Two kinds turn up again and again in maker projects:

| Screen | Pixels | Notes |
|---|---|---|
| LED matrix | 8×8, 12×8, 16×16 | Big chunky LEDs. Great for eyes and icons |
| SSD1306 OLED | 128×64 | About 2.5 cm wide, sharp and bright, very cheap |
{:class="table table-single"}

On both, each pixel is either **on** or **off**. There's no colour and no grey. We call that **1-bit** graphics, because one bit (a 1 or a 0) is all you need to store each pixel.

---

## What is a sprite?

A **sprite** is a small picture that you draw once and then place wherever you like on the screen. Game characters are sprites. So are icons, and so is a robot's eye.

Sprites save a lot of work. Draw one eye, and you can show it twice to make a pair. Draw five versions of it (open, half shut, shut, looking left, looking right) and you have an animation.

Each version is called a **frame**. Show the frames quickly, one after another, and the eye appears to move. It's the same trick as a flip book.

---

## Where things are: x and y

Every pixel has an address made of two numbers:

- **x** counts across from the left edge, starting at 0
- **y** counts down from the top edge, starting at 0

So on a 128×64 screen the top-left pixel is `(0, 0)` and the bottom-right is `(127, 63)`.

Notice that y goes **down**. That's the opposite of the graphs you draw in maths, and it catches everybody out once.

Try it. This box draws a 16×8 grid and lights one pixel:

{% capture snakie_code %}WIDTH = 16
HEIGHT = 8

x = 3
y = 1

for row in range(HEIGHT):
    line = ""
    for col in range(WIDTH):
        if col == x and row == y:
            line += "#"
        else:
            line += "."
    print(line)
{% endcapture %}
{% include snakie.html code=snakie_code name="grid.py" view="code" %}

Change `x` and `y` and run it again. Where does `(15, 7)` land? What happens with `(16, 0)`?

---

## How much memory does a picture need?

A microcontroller has very little memory, so it's worth knowing what a picture costs.

Eight pixels fit in one **byte**, because a byte has eight bits. So:

| Picture | Pixels | Bytes |
|---|---|---|
| 8×8 icon | 64 | 8 |
| 16×16 sprite | 256 | 32 |
| 128×64 full screen | 8,192 | 1,024 |
{:class="table table-single"}

A whole OLED screen fits in about one kilobyte. A Raspberry Pi Pico has 264 of those, so you can afford plenty of sprites.

---

## Try it Yourself

1. In the code box above, light the four corner pixels one at a time. Write down their `(x, y)` addresses
2. Work out how many bytes a 12×8 LED matrix picture needs. Careful: each **row** is stored in whole bytes, so a 12-pixel row takes 2 bytes, not 1.5
3. On squared paper, shade in a 16×16 eye. You'll draw it for real in two lessons' time

---

## Common Issues

**Problem**: My pixel appears lower than I expected.
**Solution**: Remember that y counts down from the top.
**Why**: Screens are drawn from the top row to the bottom row, so row 0 is at the top.

**Problem**: Nothing is lit when I use `x = 16` on a 16-wide grid.
**Solution**: Use 0 to 15.
**Why**: Counting starts at 0, so the last column is the width minus one.

---

## Check your understanding

{% quiz %}
? What does 1-bit mean for a screen?
- [ ] It has one pixel
- [x] Each pixel is either on or off
- [ ] It can show one colour at a time out of many
> One bit holds a 1 or a 0, which is exactly enough for on or off.

? On a 128×64 screen, where is pixel (0, 63)?
- [ ] Top left
- [ ] Top right
- [x] Bottom left
> x is 0, so it's on the left edge. y is 63, the last row, so it's at the bottom.

? How many bytes does one 16×16 frame need?
- [ ] 16
- [x] 32
- [ ] 256
> 256 pixels divided by 8 pixels per byte is 32 bytes.
{% endquiz %}
