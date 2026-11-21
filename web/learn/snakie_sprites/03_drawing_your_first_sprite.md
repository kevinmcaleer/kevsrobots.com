---
layout: lesson
title: Drawing Your First Sprite
author: Kevin McAleer
type: page
cover: /learn/snakie_sprites/assets/banners/03_drawing_your_first_sprite.jpg
date: 2026-11-21
previous: 02_the_sprite_editor.html
next: 04_frames_and_animation.html
description: Draw a 16 by 16 robot eye in the Snakie Sprite editor using the draw,
  fill and erase tools
percent: 28
duration: 5
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

## The plan

You're going to draw one robot eye: a rounded square with a square pupil in the middle. It's 16 pixels wide and 16 tall, with a 2-pixel empty border so two eyes can sit side by side without touching.

Here's the finished frame, so you know where you're heading:

```
................
................
....########....
...##########...
..############..
..############..
..####....####..
..####....####..
..####....####..
..####....####..
..############..
..############..
...##########...
....########....
................
................
```

---

## Start a blank sprite

1. Open the Sprite editor (Display instrument, **SPRITES** key)
2. Press **New**. Snakie asks whether to clear the current drawing. Press OK
3. In the **SIZE** menu choose **16×16**
4. Click in the **NAME** box and type `eye`

![A blank 16 by 16 sprite named eye](/learn/snakie_sprites/assets/blank.jpg){:class="img-fluid w-100"}

The name matters. Later it becomes `eye.spr` and `eye.py`, and you'll type `import eye` in your code. Keep it short, lower case and free of spaces.

---

## Draw the outline

Pick the **Draw** tool (the pencil at the top of the toolbar). Now trace the outside edge of the eye:

1. Count two rows down from the top and four pixels in from the left. Drag right for eight pixels. That's the top edge
2. Step down and out one pixel on each side to round the corners
3. Carry on down both sides, round the bottom corners the same way, and close the shape along the bottom

![The outline of the eye](/learn/snakie_sprites/assets/outline.jpg){:class="img-fluid w-100"}

Make sure the outline has **no gaps**. The next step depends on it.

If a pixel lands in the wrong place, click it again to turn it off, or use **Undo**.

---

## Fill it in

Pick the **Fill** tool (third in the toolbar) and click anywhere inside the outline.

![The filled eye](/learn/snakie_sprites/assets/filled.jpg){:class="img-fluid w-100"}

Fill spreads out from the pixel you click until it reaches a lit pixel or the edge of the sprite. If your whole canvas lights up, there was a gap in the outline. Press Undo, close the gap and try again.

---

## Cut out the pupil

Pick the **Erase** tool (second in the toolbar). Turn off a 4×4 block right in the centre: columns 6 to 9, rows 6 to 9, counting from 0.

![The finished open eye](/learn/snakie_sprites/assets/eye_open.jpg){:class="img-fluid w-100"}

That's frame 1 done. Switch back to the Draw tool so you don't erase anything by accident.

---

## A quick experiment

Press **Invert**. Every pixel swaps.

![The eye inverted](/learn/snakie_sprites/assets/inverted.jpg){:class="img-fluid w-100"}

Inverting is handy when a sprite will sit on a lit background. For this course we want the original, so press **Undo**.

---

## Why leave a border?

You could draw right up to the edges. A border is useful for two reasons:

- Two sprites placed edge to edge still have a gap between them
- There's room for the eye to shift a pixel or two in later frames without being cut off

If you ever need to move a whole drawing, the four **Nudge** arrows slide it one pixel at a time.

---

## Try it Yourself

1. Nudge the eye one pixel left, then back
2. Flip the eye left to right. Nothing seems to happen. Why not?
3. Give the pupil a one-pixel highlight in its top-left corner with the Draw tool. Decide whether you like it. Undo it if you don't
4. Start a second sprite later, once this one is saved, and try a round eye or a heart

---

## Common Issues

**Problem**: Fill lights up the whole canvas.
**Solution**: Undo, find the gap in the outline and close it.
**Why**: Fill spreads through every connected unlit pixel. Pixels that only touch at a corner still count as a closed line, but a missing pixel lets the fill escape.

**Problem**: I keep turning pixels off when I mean to paint.
**Solution**: Drag instead of clicking, or check you haven't left the Erase tool selected.
**Why**: A single click with Draw flips a pixel, so clicking a lit pixel turns it off.

**Problem**: My eye is off-centre.
**Solution**: Use the Nudge arrows until there are two empty columns on each side and two empty rows top and bottom.
**Why**: Later lessons place two eyes using their 16×16 boxes, so a centred drawing keeps the face symmetrical.

---

## Check your understanding

{% quiz %}
? Why must the outline have no gaps before you use Fill?
- [ ] Fill only works on squares
- [x] Fill spreads through any gap and floods the rest of the canvas
- [ ] Gaps make the file bigger
> Fill keeps spreading until it meets lit pixels or the edge of the sprite.

? What is the sprite's name used for?
- [ ] Nothing, it's only a label
- [x] It becomes the file name, and the module name you import
- [ ] It's shown on the OLED
> A sprite named eye saves as eye.spr and exports as eye.py.

? What does a single click with the Draw tool do to a lit pixel?
- [x] Turns it off
- [ ] Nothing
- [ ] Makes it brighter
> A click flips the pixel. Dragging paints pixels on.
{% endquiz %}
