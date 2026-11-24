---
layout: lesson
title: Frames and Animation
author: Kevin McAleer
type: page
cover: /learn/snakie_sprites/assets/banners/04_frames_and_animation.jpg
date: 2026-11-21
previous: 03_drawing_your_first_sprite.html
next: 05_saving_and_exporting.html
description: Add frames to make the eye blink and look left and right, using duplicate,
  onion skin and the filmstrip
percent: 35
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

By the end of this lesson you will be able to add frames to make the eye blink and look around, using duplicate, onion skin and the filmstrip.

## Five frames

One drawing is a picture. Several drawings shown in turn are an animation. Your eye needs five frames:

| Frame | Name | What it shows |
|---|---|---|
| 1 | open | The eye you've already drawn |
| 2 | half | The top half hidden, as the lid comes down |
| 3 | closed | A single line |
| 4 | left | Open, with the pupil moved left |
| 5 | right | Open, with the pupil moved right |
{:class="table table-single"}

Keep the order exactly as shown. Your code will ask for frames by number, so frame 3 needs to be the closed one.

---

## Frame 2: half closed

The quickest way to make a frame that's nearly the same as another is to copy it.

1. With frame 1 selected in the filmstrip, press **Duplicate**

![The first frame duplicated](/learn/snakie_sprites/assets/duplicated.jpg){:class="img-fluid w-100"}

2. Frame 2 is selected for you. Pick **Erase** and wipe out the top five rows of the eye, so the top edge is now flat and sits one row above the pupil
3. The new top edge has a hole in it where the pupil was. Pick **Draw** and fill in those four pixels, so the pupil is left as a 4×2 slot

![The half-closed frame](/learn/snakie_sprites/assets/half.jpg){:class="img-fluid w-100"}

It should look like this:

```
................
................
................
................
................
................
................
..############..
..####....####..
..####....####..
..############..
..############..
...##########...
....########....
................
................
```

Don't worry about matching it pixel for pixel. If it reads as a half-shut eye, it's right.

---

## Frame 3: closed

A closed eye is one line, so start from nothing.

1. Press **+** in the filmstrip to add a blank frame after the current one
2. Make sure **Onion skin** is on (the eye button at the bottom of the toolbar). You'll see a faint ghost of frame 2
3. With **Draw**, drag a line 12 pixels long across the middle, in row 8

![The closed frame with the previous frame showing through](/learn/snakie_sprites/assets/closed_onion.jpg){:class="img-fluid w-100"}

The ghost is only a guide. It isn't part of the frame and won't be saved.

---

## Frames 4 and 5: looking around

1. Click frame 1 and press **Duplicate**. The copy appears as frame 2, which is the wrong place
2. Press the **move later** arrow (next to Duplicate) until the copy is the last frame
3. Fill in the old pupil with **Draw**, then **Erase** a new 4×4 pupil three columns further left, so there's one lit column between the pupil and the edge of the eye
4. Duplicate that frame. In the copy, move the pupil to the right-hand side the same way

Here's a shortcut for step 4: duplicate the left-looking frame and press **Flip horizontally**. Done.

![All five frames in the filmstrip](/learn/snakie_sprites/assets/five_frames.jpg){:class="img-fluid w-100"}

Check the filmstrip reads open, half, closed, left, right.

---

## Play it

Set **FPS** to `6` and press **play**.

![The animation playing](/learn/snakie_sprites/assets/play_eye.jpg){:class="img-fluid w-100"}

It looks odd, doesn't it? The eye blinks, then darts left and right, over and over. That's because the editor plays the frames in order, and real eyes don't work to a fixed loop.

That's fine. These five frames are **ingredients**. In lesson 9 your code will choose which frame to show and when, and the eye will start to feel alive.

---

## How fast is a blink?

A real blink takes roughly a third of a second. With three frames in it (half, closed, half), each frame should last about a tenth of a second, which is 10 FPS. Anything from 6 to 12 looks natural.

For a pure loop, such as a spinning radar or a walking robot, FPS matters much more, because the editor's speed is the speed you'll get.

---

## Try it Yourself

1. Play the animation at 2, 6 and 20 FPS
2. Add a sixth frame: a wide-open, surprised eye with a smaller 2×2 pupil
3. Turn Onion skin off and on while stepping through the frames. Which do you prefer for drawing?
4. Use the move arrows to put your surprised frame last, so the first five keep their numbers

---

## Common Issues

**Problem**: My new frame appeared in the middle of the filmstrip.
**Solution**: Select it and use the move arrows to put it where it belongs.
**Why**: Duplicate and **+** both insert straight after the selected frame.

**Problem**: I can see faint pixels I didn't draw.
**Solution**: That's the onion skin. Turn it off with the eye button if it's distracting.
**Why**: Onion skin overlays the previous frame as a drawing guide.

**Problem**: I deleted the wrong frame.
**Solution**: Press Undo straight away.
**Why**: Undo covers frame changes as well as pixels.

---

## Check your understanding

{% quiz %}
? What's the fastest way to make a frame that's almost the same as the last one?
- [ ] Draw it again from scratch
- [x] Duplicate the frame and change the copy
- [ ] Use Import
> Duplicate copies every pixel, so you only change what moves.

? Why does frame order matter in this project?
- [ ] The editor refuses to save otherwise
- [x] The code will pick frames by their number
- [ ] Frames must be sorted by brightness
> Later you'll write names such as CLOSED = 2 that point at positions in this list.

? Where does a duplicated frame appear?
- [ ] At the end
- [ ] At the start
- [x] Straight after the frame you duplicated
> New frames go after the selected one. Use the move arrows to reposition them.
{% endquiz %}
