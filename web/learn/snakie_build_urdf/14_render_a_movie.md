---
layout: lesson
title: Render a Movie
author: Kevin McAleer
type: page
cover: /learn/snakie_build_urdf/assets/banners/14_render_a_movie.jpg
date: 2026-11-21
previous: 13_urdf_in_and_out.html
next: 15_summary.html
description: Turn your robot animation into a video or GIF with the Render movie tool
  in Snakie, ready to share
percent: 90
duration: 5
quiz_total: 46
date_updated: 2026-11-21
navigation:
- name: Build and Animate a Robot in Snakie
- content:
  - section: Overview
    content:
    - name: Build and Animate a Robot in Snakie
      link: 00_intro.html
    - name: The Build Workspace
      link: 01_the_build_workspace.html
  - section: Building the Arm
    content:
    - name: Your First Block
      link: 02_your_first_block.html
    - name: Joining Blocks
      link: 03_joining_blocks.html
    - name: Hinges, Wheels and Sliders
      link: 04_hinges_wheels_and_sliders.html
    - name: Mass, Balance and Measuring
      link: 05_mass_balance_and_measuring.html
  - section: Making It Move
    content:
    - name: Posing the Arm
      link: 06_posing_the_arm.html
    - name: Saving Poses
      link: 07_saving_poses.html
    - name: Building a Motion
      link: 08_building_a_motion.html
    - name: Joint Tracks and Keyframes
      link: 09_joint_tracks_and_keyframes.html
  - section: Servos and Code
    content:
    - name: Binding Servos
      link: 10_binding_servos.html
    - name: Driving the Arm from Code
      link: 11_driving_the_arm_from_code.html
    - name: Exporting a Motion to MicroPython
      link: 12_exporting_a_motion_to_micropython.html
  - section: Sharing
    content:
    - name: URDF In and Out
      link: 13_urdf_in_and_out.html
    - name: Render a Movie
      link: 14_render_a_movie.html
    - name: Summary and Next Steps
      link: 15_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

## Show it off

You've built a robot and taught it to move. Before any plastic is printed, you can already show people what it will do.

**Render movie** plays your motion and records it as a video file. It all happens on your own computer.

---

## Make your first movie

1. Open your desk arm project and switch to **Build**, then **Animate**
2. Open the **Timeline** tab and check the motion from lesson 8 is there. Press play once to be sure
3. Turn the view to an angle you like
4. Click **Render movie**

![The Render movie settings](/learn/snakie_build_urdf/assets/render.jpg){:class="img-fluid w-100"}

5. Leave the settings alone for now and click **Quick draft**

A draft is small and fast. Use it to check the framing and the timing before you spend time on a good one.

---

## The settings

| Setting | Choices | Use it for |
|---|---|---|
| Size | 720p, 1080p, 4K, Square, Vertical | Where the movie will be watched |
| Frame rate | 24, 30, 60 | How smooth it looks |
| Loops | A number | Playing the motion more than once |
| Camera | Current view, Turntable | A still camera, or one that circles the robot |
| Look | Studio light, Studio dark, Grid | The background |
| Quality | Draft, Good, Best | Speed against polish |
| Format | MP4, WebM, GIF, PNG frames | The kind of file you get |
{:class="table table-single"}

There's also a switch for a small "Made with snakie.org" mark. The top of the window shows how long the movie will be, and the bottom shows roughly how big the file will be.

Some good starting points:

| You want | Try |
|---|---|
| A clip for a video edit | 1080p, 30, MP4, Good |
| A short for a phone | Vertical, 30, MP4 |
| Something for a forum post or a README | 720p or Square, GIF, 2 loops |
| Frames to edit yourself | PNG frames |
{:class="table table-single"}

---

## Current view or Turntable

**Current view** keeps the camera exactly where you left it. It's the best choice when the motion itself is the star, such as a wave or a walk.

**Turntable** slowly circles the robot while the motion plays. It's the best choice for showing the design from every side.

**Loops** sets how many times the motion repeats. It starts at 3, which gives a turntable time to travel round.

---

## Tips for a better movie

- **Frame it first.** With Current view, what you see is what you get. Leave room for the arm at full reach
- **Start and end on the same pose.** Turn on **Loop** in the Timeline so the motion returns to the start. A looping GIF then plays without a jump
- **Draft, then render.** Quality **Best** at 4K takes a while. Get everything right in a draft first
- **Keep GIFs short and small.** GIF files grow quickly. A few seconds at 720p is plenty

---

## Try it Yourself

1. Render a **Quick draft** of your motion from the current view
2. Render it again with **Turntable** and 2 loops
3. Make a square GIF with the **Studio dark** look
4. Open the BetterCad desk arm from the last lesson, give it a two-pose motion and render that

---

## Common Issues

**Problem**: The movie shows the arm standing still.
**Solution**: Check the motion plays in the Timeline before you render.
**Why**: Render movie records the motion you've built. With nothing to play, there's nothing to move.

**Problem**: Part of the arm goes out of frame.
**Solution**: Zoom out a little, or use Turntable, and render a draft to check.
**Why**: Current view records exactly what the camera sees, and the arm at full reach is bigger than the arm at rest.

**Problem**: The render is slow.
**Solution**: Lower the size or the quality, or use Quick draft while you're still making changes.
**Why**: Every frame is drawn on your computer. 4K has nine times as many pixels as 720p.

---

## Check your understanding

{% quiz %}
? Where is the movie rendered?
- [x] On your own computer
- [ ] On the Pico
- [ ] On a paid cloud service
> Snakie renders on your computer. Nothing is uploaded and there are no credits to buy.

? Which camera setting circles the robot while the motion plays?
- [ ] Current view
- [x] Turntable
- [ ] Grid
> Turntable orbits the robot. Current view holds the camera still.

? What makes a GIF loop without a visible jump?
- [ ] A higher frame rate
- [ ] The Vertical size
- [x] A motion that ends on the pose it started from
> If the last frame matches the first, the loop is seamless.
{% endquiz %}
