---
layout: lesson
title: Building a Motion
author: Kevin McAleer
type: page
cover: /learn/snakie_build_urdf/assets/banners/08_building_a_motion.jpg
date: 2026-11-21
previous: 07_saving_poses.html
next: 09_joint_tracks_and_keyframes.html
description: String poses into a looping motion on Snakie's timeline and preview it
  with play, loop and onion skin
percent: 54
duration: 5
quiz_total: 46
date_updated: 2026-11-24
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

By the end of this lesson you will be able to string poses into a looping motion on the timeline and preview it with play, loop and onion skin.

## Poses in a row

A motion is poses played one after another, with the robot gliding between them. The **Timeline** tab at the bottom of Animate mode is where you line them up.

---

## Add poses to the timeline

1. Click the `rest` card so the arm is in that pose
2. Click **+ Add pose**, or press **K**
3. Click `reach`, then **+ Add pose**
4. Click `tuck`, then **+ Add pose**
5. Click `reach`, then **+ Add pose** again

Each press adds a block to the timeline. You should have four: rest, reach, tuck, reach.

![Four pose blocks on the timeline](/learn/snakie_build_urdf/assets/timeline.jpg){:class="img-fluid w-100"}

---

## Reading the timeline

- Each **block** is a pose, with the time it's reached written underneath
- The number between two blocks is how long the move takes. New moves take **0.5s**. Click the number to type a new time, or drag the small handle beside it
- The little curve under that number is the **easing**. The default is smooth: it starts gently, speeds up and slows to a stop, which is how real arms like to move. Click the curve to switch to a steady, constant speed
- With **Loop** on, a final faint link is added, labelled **back to start**
- The clock beside the play button shows where you are and the total length

Click a block to select it. The **←** and **→** buttons then move it earlier or later, and **✕** takes it off the timeline. The saved pose stays on the shelf.

---

## Play it

Press the **play** button or the **Space** bar. The arm runs through the blocks and, with **Loop** on, goes round again until you stop it.

![The motion playing](/learn/snakie_build_urdf/assets/timeline_playing.jpg){:class="img-fluid w-100"}

The joint sliders on the right move as it plays. They're showing the same angles your servos will be sent later.

---

## Onion skin

Click **Onion skin**, then click a block. Snakie draws faint ghosts of the pose before and the pose after.

![Onion skin showing the neighbouring poses as ghosts](/learn/snakie_build_urdf/assets/onion_skin.jpg){:class="img-fluid w-100"}

Animators use this to judge how big each move is. A huge jump between two ghosts means a fast, jerky move. Either give it more time or add a pose in between.

---

## The rest of the buttons

| Button | What it does |
|---|---|
| **Make walk cycle** | Builds a walking loop for robots with left and right legs |
| **Live** | Sends the motion to real servos as it plays. Needs a connected board |
| **Export** | Writes MicroPython for the motion. Lesson 12 |
| **Render movie** | Makes a video. Lesson 14 |
{:class="table table-single"}

Your motion is saved in `robot.yml` under `sequences`, as a list of pose names, each with a duration and an easing.

---

## Try it Yourself

1. Build the four-block motion and play it
2. Change the move from `reach` to `tuck` to take `1.5` seconds and play it again
3. Add your `wave` pose from the last lesson to the end
4. Turn on **Onion skin** and find the biggest jump in your motion

---

## Common Issues

**Problem**: **+ Add pose** added the wrong pose.
**Solution**: Click the pose card first and wait for the arm to arrive, then add.
**Why**: Add pose uses whatever the robot is doing right now.

**Problem**: The motion snaps instead of gliding.
**Solution**: Give the move more time, or split it with an extra pose.
**Why**: A big change in half a second is a fast move.

**Problem**: I removed a block and now I'm worried I've lost the pose.
**Solution**: Look on the Poses shelf. It's still there.
**Why**: The timeline only holds references to poses. Removing a block never deletes the pose itself.

---

## Check your understanding

{% quiz %}
? What does + Add pose put on the timeline?
- [ ] The first saved pose
- [x] The pose the robot is in right now
- [ ] An empty block
> It captures the current pose, so pick your pose first.

? What does easing change?
- [ ] The colour of the block
- [ ] Which servo is used
- [x] How the speed builds up and slows down during a move
> Easing makes moves start and stop gently instead of jerking.

? Onion skin shows a huge gap between two ghosts. What does that tell you?
- [x] That move will be fast and may be jerky
- [ ] The robot is too heavy
- [ ] The pose is not saved
> Big gaps in a short time mean fast moves.
{% endquiz %}
