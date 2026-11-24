---
layout: lesson
title: Joint Tracks and Keyframes
author: Kevin McAleer
type: page
cover: /learn/snakie_build_urdf/assets/banners/09_joint_tracks_and_keyframes.jpg
date: 2026-11-21
previous: 08_building_a_motion.html
next: 10_binding_servos.html
description: Take fine control of a robot motion in Snakie with joint tracks, keyframes,
  easing and live pose controls
percent: 60
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

By the end of this lesson you will be able to use joint tracks, keyframes and easing for finer control of a motion.

## A second way to animate

The Timeline thinks in whole poses. The **Joint tracks** tab thinks in joints. Each joint gets its own row, and you place **keyframes** on it: marks that say "be at this angle at this time". Snakie fills in the movement between them.

It's the same idea used in film animation and game engines. It takes a little longer than dropping pose blocks, and it gives you exact control over timing. It's also the tab that writes code, which is why we need it.

Click **Joint tracks**.

---

## The controls

Along the top row:

- **Play** and **stop**, and a **loop** tick box
- The **scrubber**: a long slider for the current time. Drag it and the robot follows
- **dur**: the length of the whole motion in seconds
- An **easing** menu: `ease-in-out` or `linear`
- **fps**: how many steps per second the motion is cut into when it's exported

On the second row:

- **+ Keyframe** marks every joint's current angle at the current time
- **+ pose…** does the same using one of your saved poses
- **Duplicate** copies the keyframes at the current time a little later
- **Export .py** writes the code. That's lesson 12

---

## Build the motion with poses

We'll make a two-second motion: rest, reach, tuck, and back round.

1. Drag the scrubber to the far left, **0.00s**
2. Open **+ pose…** and choose `rest`
3. Drag the scrubber to the middle, **1.00s**
4. Open **+ pose…** and choose `reach`
5. Drag the scrubber to the far right, **2.00s**
6. Open **+ pose…** and choose `tuck`

Three columns of diamonds appear, one diamond for each joint at each time.

![Joint tracks with keyframes at the start, middle and end](/learn/snakie_build_urdf/assets/joint_tracks.jpg){:class="img-fluid w-100"}

Press play. The scrubber sweeps along and the arm moves through the three poses.

If you gave your parts a mass in lesson 5, there's a **balance** row above the joints. It marks moments in the motion where the weight shifts.

---

## Why per-joint keys are useful

Because every joint has its own row, they don't all have to move at once.

- Drag a diamond sideways to change **when** that joint arrives
- Put a keyframe on only one joint by posing just that joint, then pressing **+ Keyframe** at a new time
- Click a diamond to select it, then delete it to let the joint glide straight through

That's how you get natural movement. A real arm doesn't start every joint at the same instant. Let the shoulder lead and the elbow follow a moment later, and the same three poses suddenly look alive.

---

## Easing and frame rate

**Easing** is how speed changes during a move.

| Easing | Feels like | Good for |
|---|---|---|
| `ease-in-out` | Gentle start, gentle stop | Almost everything |
| `linear` | Constant speed, sudden stops | Conveyor belts, scanning sensors |
{:class="table table-single"}

**fps** only matters when you export. At 20 fps a two-second motion becomes 40 steps. More steps are smoother and make a longer program. 20 is plenty for hobby servos.

---

## Perform: mix poses with a slider

The third tab, **Perform**, makes live controls. Click **+ Control**, give it a name, click two or more poses in order and choose **Create control**.

![Making a live control from poses](/learn/snakie_build_urdf/assets/perform.jpg){:class="img-fluid w-100"}

You get a slider that blends from one pose to the next as you drag it. A control built from `tuck` and `reach` becomes a single "reach out" slider. It's a quick way to puppeteer a robot by hand.

---

## Try it Yourself

1. Build the three-pose motion on the joint tracks and play it
2. Switch the easing to `linear` and play it again. Which looks more like a real arm?
3. Drag the middle `elbow` diamond a little to the right so the elbow arrives late
4. Make a Perform control called `reach_out` from `tuck` and `reach`

---

## Common Issues

**Problem**: Both poses landed at the same time and one replaced the other.
**Solution**: Move the scrubber before adding each pose.
**Why**: Keyframes are placed at the scrubber's current time.

**Problem**: The Timeline and Joint tracks show different motions.
**Solution**: That's expected. Treat them as two separate motions.
**Why**: The Timeline stores a list of poses. Joint tracks store keyframes. Changing one doesn't rewrite the other.

**Problem**: **Export .py** is greyed out.
**Solution**: Bind a servo to a joint first. That's the next lesson.
**Why**: The exported code drives servos, so Snakie needs to know which servo moves which joint.

---

## Check your understanding

{% quiz %}
? What is a keyframe?
- [x] A mark that says a joint should be at an angle at a time
- [ ] A saved copy of the project
- [ ] The first frame of a video
> Keyframes pin a joint's angle at a moment. Snakie fills in the motion between them.

? Where does a new keyframe go?
- [ ] Always at the start
- [ ] Always at the end
- [x] At the scrubber's current time
> Move the scrubber first, then add the keyframe.

? What does fps change?
- [ ] How heavy the robot is
- [x] How many steps per second the exported motion has
- [ ] The joint limits
> More frames per second means smoother motion and a longer program.
{% endquiz %}
