---
layout: lesson
title: Hinges, Wheels and Sliders
author: Kevin McAleer
type: page
cover: /learn/snakie_build_urdf/assets/banners/04_hinges_wheels_and_sliders.jpg
date: 2026-11-21
previous: 03_joining_blocks.html
next: 05_mass_balance_and_measuring.html
description: Pick the right joint type in Snakie and set its axis and limits so the
  model moves like the real robot
percent: 30
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

By the end of this lesson you will be able to pick the right joint type for a moving part and set its axis and limits.

## Four ways to connect two parts

Every block except the base has exactly one joint: the one that connects it to its parent. Right-click a block and choose **Edit joint…** to open it. You can also click the small joint icon beside the block's name.

![The elbow joint open for editing](/learn/snakie_build_urdf/assets/joint_edit.jpg){:class="img-fluid w-100"}

The four buttons under **Joint** are the four kinds.

| Kind | Moves how | Real-world part | URDF name |
|---|---|---|---|
| **Fixed** | Not at all | A bracket, a sensor mount | `fixed` |
| **Hinge** | Turns between two limits | A standard hobby servo | `revolute` |
| **Wheel** | Turns round and round | A wheel on a motor | `continuous` |
| **Slider** | Slides along a line | A lift, a gripper jaw | `prismatic` |
{:class="table table-single"}

The joint tool in the last lesson used slightly different words. **Static** there is **Fixed** here, **Rotation** is **Hinge** and **Linear** is **Slider**.

---

## Axis

A hinge turns around a line, and a slider slides along one. That line is the **axis**, and it's one of **X**, **Y** or **Z**.

The axis is measured from the joint's own point of view, which is why the quickest way to choose is to try it. Pick one, pose the joint (next lesson), and if the part moves the wrong way, come back and pick another.

Our arm uses:

| Joint | Kind | Axis | Movement |
|---|---|---|---|
| `waist` | Hinge | Z | The whole arm turns left and right |
| `shoulder` | Hinge | Y | The upper arm leans forward and back |
| `elbow` | Hinge | Y | The forearm bends |
{:class="table table-single"}

---

## Limits

A hobby servo can't turn forever. Most manage about 180 degrees. **Limits** tell Snakie how far the joint may go, so the model can't do anything the real robot can't.

New hinges get **-90** to **90** degrees: a quarter turn either way from where you built it. That matches a standard servo nicely, so leave the arm's three joints alone.

Limits do three jobs later in the course:

- They set the ends of each joint's slider when you pose the robot
- They stop a dragged pose from bending a joint backwards
- They set how joint angles are converted to servo angles

A **Wheel** has no limits at all. A **Slider** has limits in millimetres.

![A joint switched to Slider, with limits in millimetres](/learn/snakie_build_urdf/assets/joint_slider.jpg){:class="img-fluid w-100"}

When you switch a hinge to a slider, check the limits. Snakie carries the old numbers across and they come out very large, so type the travel you really want, such as `0` to `30`.

---

## The other boxes

- **Attaches to** changes the block's parent. The block stays where it is on screen. Only its place in the chain changes
- **Offset** moves the joint away from the parent's centre, in millimetres
- **Roll** turns the block on the spot, around the line through the joint
- **Copies** makes this joint follow another one. Two gripper fingers that open together are the classic use: one finger copies the other
- **Delete** removes the joint and leaves the block loose

---

## Rest position

The pose you build the robot in is its **rest position**. Every joint reads zero there. A shoulder limit of -90 to 90 means a quarter turn either side of how you built it.

That's worth a moment's thought on a real robot. Build the model in the pose where your servos sit at the middle of their travel, and both will have the same room to move each way.

---

## Try it Yourself

1. Open the `elbow` joint and switch it to **Wheel**. Notice the limits vanish
2. Switch to **Slider** and read the limits. Then click **Cancel** so nothing is saved
3. Open the `waist` joint and check it says **Hinge**, axis **Z**, limits -90 to 90
4. Open `robot.urdf` in the Code workspace and find the `shoulder` joint. Its limits are there too, in radians: 1.5708 is 90 degrees

---

## Common Issues

**Problem**: My joint turns the wrong way round.
**Solution**: Pick a different **Axis**.
**Why**: The axis depends on how the block was snapped on, so the right letter isn't always the one you'd guess.

**Problem**: The joint only moves a tiny bit.
**Solution**: Check the **Limits**. For a hinge they're degrees. For a slider they're millimetres.
**Why**: Switching kinds keeps the old numbers, which may not suit the new kind.

**Problem**: A part has come loose from the robot.
**Solution**: Open its properties and choose its parent under **Attaches to**, or use the joint tool again.
**Why**: Deleting a joint leaves the block with nothing holding it.

---

## Check your understanding

{% quiz %}
? Which joint kind suits a standard hobby servo?
- [ ] Fixed
- [x] Hinge
- [ ] Wheel
> A hobby servo turns between two limits, which is exactly what a hinge does.

? What are a slider's limits measured in?
- [ ] Degrees
- [x] Millimetres
- [ ] Seconds
> A slider moves along a line, so its limits are distances.

? What does every joint read in the rest position?
- [x] Zero
- [ ] 90
- [ ] Its maximum limit
> The pose you build in is zero for every joint. Limits are measured from there.
{% endquiz %}
