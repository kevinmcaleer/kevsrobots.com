---
layout: lesson
title: Joining Blocks
author: Kevin McAleer
type: page
cover: /learn/snakie_build_urdf/assets/banners/03_joining_blocks.jpg
date: 2026-11-21
previous: 02_your_first_block.html
next: 04_hinges_wheels_and_sliders.html
description: Use Snakie's joint tool to snap one block onto another and choose how
  the joint moves
percent: 24
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

## A better way to place parts

In the last lesson you placed the tower by working out an offset. That's fine for one block. For an arm, where each piece hangs off the end of the last, there's a faster tool: **Add a joint**. You click a spot on one block, then a spot on another, and Snakie brings them together.

---

## Make the upper arm

1. Click **Add a box**
2. Set **L** `80`, **W** `16`, **H** `8`
3. Pick an orange colour
4. Name the joint `shoulder`
5. Set **Offset Z** to `100` so the block floats above the tower where you can reach it
6. Click **OK**, then rename the block to `upper_arm`

![The upper arm floating above the tower](/learn/snakie_build_urdf/assets/arm_floating.jpg){:class="img-fluid w-100"}

It's attached to the base for now, and it's the wrong way up for an arm. The joint tool fixes both.

---

## Snap it on

1. Click **Add a joint** in the toolbar. The **Add Joint** panel opens
2. Click the **top face of the tower**. That's the parent, the part that stays put
3. Click one **end face of the upper arm**. That's the child, the part that moves

As you move over a block, Snakie highlights the nearest snap point: a face centre, an edge or a corner. When both are picked, the arm jumps into place for a preview, standing on its end on top of the tower.

![The Add Joint panel with the arm previewed on the tower](/learn/snakie_build_urdf/assets/add_joint.jpg){:class="img-fluid w-100"}

The panel now shows both parts, each with the word **centre** because you picked the centres of two faces. If you clicked them the wrong way round, **swap** flips parent and child.

---

## Choose how it moves

Under **Joint type** there are three choices.

| Type | What it does | Example |
|---|---|---|
| **Static** | Glued in place | A sensor bolted to a chassis |
| **Rotation** | Turns about an axis, between two limits | An elbow |
| **Linear** | Slides along a line | A drawer or a lift |
{:class="table table-single"}

Choose **Rotation** and click **Add**.

![The upper arm joined to the tower](/learn/snakie_build_urdf/assets/joint_added.jpg){:class="img-fluid w-100"}

The **Offset** and **roll** boxes in the panel nudge the child after it has snapped. Leave them at zero today.

---

## Point the hinge the right way

A joint made with this tool turns around the line that goes straight through the two faces you clicked, like a wheel on an axle. For our arm that would spin the upper arm on the spot. A shoulder needs to swing over instead.

1. Right-click `upper_arm` in the Components list and choose **Edit joint "shoulder"…**
2. Next to **Axis**, click **Y**
3. Click **OK**

![Editing the shoulder joint's axis](/learn/snakie_build_urdf/assets/joint_axis.jpg){:class="img-fluid w-100"}

You'll see it swing in lesson 6. If a joint ever turns the wrong way, this is the box to change.

---

## Add the forearm

Same steps again.

1. **Add a box**: **L** `60`, **W** `12`, **H** `8`, a green colour, joint name `elbow`, **Offset Z** `200`. Rename it `forearm`
2. **Add a joint**: click the **top end of the upper arm**, then an **end face of the forearm**
3. Choose **Rotation** and click **Add**
4. Edit the joint and set **Axis** to **Y**

![The finished arm with base, tower, upper arm and forearm](/learn/snakie_build_urdf/assets/arm_built.jpg){:class="img-fluid w-100"}

Look at the Components list. Each part is indented under the part it hangs from: base, tower, upper arm, forearm. That's called the **chain**, and it's why moving the shoulder will carry the forearm along with it.

---

## Try it Yourself

1. Build the upper arm and forearm
2. Check the status chip in the bottom left. It should read **3 joints · 4 links**
3. Click each part in the Components list. Snakie selects it and zooms to it
4. Hover over the small icon beside each name. It tells you the joint type

---

## Common Issues

**Problem**: I can't click the block I want because it's inside another one.
**Solution**: Give it an **Offset Z** big enough to float clear before using the joint tool.
**Why**: You can only click what you can see.

**Problem**: The arm snapped on sideways.
**Solution**: Click **Cancel** and pick again, aiming at the middle of the small end face.
**Why**: The joint lines up the two faces you click, so clicking a long side lays the arm flat against the tower.

**Problem**: Parent and child are the wrong way round.
**Solution**: Click **swap** before **Add**.
**Why**: The first click is always the part that stays put.

---

## Check your understanding

{% quiz %}
? In the joint tool, which part do you click first?
- [x] The parent, which stays put
- [ ] The child, which moves
- [ ] It doesn't matter
> Parent first, then child. Swap flips them if you slip.

? Which joint type is glued in place?
- [ ] Rotation
- [ ] Linear
- [x] Static
> Static joints don't move. They just hold two parts together.

? The shoulder spins on the spot when it should swing over. What do you change?
- [ ] The block's colour
- [x] The joint's Axis
- [ ] The block's mass
> The axis is the line the joint turns around.
{% endquiz %}
