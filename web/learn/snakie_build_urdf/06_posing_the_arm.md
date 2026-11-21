---
layout: lesson
title: Posing the Arm
author: Kevin McAleer
type: page
cover: /learn/snakie_build_urdf/assets/banners/06_posing_the_arm.jpg
date: 2026-11-21
previous: 05_mass_balance_and_measuring.html
next: 07_saving_poses.html
description: Switch Snakie to Animate mode and pose the robot with joint sliders or
  by dragging it in 3-D
percent: 42
duration: 4
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

## From building to moving

So far you've been in **Assemble** mode. Click **Animate** at the left of the toolbar and the workspace changes.

![Animate mode with the Joints panel and motion tools](/learn/snakie_build_urdf/assets/animate.jpg){:class="img-fluid w-100"}

- A **Joints** panel opens on the right with a slider for each joint
- A **Poses** shelf and a **Timeline** appear along the bottom
- The toolbar swaps its building tools for **Mirror**, **Pin**, **Joints** and **Rest**

Nothing you do in Animate mode changes the robot's shape. You're only moving it.

---

## Sliders

Drag the **Shoulder** slider. The upper arm leans, and the forearm goes with it because it's further down the chain.

Each slider runs between the joint's limits, so the ends are -90 and 90 for our hinges. For an exact angle, type it into the box beside the slider and press **Enter**.

Try **Waist** `-40`, **Shoulder** `50`, **Elbow** `40`.

![The arm posed with the joint sliders](/learn/snakie_build_urdf/assets/posed.jpg){:class="img-fluid w-100"}

This is the moment to check your axes. If the shoulder twists when it should lean, go back to **Assemble**, edit the joint and change its **Axis**, as in lesson 4.

---

## Drag the robot

Sliders are exact. Dragging is quicker. In Animate mode, grab the tip of the forearm in the 3-D view and pull.

![Dragging the tip of the arm](/learn/snakie_build_urdf/assets/drag_pose.jpg){:class="img-fluid w-100"}

The whole chain bends to follow your mouse, and the sliders move by themselves to match. Snakie is working backwards from where you want the tip to be to the joint angles that put it there. Robot builders call that **inverse kinematics**, or **IK** for short.

Drag an empty part of the view and you orbit the camera instead, so aim for the robot.

---

## Rest, undo and redo

- **Rest** puts every joint back to zero, the pose you built the robot in
- The **undo** and **redo** arrows step back and forward through your pose changes. **Ctrl+Z** works too (**Cmd+Z** on a Mac)
- The **Joints** button hides and shows the slider panel when you want a bigger view

---

## Tools for bigger robots

Two buttons need more than one arm to be useful, so you'll meet them on a later robot:

- **Pin** keeps one part still, such as a foot, so the body moves when you bend that leg
- **Mirror** copies a pose from one side of a robot to the other. It needs joints named in left and right pairs, such as `hip_left` and `hip_right`

There are two more helpers. **Grab & pose** in the Assemble toolbar does the same dragging without leaving the building tools. **IK goal** in the bottom right lets you pick a part and drag a target for it to reach.

---

## Try it Yourself

1. Set the three joints to -40, 50 and 40 with the number boxes
2. Click **Rest**, then **undo** to get your pose back
3. Drag the tip of the forearm around and watch all three sliders move
4. Type `120` into the Elbow box. What does Snakie do with a number past the limit?

---

## Common Issues

**Problem**: Dragging spins the camera.
**Solution**: Start the drag on the robot itself, near the tip.
**Why**: Dragging the robot poses it. Dragging anywhere else orbits the view.

**Problem**: A slider won't go as far as I want.
**Solution**: Widen the joint's **Limits** in Assemble mode.
**Why**: Sliders stop at the joint's limits.

**Problem**: I've lost my view of the robot.
**Solution**: Click **Zoom to fit** in the bottom right.
**Why**: It's easy to orbit or zoom somewhere unhelpful.

---

## Check your understanding

{% quiz %}
? You move the shoulder slider. Why does the forearm move too?
- [ ] They share a servo
- [x] The forearm hangs from the upper arm in the chain
- [ ] It's a bug
> A joint carries everything further down the chain.

? What is inverse kinematics?
- [ ] Playing a motion backwards
- [x] Working out joint angles from where you want the tip to be
- [ ] Measuring the robot's mass
> You choose the target, and IK finds the angles.

? What does Rest do?
- [ ] Deletes the pose
- [ ] Saves the project
- [x] Puts every joint back to zero
> Rest returns the robot to the pose you built it in.
{% endquiz %}
