---
layout: lesson
title: Saving Poses
author: Kevin McAleer
type: page
cover: /learn/snakie_build_urdf/assets/banners/07_saving_poses.jpg
date: 2026-11-21
previous: 06_posing_the_arm.html
next: 08_building_a_motion.html
description: Save named poses for your robot in Snakie, then rename, update and reuse
  them
percent: 48
duration: 4
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

By the end of this lesson you will be able to save named poses, then rename, update and reuse them.

## A pose is a snapshot

A **pose** is the angle of every joint at one moment, saved under a name. Poses are the building blocks of everything that follows: motions are made from them, and so is the code Snakie writes for you.

We'll save three.

| Pose | Waist | Shoulder | Elbow |
|---|---|---|---|
| `rest` | 0 | 0 | 0 |
| `reach` | -40 | 60 | 30 |
| `tuck` | -40 | -30 | -80 |
{:class="table table-single"}

---

## Save a pose

1. Click **Rest** so every joint is at zero
2. Click **Save pose** at the right of the Poses shelf, or press **S**

A card appears on the shelf with a little picture of the robot and the name **Pose 1**. Double-click the name, type `rest` and press **Enter**.

Now set the joints for `reach` with the number boxes, save, and rename. Do the same for `tuck`.

![Three saved poses on the shelf](/learn/snakie_build_urdf/assets/poses.jpg){:class="img-fluid w-100"}

Click any card's picture and the robot glides to that pose. The poses are also listed under **Poses** in the Build panel on the left.

---

## Changing a pose

Each card has a **⋯** button with a small menu.

| Choice | What it does |
|---|---|
| **Duplicate** | Makes a copy to adjust |
| **Update from current** | Replaces the saved angles with what's on screen now |
| **Add to timeline at playhead** | Drops the pose into a motion. That's the next lesson |
| **Delete** | Removes the pose |
| **Mirror to a new pose** | Makes a left-right flipped copy, on robots with left and right joints |
{:class="table table-single"}

**Update from current** is the one you'll use most. Click a pose, nudge a joint until it looks right, then update.

---

## Poses for part of a robot

**+ New pose** in the Build panel opens a different panel, with a tick box beside each joint.

![The New pose panel with a tick box per joint](/learn/snakie_build_urdf/assets/new_pose.jpg){:class="img-fluid w-100"}

Untick a joint and the pose leaves it alone. That's how you make a pose that only opens a gripper, or only turns a head, whatever the rest of the robot is doing. For our three-joint arm, keep them all ticked.

---

## Where poses live

Open `robot.yml` in the Code workspace and you'll find them, in plain text:

```yaml
  poses:
    - name: rest
      values:
        waist: 0
        shoulder: 0
        elbow: 0
    - name: reach
      values:
        waist: -40
        shoulder: 60
        elbow: 30
```

The values are joint angles in degrees. Because it's a text file, poses travel with your project and work nicely with Git.

> **Teachers**: Poses and motions are saved as plain text in `robot.yml` inside the project folder, so copying or collecting the folder collects the learner's work. Make sure learners save into a folder they can get back to on a shared computer.

---

## Try it Yourself

1. Save `rest`, `reach` and `tuck`
2. Click between the three cards and watch the arm glide
3. Click `reach`, change the elbow to 45, and use **Update from current**. Then change it back to 30 and update again
4. Make a fourth pose of your own and call it `wave`

---

## Common Issues

**Problem**: The picture on a card doesn't match the pose.
**Solution**: Click the card, wait for the arm to stop, then choose **Update from current**.
**Why**: The picture is taken at the moment you save. If the arm was still gliding, it caught it mid-move.

**Problem**: Clicking the name doesn't move the robot.
**Solution**: Click the picture on the card.
**Why**: Double-clicking the name is for renaming.

**Problem**: My poses have gone after reopening the project.
**Solution**: Check `robot.yml` is still in the project folder.
**Why**: Poses are stored in that file, not inside Snakie.

---

## Check your understanding

{% quiz %}
? What does a pose store?
- [ ] A video of the robot
- [x] The angle of each joint, under a name
- [ ] The robot's shape
> A pose is a named set of joint angles.

? You've improved a pose on screen. How do you keep the change?
- [ ] Delete it and start again
- [ ] Rename it
- [x] Use Update from current
> Update from current replaces the saved angles with the ones on screen.

? Which file holds your poses?
- [ ] robot.urdf
- [x] robot.yml
- [ ] skeleton.json
> The URDF is the robot's shape. robot.yml holds poses, motions and servos.
{% endquiz %}
