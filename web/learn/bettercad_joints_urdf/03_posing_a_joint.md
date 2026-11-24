---
layout: lesson
title: Posing a Joint
author: Kevin McAleer
type: page
cover: /learn/bettercad_joints_urdf/assets/banners/03_posing_a_joint.jpg
date: 2026-11-21
previous: 02_your_first_joint.html
next: 04_a_second_joint.html
description: Move a BetterCad joint with its slider, by typing an angle or by dragging
  the part, and go back to the home pose
percent: 28
duration: 4
quiz_total: 42
date_updated: 2026-11-24
navigation:
- name: Make It Move with BetterCad
- content:
  - section: Overview
    content:
    - name: Make It Move with BetterCad
      link: 00_intro.html
  - section: Joints
    content:
    - name: Components and the Assemble Tab
      link: 01_components_and_the_assemble_tab.html
    - name: Your First Joint
      link: 02_your_first_joint.html
    - name: Posing a Joint
      link: 03_posing_a_joint.html
    - name: A Second Joint
      link: 04_a_second_joint.html
    - name: Joint Kinds and Line-up
      link: 05_joint_kinds_and_line_up.html
  - section: Poses
    content:
    - name: Named Poses
      link: 06_named_poses.html
    - name: Drag the Arm by Its Tip
      link: 07_drag_the_arm_by_its_tip.html
  - section: Checks
    content:
    - name: Mass and Balance
      link: 08_mass_and_balance.html
    - name: Checking for Clashes
      link: 09_checking_for_clashes.html
    - name: Sweeps and Joint Limits
      link: 10_sweeps_and_joint_limits.html
  - section: Drives
    content:
    - name: Drive Links
      link: 11_drive_links.html
  - section: Export
    content:
    - name: Exporting URDF
      link: 12_exporting_urdf.html
    - name: Summary and Next Steps
      link: 13_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to move a joint with its slider, by typing an angle or by dragging the part, and go back to the home pose.

The shoulder is made. Now make it move. There are three ways, and you'll end up using all of them.

---

## The Pose section

1. Press **Esc** so nothing is selected.

Properties shows **This model**, and it has a new section called **Pose** with one row for each joint: "Shoulder, hinge", a slider and an **Angle** box.

2. Drag the slider.

The arm swings as you drag. The slider stops at the joint's limits, which are still -90 and 90 degrees.

3. Click in the **Angle** box, type `45` and press Tab.

![The shoulder posed at 45 degrees](/learn/bettercad_joints_urdf/assets/pose_45.jpg){:class="img-fluid w-100"}

Typing is the way to get an exact angle. Like every number box in BetterCad, it takes sums too.

---

## Dragging the part

4. In the 3D view, press on the upper arm and drag.

The arm follows the pointer round its hinge. This is the quickest way to try something out: grab it and move it.

![Dragging the upper arm round its shoulder joint](/learn/bettercad_joints_urdf/assets/drag_pose.jpg){:class="img-fluid w-100"}

Dragging an empty part of the view still turns the camera, as it always has.

---

## Home

Every joint has a **home**: the position it was in when you made it. For the shoulder, that's the arm standing straight up.

5. Click **Home** under the sliders.

Every joint goes back to zero. The Browser has a **Poses** folder with a pose called **Home** in it, and a small "Now" beside it whenever the model is in that pose.

Home matters more than it looks:

- Angles are measured from it. 45 degrees means 45 degrees from home
- A URDF export describes the robot at home
- An STL export puts every joint at home, unless you tick **Export as posed**

So it pays to make each joint with the parts in a sensible resting position. Straight up is a good home for an arm.

---

## A pose isn't a step

Look at the timeline while you drag the slider. No new steps appear. Posing doesn't change the design, in the same way that opening a door doesn't change the house.

The pose is still remembered. **Ctrl+Z** takes back the last move, and the model saves in whatever pose you left it, so it opens that way next time.

---

## From a selected part

There's one more place to pose from.

6. Click the upper arm.

Properties shows the part as usual, with a new **Joint** section: the joint's name, "Hinge joint, on the base", an **Edit** button, and the same slider and Angle box. That's handy when a model has a dozen joints and you just want the one this part turns on.

---

## Try it Yourself

1. Pose the shoulder at -30 degrees by typing. Which way is negative?
2. Type `90/4` in the Angle box. What angle do you get?
3. Drag the arm as far as it will go each way. It stops at the limits, wherever the pointer goes.

---

## Common Issues

- **Problem**: The Home button is greyed out
- **Solution**: Nothing to do
- **Why**: Every joint is already at home

- **Problem**: The Pose section isn't in Properties
- **Solution**: Press Esc to clear the selection
- **Why**: The sliders for every joint show under This model. With a part selected you see only that part's own joint

---

## Check your understanding

{% quiz %}
? What is a joint's home?
- [ ] Its lower limit
- [x] The position it was in when the joint was made
- [ ] The middle of the bed
> Angles are measured from home, and exports use it unless you say otherwise.

? You posed the arm. How many steps did that add to the timeline?
- [ ] One for each joint
- [x] None
- [ ] One
> Posing shows the design in a different position. It doesn't change the design.

? Which way of posing gives an exact angle?
- [ ] Dragging the part
- [ ] Dragging the slider
- [x] Typing in the Angle box
> Dragging is quick, typing is exact.
{% endquiz %}
