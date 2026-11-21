---
layout: lesson
title: Mass, Balance and Measuring
author: Kevin McAleer
type: page
cover: /learn/snakie_build_urdf/assets/banners/05_mass_balance_and_measuring.jpg
date: 2026-11-21
previous: 04_hinges_wheels_and_sliders.html
next: 06_posing_the_arm.html
description: Give each part a mass in Snakie, find the robot's centre of mass and
  check sizes with the measure, skeleton and exploded views
percent: 36
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

## Will it fall over?

An arm that reaches out too far tips its base. It's much nicer to find that out on screen than on your desk. Snakie can work out where your robot's weight sits, as long as it knows how heavy each part is.

Under the Components list there's a line that says **mass known for 0 of 4 parts**. Let's fix that.

---

## Give each part a mass

Select a part and click its **Edit properties** pencil. The **Mass** section has two ways to answer.

![The mass section of a part's properties](/learn/snakie_build_urdf/assets/mass.jpg){:class="img-fluid w-100"}

- **Use estimate**: Snakie works out the mass from the part's size, a **Material** (PLA, PETG, ABS or Resin) and an **Infill** percentage. That's how a 3-D printed part is described, so the guess is a good one
- **Measured (g)**: type what the kitchen scales say. A real measurement always beats an estimate

Click **Use estimate** on all four parts. The line under the list changes to **mass known for 4 of 4 parts**, and a **Total mass** appears. Ours comes to **42 g**.

Snakie only uses the parts you've weighed. It leaves the others out instead of guessing, which is why it's worth doing them all.

---

## See the centre of mass

Click **Balance** in the bottom right of the view.

![The centre of mass marker on the arm](/learn/snakie_build_urdf/assets/balance.jpg){:class="img-fluid w-100"}

Two markers appear:

- A **white ball** at the centre of mass: the single point where the robot's weight acts
- A **ring on the floor** directly underneath it

The rule for standing up is simple. If the ring is inside the robot's footprint, it stands. If the ring wanders outside, it falls.

---

## Tell Snakie where the feet are

Snakie doesn't know which bits of your robot touch the floor until you say so. That's what **Ground contacts** in the properties panel are for.

1. Open the properties of `base_link`
2. Click **+ Add contact point**. A row of **x**, **y** and **z** boxes appears
3. Add four points, one for each bottom corner of the base: x and y of `40` or `-40`, and z of `-10`

![A ground contact point on the base](/learn/snakie_build_urdf/assets/ground_contact.jpg){:class="img-fluid w-100"}

With the contacts in place, Snakie draws the support area on the floor and the chip changes from "tag feet for stability" to **stable**, with the distance the balance point has to spare before the robot tips.

![The support area and the stable chip](/learn/snakie_build_urdf/assets/support.jpg){:class="img-fluid w-100"}

The numbers are measured from the middle of the part. The base is 80 mm square and 20 mm thick, so its bottom corners are 40 mm out and 10 mm down.

For a walking robot you'd tag each foot. For a rover, each wheel.

---

## Three more ways to look

**Bone Mode** fades the blocks and shows the skeleton: each joint's name and angle, and the length of each bone between joints.

![Bone Mode showing joint names and bone lengths](/learn/snakie_build_urdf/assets/bone_mode.jpg){:class="img-fluid w-100"}

**Measure distance** is in the top toolbar. Click two points and Snakie tells you how far apart they are. It snaps to corners, edges and face centres, so you can measure real features of the robot.

![Measuring between two points on the arm](/learn/snakie_build_urdf/assets/measure.jpg){:class="img-fluid w-100"}

**Exploded view** pulls the parts away from each other so you can see how they connect. Click it again to put the robot back together.

---

## Try it Yourself

1. Give all four parts an estimated mass
2. Change the base's **Infill** to 100% and click **Use estimate** again. How much did the total go up?
3. Turn on **Balance** and note where the ring sits
4. Measure from one end of the forearm to the other. It should be close to 60 mm

---

## Common Issues

**Problem**: The Balance button shows nothing.
**Solution**: Give at least one part a mass.
**Why**: With no masses there's no weight to find the centre of.

**Problem**: The centre of mass looks too low.
**Solution**: Check that every part has a mass.
**Why**: Parts with no mass are left out, so a heavy base on its own drags the marker down.

**Problem**: My measured mass doesn't match the estimate.
**Solution**: Trust the scales.
**Why**: The estimate can't know about screws, servos or how your slicer really filled the part.

---

## Check your understanding

{% quiz %}
? What does Snakie need before it can show the centre of mass?
- [ ] A servo on every joint
- [x] A mass for the parts
- [ ] A saved pose
> No mass, no centre of mass. Estimated or measured both work.

? The floor ring is outside the robot's footprint. What happens to a real robot?
- [x] It tips over
- [ ] Nothing
- [ ] It gets lighter
> The weight must sit over the feet for a robot to stand.

? Which is more trustworthy?
- [ ] The estimate
- [x] A measured mass
- [ ] They're always the same
> Estimates are good guesses. Scales are facts.
{% endquiz %}
