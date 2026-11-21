---
layout: lesson
title: Summary and Next Steps
author: Kevin McAleer
type: page
cover: /learn/bettercad_joints_urdf/assets/banners/13_summary.jpg
date: 2026-11-21
previous: 12_exporting_urdf.html
description: Review what you learned about joints, poses, clash checks, drive links
  and URDF in BetterCad and find out where to go next
percent: 100
duration: 4
quiz_total: 39
date_updated: 2026-11-21
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

You started with three flat bars on a print bed. You finish with an arm that bends at two joints, knows its own weight and limits, can't be driven into itself, and can be handed to a simulator in one file.

![The finished desk arm with its joints and poses](/learn/bettercad_joints_urdf/assets/poses.jpg){:class="img-fluid w-100"}

If you'd like to compare, here is my finished arm: [desk-arm.bcad](/learn/bettercad_joints_urdf/assets/desk-arm.bcad). The servo deck with its drive link is here: [servo-deck-drive.bcad](/learn/bettercad_joints_urdf/assets/servo-deck-drive.bcad).

---

## What you learned

| Topic | The short version |
|---|---|
| Components | A group of bodies that move as one. The Joint tool makes them for you |
| Joints | Two clicks: a snap on the part that moves, then a snap on what it hangs from |
| Posing | Slider, typed angle or drag. Home is where each joint was made |
| Chains | A part moves with every joint between it and Base |
| Kinds | Hinge, Wheel, Slider and Fixed, which URDF calls revolute, continuous, prismatic and fixed |
| Line-up | Angle, Gap, offsets and Flip set where a joint rests |
| Named poses | Save the joint angles that matter and go back in a click |
| Tool point | Drag the arm by its tip and the joints work themselves out |
| Mass | Printed estimate or datasheet weight, with a centre of mass for each moving part |
| Clash check | Finds overlaps and touches in one pose |
| Sweep | Runs joints through their range, reports where parts hit, and sets safe limits |
| Drive links | One joint drives another through gears, a belt, a rack or a ratio |
| URDF | Links, joints, meshes, masses and limits in one zip |
{:class="table table-single"}

---

## A workflow worth keeping

1. Design the parts with a hole wherever a joint will go.
2. Make each joint at a sensible home pose, and name it straight away.
3. Set limits from your hardware.
4. Save the poses you care about.
5. Sweep. Fix what hits, or tighten the limits.
6. Check the heaviest pose against your servo's torque.
7. Print.

Steps 5 and 6 are the ones that save filament.

---

## Try it Yourself

1. **Add a wrist.** Design a small gripper plate, joint it to the tip of the forearm, and move the tool point onto it.
2. **Fit real servos.** Drop an SG90 from the robot catalogue into the base and the upper arm, cut their pockets, and re-run the sweep with **Reference bodies** ticked.
3. **Gear the shoulder.** Put a 2 : 1 gear pair between a servo and the shoulder, add the drive link, and work out how much more the arm can now lift.
4. **Build a rover.** Four wheel joints, linked with **Link joints**, on a chassis plate.

---

## What's Next

- [Design Robot Parts with BetterCad](/learn/bettercad_robot_parts/) if you came here first. It covers the catalogue, pockets, mounts and the gears from the drive links lesson
- [Print It Right with BetterCad](/learn/bettercad_printing/) to make sure the holes your joints rely on come out the right size
- [Learn ROS with me](/learn/learn_ros/) to put your URDF to work
- [Wire It Up - Electronics in Snakie](/learn/snakie_electronics/) to wire the servos to a Pico
- [Getting Started with Snakie](/learn/snakie_getting_started/) to write the MicroPython that moves them to the angles in your poses

Both BetterCad courses for robot builders are in the [BetterCad for Robot Builders](/learn/learning_pathways/bettercad-robot-builders.html) pathway.

And for build videos, robot projects and livestreams, subscribe to [Kevin McAleer on YouTube](https://www.youtube.com/c/kevinmcaleer28).

---

## Check your understanding

{% quiz %}
? You've changed the shape of the upper arm. Which check should you run again before printing?
- [ ] Export a URDF
- [x] A sweep
- [ ] Save a pose
> A changed part can clash where the old one didn't. A sweep covers every joint's whole range.

? Which of these is saved in a URDF export?
- [ ] Your named poses
- [ ] The timeline
- [x] Each joint's kind, axle and limits
> URDF describes the robot: links, joints, meshes and masses. Poses stay in the BetterCad model.

? What is the quickest way to find the joint angles that put the tip at a certain spot?
- [ ] Try angles in the sliders until it looks right
- [x] Set a tool point and drag the tip there
- [ ] Run a sweep
> BetterCad solves the angles as you drag, then you can read them from the Pose section.
{% endquiz %}
