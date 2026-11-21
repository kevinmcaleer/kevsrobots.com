---
layout: lesson
title: Build and Animate a Robot in Snakie
author: Kevin McAleer
type: page
cover: /learn/snakie_build_urdf/assets/banners/00_intro.jpg
date: 2026-11-21
next: 01_the_build_workspace.html
description: Learn to build a robot in 3-D in Snakie from blocks and joints, pose
  and animate it, then drive it from MicroPython
percent: 6
duration: 3
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

## Overview

Snakie isn't only a code editor. Its **Build** workspace is a small 3-D robot studio. You stack blocks into a robot, join them with hinges, drag the robot into poses, and string the poses into a motion. Then comes the best part: your MicroPython code moves the same 3-D model, with no hardware on the desk.

In this course you'll build one robot from a blank floor: a **three-joint desk arm** with a waist, a shoulder and an elbow. You'll pose it, animate it, bind a servo to each joint and write the code that makes it wave.

{% include youtubeplayer.html id="QVe9meIHZtk" %}

New to Snakie? Watch Kev's introduction above first, then come back.

---

## Course Content

- A tour of the Build workspace and the files it keeps
- Adding blocks and sizing, colouring and naming them
- Joining blocks with the joint tool
- Hinges, wheels and sliders, and the limits that keep them safe
- Mass, balance, measuring and the skeleton view
- Posing with sliders and by dragging the robot
- Saving poses and building a motion from them
- Fine control with joint tracks and keyframes
- Binding servos to joints
- Driving the 3-D arm from your own MicroPython
- Exporting a motion as ready-to-run code
- URDF files: exporting yours and opening other people's
- Rendering a movie of your robot

---

## Key Results

After this course you will be able to:

- Build a jointed robot model from boxes, tubes and balls
- Choose the right joint type, axis and limits for each moving part
- Save named poses and play them back as a looping motion
- Tell Snakie which servo pin drives which joint
- Run MicroPython on the simulated device and watch the 3-D robot follow it
- Export the robot as a URDF file and open a URDF made elsewhere
- Render a short video of your robot moving

---

## What you'll need

- **Snakie**, either the desktop app or [app.snakie.org](https://app.snakie.org) in Chrome or Edge. The Build workspace saves its files in a project folder, so the browser needs to be one that can open folders.
- Nothing else. The whole course runs on Snakie's simulated device.
- Optional: three hobby servos and a Raspberry Pi Pico, if you want to build the real thing afterwards.

It helps to have done [Snakie - Getting Started](/learn/snakie_getting_started/) so you know your way around, and [Wire It Up - Electronics in Snakie](/learn/snakie_electronics/) if you plan to wire real servos.

This course was written with Snakie **0.92.0**. Snakie changes quickly, so a button may have moved by the time you read this.

---

## How the course works

Each lesson adds one thing to the same arm, so do them in order. Screenshots show exactly what to expect, there's a **Try it Yourself** task in every lesson, and a short quiz at the end checks that the idea has stuck.

Snakie also has a short built-in course called **Build a Robot in 3-D** under the **Learn** button. This course goes further and builds a different robot, so the two work well together.
