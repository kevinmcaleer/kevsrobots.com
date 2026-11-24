---
layout: lesson
title: Introduction
author: Kevin McAleer
type: page
cover: /learn/open_duck_mini/assets/banners/00_intro.jpg
date: 2026-11-24
next: 01_what_is_open_duck_mini.html
description: Meet the Open Duck Mini, a small open-source walking robot with a reinforcement
  learning brain, and see how this course takes you from printed parts to a duck that
  walks
percent: 6
duration: 4
quiz_total: 12
date_updated: 2026-11-24
navigation:
- name: Build an Open Duck Mini Walking Robot
- content:
  - section: Meet the duck
    content:
    - name: Introduction
      link: 00_intro.html
    - name: What is the Open Duck Mini?
      link: 01_what_is_open_duck_mini.html
  - section: Parts and printing
    content:
    - name: The Parts List
      link: 02_parts_list.html
    - name: Printing the Parts
      link: 03_printing_the_parts.html
    - name: Serial Bus Servos and IDs
      link: 04_bus_servos_and_ids.html
  - section: Assembly
    content:
    - name: Assembling the Legs
      link: 05_assembling_the_legs.html
    - name: Head, Body and Battery
      link: 06_head_body_and_battery.html
    - name: Wiring the Electronics
      link: 07_wiring_the_electronics.html
  - section: Bring it to life
    content:
    - name: Setting up the Pi
      link: 08_setting_up_the_pi.html
    - name: Testing the IMU, Motors and Controller
      link: 09_testing_imu_motors_and_controller.html
    - name: Calibration and Offsets
      link: 10_calibration_and_offsets.html
    - name: First Steps
      link: 11_first_steps.html
  - section: The walking brain
    content:
    - name: How the Duck Learned to Walk
      link: 12_how_the_duck_learned_to_walk.html
    - name: Training Your Own Policy
      link: 13_training_your_own_policy.html
    - name: Troubleshooting
      link: 14_troubleshooting.html
    - name: Summary
      link: 15_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

Ahoy there makers! Have you seen the little Disney droid that waddles around with real personality? The Open Duck Mini is a small, open-source take on that robot, and you can build one at home. In this course we'll build one duck, get it walking, and then open up its head to see how its walking brain was made.

Here is Kev's build log livestream. Watch a bit of it first to see what you're getting into, then come back and let's build.

{% include youtubeplayer.html id="5EZGgPj6ZLE" %}

---

## Overview

The Open Duck Mini is about 42 cm tall. It has two legs with five joints each, a head that can nod and turn, and a Raspberry Pi Zero 2W inside running a neural network. Nobody wrote the code for its walk by hand. The walk was **learned** in a physics simulator using reinforcement learning, then copied onto the real robot.

The project was created by **Antoine Pirrone** ([apirrone on GitHub](https://github.com/apirrone)) and is built and improved by the Open Duck community. It's released under the Apache-2.0 licence and sponsored by Hugging Face. Everything in this course stands on their work, so a big thank you to Antoine and everyone who has contributed.

The project lives in four repos. These are the source of truth, and they change, so keep them open as you go:

- [Open_Duck_Mini](https://github.com/apirrone/Open_Duck_Mini): the CAD, the bill of materials, print and assembly guides
- [Open_Duck_Mini_Runtime](https://github.com/apirrone/Open_Duck_Mini_Runtime) (branch `v2`): the code that runs on the Pi
- [Open_Duck_Playground](https://github.com/apirrone/Open_Duck_Playground): where the walking policy is trained
- [Open_Duck_reference_motion_generator](https://github.com/apirrone/Open_Duck_reference_motion_generator): makes the example gaits the policy learns from

There's one thread through the whole course: **build one Open Duck Mini, get it walking with the pretrained policy, then understand and retrain its walking brain.**

## Course Content

- **Meet the duck**: what the Open Duck Mini is, and how the four repos fit together
- **Parts and printing**: the parts list, printing in PLA and TPU, and setting up the serial bus servos and their IDs
- **Assembly**: the legs, the head, body and battery, and wiring the electronics
- **Bring it to life**: setting up the Pi, testing each part, calibration and offsets, and first steps
- **The walking brain**: how the duck learned to walk, training your own policy, troubleshooting, and where to go next

## Key Results

After completing this course, you will:

- Explain how the CAD, simulator, training code and runtime fit together
- Configure 14 Feetech STS3215 serial bus servos with the right IDs
- Assemble and wire a complete Open Duck Mini
- Set up the runtime on the Pi, test every part and find the joint offsets
- Make the duck walk with the pretrained policy and steer it with an Xbox controller
- Describe how an imitation reward and a motor model help a simulated walk work on a real robot

## What you'll need

- The parts on the **bill of materials (BOM)**, a Google Sheet linked from the [Open_Duck_Mini README](https://github.com/apirrone/Open_Duck_Mini). The project aims to keep the whole BOM under $400. Check the sheet for current parts and prices
- A 3D printer that can print PLA and TPU
- A soldering iron and some basic soldering skill
- A computer to set up the Pi from, and an Xbox controller with Bluetooth
- Optional, for lesson 13: a decent NVIDIA GPU, or a cloud GPU

This course leans on two others. If reinforcement learning or the Linux terminal are new to you, start with these:

- [Reinforcement Learning for Beginners](/learn/reinforcement_learning/)
- [Introduction to the Linux Command Line on Raspberry Pi OS](/learn/linux_intro/)

## How the course works

**One duck, start to finish.** Each lesson moves the same robot one step closer to walking. By lesson 11 it takes its first steps.

**The upstream repos win.** The Open Duck project moves fast. Where this course and the repo disagree, trust the repo. We'll point you to the right file whenever exact numbers matter, such as prices, screw counts or print settings.

**Commands.** Shell commands are shown in `bash` blocks. Run them on the Pi unless the lesson says to run them on your computer.

**Quizzes.** Some lessons end with a short quiz. Your score is kept in your browser and shown in the sidebar.

**Mistakes are normal.** A walking robot has a lot of moving parts. Lesson 14 collects the common problems, and most lessons have a Common Issues list too.

Ready? Let's meet the duck.
