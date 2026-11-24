---
layout: course_overview
title: Build an Open Duck Mini Walking Robot
author: Kevin McAleer
cover: /learn/open_duck_mini/assets/cover.jpg
date: 2026-11-24
description: Build an Open Duck Mini, the open-source walking robot inspired by Disney's
  BDX droid. Print and assemble it, wire up a Raspberry Pi Zero 2W, 14 serial bus
  servos and an IMU, get it walking with the pretrained policy, then learn how its
  reinforcement learning brain was trained and train your own.
level: intermediate
course_duration: 1h 17m
lesson_count: 16
first_lesson: 00_intro.html
quiz_total: 12
video: _YHOQFOzjKg
outcomes:
- Explain how the four Open Duck repos fit together, from CAD to simulation to a policy
  on the Pi
- Configure 14 Feetech STS3215 serial bus servos with the right IDs before assembly
- Assemble the legs, head and body, and wire the Pi, IMU, foot switches and servo
  bus
- Set up the runtime on the Pi, test every part, calibrate the IMU and find the joint
  offsets
- Make the duck walk with the pretrained ONNX policy and drive it with an Xbox controller
- Describe how imitation rewards, a motor model and domain randomisation close the
  sim-to-real gap, and train a policy of your own
prerequisites:
- Reinforcement Learning for Beginners
- Introduction to the Linux Command Line on Raspberry Pi OS
- Basic soldering
you_need:
- The parts on the Open Duck Mini bill of materials (a Google Sheet linked from the
  Open_Duck_Mini repo)
- 14 Feetech STS3215 serial bus servos and the USB-serial servo control board listed
  on the BOM
- A Raspberry Pi Zero 2W (a Raspberry Pi 5 also works) and a microSD card
- A BNO055 IMU, two foot micro switches and the battery pack from the BOM
- M3 heat-set inserts, M3x10 and M3x6 screws, and Loctite 243
- A 3D printer that can print PLA and TPU, and a soldering iron
- An Xbox controller with Bluetooth
- Optional, for lesson 13, a decent NVIDIA GPU or a cloud GPU
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
