---
layout: course_overview
title: Build a SMARS Quad Walking Robot
author: Kevin McAleer
cover: /learn/smars_quad/assets/how_to_build_a_quad.jpg
date: 2023-03-04
description: 3D print, assemble and wire up the SMARS Quad, a four-legged walking
  robot with eight SG90 servos, a PCA9685 servo board and a Raspberry Pi Zero, then
  make it walk with Python.
level: intermediate
course_duration: 0h 57m
lesson_count: 15
first_lesson: 00_intro.html
quiz_total: 42
video: 32GXhAuXZrQ
outcomes:
- Print and assemble a four-legged SMARS Quad with eight servos
- Wire a PCA9685 servo board to a Raspberry Pi and power the servos from their own
  supply
- Work out which PCA9685 channel drives which leg and foot
- Turn a servo angle into a PCA9685 count
- Explain a creeping gait and why three feet stay on the ground
- Run SMARSLab and your own Python script to make the quad walk
prerequisites:
- Build a SMARS Robot (helpful, not essential)
- Some Python and comfort typing commands in a terminal
you_need:
- A 3D printer and the SMARS Quad files from Thingiverse
- A Raspberry Pi Zero 2 WH with Raspberry Pi OS on a microSD card
- A PCA9685 16-channel servo board and 8 SG90 servos
- A separate 5V supply or battery pack for the servos, able to give at least 2 A
- Jumper wires and a small screwdriver
navigation:
- name: Build a SMARS Quad Walking Robot
- content:
  - section: Overview
    content:
    - name: SMARS Quad robot Overview
      link: 00_intro.html
  - section: Print parts
    content:
    - name: Print the Parts
      link: quad_01.html
    - name: Servo Holders
      link: quad_02.html
    - name: Fit the servos
      link: quad_03.html
  - section: Assemble robot
    content:
    - name: Servo Frame
      link: quad_04.html
    - name: Servo Arms
      link: quad_05.html
    - name: Assemble arms
      link: quad_06.html
    - name: Calibrate Servos
      link: quad_07.html
    - name: Remaining Servo Holders
      link: quad_08.html
    - name: Print Feet
      link: quad_09.html
    - name: Assemble Feet
      link: quad_10.html
    - name: Check Rotation
      link: quad_11.html
  - section: Wire and code
    content:
    - name: Code
      link: quad_12.html
    - name: Wiring
      link: quad_13.html
    - name: Make it walk
      link: quad_14.html
---
