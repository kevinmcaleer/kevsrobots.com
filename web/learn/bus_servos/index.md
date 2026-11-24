---
layout: course_overview
title: Bus Servos and the Buddy Robot Arm
author: Kevin McAleer
cover: /learn/bus_servos/assets/cover.jpg
date: 2026-11-24
description: 'Learn to drive Feetech STS3215 bus servos from a Raspberry Pi Pico.
  Start with one servo on a single data wire, chain two, record and replay poses by
  hand, then build Buddy: a six-joint robot arm with calibration, smooth synchronised
  moves, inverse kinematics and its own web control panel.'
level: intermediate
course_duration: 1h 24m
lesson_count: 16
first_lesson: 00_intro.html
quiz_total: 19
video: nzBZOTdEdtE
outcomes:
- Explain what a bus servo is and why it beats a PWM hobby servo for robot arms
- Wire a Pico to a Waveshare bus servo driver board and move an STS3215 from MicroPython
- Read position, speed, movement and temperature back from every servo
- Give each servo its own ID and chain several on one cable
- Record poses by hand with torque off and play them back
- Calibrate a six-joint arm, move its joints so they arrive together, and drive it
  with kinematics
- Control the arm from a web page served by the Pico itself
prerequisites:
- Learn MicroPython - The basics
- Motors and motor drivers
you_need:
- A Raspberry Pi Pico (lessons 1 to 8). A Pico W for the web control panel in lesson
  13
- A Waveshare serial bus servo driver board
- At least two Feetech STS3215 servos. Six for the Buddy arm
- A 7.4V power supply or 2S LiPo battery
- Jumper wires, and a computer with Thonny or mpremote
- Optional, a 3D printed arm. The files and the code are in Kev's GitHub repo
navigation:
- name: Bus Servos and the Buddy Robot Arm
- content:
  - section: Introduction
    content:
    - name: Introduction
      link: 00_intro.html
  - section: Your first bus servo
    content:
    - name: Why bus servos
      link: 01_why_bus_servos.html
    - name: Parts and wiring
      link: 02_parts_and_wiring.html
    - name: The library and your first move
      link: 03_the_library_and_first_move.html
    - name: Reading feedback
      link: 04_reading_feedback.html
  - section: Two servos and beyond
    content:
    - name: Servo IDs and chaining
      link: 05_servo_ids_and_chaining.html
    - name: The wave demo
      link: 06_the_wave_demo.html
    - name: Teach and repeat
      link: 07_teach_and_repeat.html
    - name: Gearing and choosing servos
      link: 08_gearing_and_choosing_servos.html
  - section: The Buddy arm
    content:
    - name: Meet Buddy
      link: 09_meet_buddy.html
    - name: Calibrating offsets
      link: 10_calibrating_offsets.html
    - name: Smooth synchronised moves
      link: 11_smooth_synchronised_moves.html
    - name: Kinematics
      link: 12_kinematics.html
  - section: Control panel and wrap-up
    content:
    - name: The web control panel
      link: 13_web_control_panel.html
    - name: Troubleshooting
      link: 14_troubleshooting.html
    - name: Summary
      link: 15_summary.html
---
