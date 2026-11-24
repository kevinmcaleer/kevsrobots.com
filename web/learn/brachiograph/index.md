---
layout: course_overview
title: BrachioGraph Tutorial
author: Kevin McAleer
cover: /learn/brachiograph/assets/cover.png
date: 2024-07-26
description: Build your own BrachioGraph, a cheap pen plotter made from three hobby
  servos, lolly sticks and a Raspberry Pi. Wire it up, install the software, calibrate
  it, and draw shapes and vectorised images from Python.
level: beginner
course_duration: 0h 43m
lesson_count: 8
first_lesson: 00_intro.html
quiz_total: 21
video: tkAv3HIw3CY
outcomes:
- Build a pen plotter with two 8cm arms and a servo pen lift
- Wire three servos to GPIO 14, 15 and 18 on a Raspberry Pi
- Install the BrachioGraph library and start the pigpio service
- Move the arm and pen from Python and read the plotter status
- Calibrate a plotter with parked pulse-widths, hysteresis correction and degree values
- Convert an image to lines and plot it
prerequisites:
- Basic comfort with a keyboard and a terminal, such as Introduction to the Linux
  Command Line on Raspberry Pi OS
- Helpful but not required, Python for beginners
you_need:
- A Raspberry Pi with GPIO header, power supply, microSD card, screen and keyboard
- Three hobby servos (the official docs recommend SG90) and jumper wires
- Lolly sticks or card, a clothes peg, foam board, hot glue and a pen
- Paper (A5) and a printer for the base template
navigation:
- name: BrachioGraph Tutorial
- content:
  - section: Introduction
    content:
    - name: BrachioGraph Pen Plotter
      link: 00_intro.html
  - section: Build the plotter
    content:
    - name: Building the Plotter
      link: 01_build.html
    - name: Wiring up the BrachioGraph
      link: 02_wire.html
  - section: Programming
    content:
    - name: Setup Raspberry Pi
      link: 03_setup_pi.html
    - name: Code
      link: 04_code.html
    - name: Draw
      link: 05_draw.html
    - name: Vectorize Images
      link: 06_vectorize.html
  - section: Summary
    content:
    - name: Summary and Key Takeaways
      link: 07_summary.html
---
