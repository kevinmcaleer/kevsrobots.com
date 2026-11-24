---
layout: course_overview
title: Wall Drawing Robot Tutorial
author: Kevin McAleer
cover: /learn/scrawly_wally//assets/cover.png
date: 2024-08-02
description: Learn how to build your own wall drawing robot using a Raspberry Pi Zero
  2 W, stepper motors, and a gondola with a marker. This course covers the essential
  steps from assembling the robot, installing the software, and creating your first
  drawing.
level: intermediate
course_duration: 1h 8m
lesson_count: 11
first_lesson: 00_intro.html
quiz_total: 30
video: JB4rPNBCL3Q
outcomes:
- Explain how two belt lengths fix the position of the pen on a wall plotter
- Assemble and wire a two-motor wall drawing robot around a Raspberry Pi Zero 2 W
- Test 28BYJ-48 stepper motors and a pen servo from Python
- Convert an (x, y) position into belt lengths and then motor steps, and calibrate
  the numbers for your own board
- Run a complete program that draws a square, after a dry run on your computer
- Turn a simple SVG into coordinates the plotter can follow
prerequisites:
- Basic Python, such as the Python course
- A little comfort in a terminal, such as the Linux command line course
- Helpful but not required, Motors and motor drivers
you_need:
- A Raspberry Pi Zero 2 W, a microSD card and a power supply
- Two 28BYJ-48 stepper motors with ULN2003 driver boards, and an SG90 servo
- Two 3GT 20 tooth cogs and timing belt, and the three 3D printed parts (gondola and
  two motor mounts)
- A whiteboard or smooth board, a marker pen, and Dupont jumper wires
- A computer for writing code and preparing drawings. Lessons 8 to 10 work without
  the robot
navigation:
- name: Wall Drawing Robot Tutorial
- content:
  - section: Introduction
    content:
    - name: Introduction to the Wall Drawing Robot
      link: 00_intro.html
  - section: Build the Robot
    content:
    - name: Components Needed
      link: 02_components.html
    - name: Assembling the Robot
      link: 03_assembly.html
    - name: Wiring the Components
      link: 04_wiring.html
  - section: Programming
    content:
    - name: Setting Up the Raspberry Pi
      link: 05_setup_pi.html
    - name: Installing the Software
      link: 06_software.html
    - name: Writing Control Code
      link: 07_control.html
    - name: Understanding Positioning with Trigonometry
      link: 08_positioning.html
    - name: Creating Your First Drawing
      link: 09_draw.html
    - name: Vectorizing Images for Drawing
      link: 10_vectorize.html
  - section: Summary
    content:
    - name: Summary and Next Steps
      link: 11_summary.html
---
