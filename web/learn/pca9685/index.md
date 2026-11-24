---
layout: course_overview
title: Building a Robot Arm with Raspberry Pi and PCA9685
author: Kevin McAleer
cover: /learn/pca9685/assets/cover.png
date: 2024-04-06
description: Build Buddy Jr, a small 3D printed robot arm with four servos, and bring
  it to life with a Raspberry Pi, Python and a PCA9685 servo driver. You'll set up
  the Pi, wire the board safely with a separate servo supply, learn the maths from
  angle to pulse, assemble the arm and program smooth, repeatable movements.
level: beginner
course_duration: 1h 10m
lesson_count: 12
first_lesson: 00_intro.html
quiz_total: 33
video: _JJAV97Mm98
outcomes:
- Wire a PCA9685 to a Raspberry Pi and power the servos from their own supply
- Turn an angle into a 12-bit PWM count and explain the maths
- Control four servos by name from Python with the Adafruit PCA9685 library
- Calibrate each joint and keep it inside safe limits
- Make smooth movements between named poses and run an automated routine
- Find faults in a servo arm by testing one link of the chain at a time
prerequisites:
- Some Python, such as Python for beginners
- Comfort typing commands in a terminal, such as Introduction to the Linux Command
  Line on Raspberry Pi OS
you_need:
- A Raspberry Pi with header pins, a power supply and a microSD card
- A PCA9685 16-channel servo driver board
- Four SG90 style servos and the 3D printed Buddy Jr parts
- A separate 5 V supply for the servos, able to give at least 2 A
- Jumper wires and a small cross-head screwdriver
navigation:
- name: Building a Robot Arm with Raspberry Pi and PCA9685
- content:
  - section: Getting Started with Raspberry Pi and Python
    content:
    - name: Introduction
      link: 00_intro.html
    - name: Setting Up Your Raspberry Pi
      link: 02_raspberry_pi_setup.html
    - name: Python Basics for Robotics
      link: 03_python_basics_for_robotics.html
  - section: Understanding the PCA9685 Servo Driver
    content:
    - name: PCA9685 Servo Driver
      link: 04_introduction_to_pca9685.html
    - name: Setting Up PCA9685
      link: 05_setting_up_the_pca9685_with_raspberry_pi.html
  - section: Building the Robot Arm
    content:
    - name: Design
      link: 06_designing_your_robot_arm.html
    - name: Assembly
      link: 07_assembling_the_robot_arm.html
  - section: Programming the Robot Arm
    content:
    - name: Controlling Servos with Python
      link: 08_controlling_servos_with_python.html
    - name: Basic Arm Movements
      link: 09_creating_basic_arm_movements.html
    - name: Advanced Control and Automation
      link: 10_advanced_arm_control_and_automation.html
  - section: Project Challenges and Ideas
    content:
    - name: Project Ideas
      link: 11_project_ideas.html
    - name: Troubleshooting and Next Steps
      link: 12_troubleshooting_common_issues.html
---
