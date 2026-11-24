---
layout: course_overview
title: Arduino to MicroPython - A Quick Start Guide
author: Kevin McAleer
cover: /learn/arduino_to_python/assets/cover.jpg
date: 2025-11-20
description: Transition from Arduino C++ to MicroPython quickly and easily. Learn
  Python syntax with side-by-side comparisons, control hardware, and build IoT and
  robotics projects on Arduino Nano ESP32, Raspberry Pi Pico W, or Pico 2W.
level: beginner
course_duration: 1h 15m
lesson_count: 12
first_lesson: 00_intro.html
quiz_total: 43
video: A7ZJ5TC-zUk
outcomes:
- Translate common Arduino C++ sketches into working MicroPython programs
- Control digital pins, analogue inputs and PWM outputs with the machine module
- Use seconds, milliseconds and ticks_diff correctly, and blink without delay
- Test hardware live in the REPL and debug with print and f-strings
- Build a WiFi temperature web server and an obstacle-avoiding robot
- Decide when MicroPython or Arduino C++ is the better tool
prerequisites:
- Arduino C++ basics, such as writing and uploading a sketch
- Basic electronics (LEDs, resistors, breadboards)
you_need:
- A Raspberry Pi Pico W or Pico 2 W with MicroPython installed
- A USB cable, a breadboard, LEDs, resistors and a push button
- Thonny on your computer
- For the robot project, two DC motors, a motor driver, an HC-SR04 sensor and a battery
  pack
navigation:
- name: Arduino to MicroPython - A Quick Start Guide
- content:
  - section: Getting Started
    content:
    - name: Introduction to Arduino to MicroPython
      link: 00_intro.html
    - name: Why MicroPython?
      link: 01_why_micropython.html
  - section: Language Fundamentals
    content:
    - name: Syntax Basics - No More Semicolons!
      link: 02_syntax_basics.html
    - name: Control Flow - If, Loops, and No Braces!
      link: 03_control_flow.html
    - name: Functions - Goodbye setup and loop!
      link: 04_functions.html
    - name: Pin Control - Goodbye pinMode, Hello Pin Class!
      link: 05_pin_control.html
  - section: Hardware Basics
    content:
    - name: Analog Reading and PWM - ADC and PWM Classes
      link: 06_analog_pwm.html
    - name: Timing and Delays - Seconds vs Milliseconds!
      link: 07_timing.html
    - name: Serial Communication and REPL - No More Serial.begin!
      link: 08_serial_basics.html
  - section: Practical Projects
    content:
    - name: IoT Project - WiFi Temperature Monitor
      link: 09_iot_project.html
    - name: Robot Project - Simple Motor Control
      link: 10_robot_project.html
    - name: Next Steps and Quick Reference
      link: 11_next_steps.html
---
