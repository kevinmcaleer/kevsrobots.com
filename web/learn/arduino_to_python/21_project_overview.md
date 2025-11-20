---
layout: lesson
title: Capstone Project - Wall-Following Robot
author: Kevin McAleer
type: page
cover: /learn/arduino_to_python/assets/project_overview.jpg
date: 2025-01-20
previous: 20_when_to_use_what.html
next: 22_building_c_version.html
description: Building a robot in C++, Python, and hybrid approaches
percent: 66
duration: 2
navigation:
- name: Arduino to Python - A Developer's Guide
- content:
  - section: Getting Started
    content:
    - name: Introduction
      link: 00_intro.html
    - name: Why Python Alongside C++?
      link: 01_why_python.html
    - name: Understanding the Arduino Uno Q Architecture
      link: 02_arduino_uno_q.html
    - name: Setting Up Your Python Development Environment
      link: 03_setup_environment.html
  - section: Language Fundamentals
    content:
    - name: Variables and Data Types
      link: 04_variables_and_types.html
    - name: Control Flow - if, else, and Loops
      link: 05_control_flow.html
    - name: From setup() and loop() to Python Functions
      link: 06_functions.html
    - name: Arduino String Class vs Python str
      link: 07_strings.html
    - name: Fixed Arrays vs Dynamic Lists
      link: 08_arrays_vs_lists.html
  - section: Hardware Programming
    content:
    - name: Digital Pin Control
      link: 09_pin_control.html
    - name: Analog I/O - analogRead() and analogWrite() in MicroPython
      link: 10_analog_io.html
    - name: Serial Communication - Serial vs UART
      link: 11_serial_communication.html
    - name: Timing and Delays - delay(), millis(), and Non-Blocking Code
      link: 12_timing_and_delays.html
    - name: Interrupts and Event Handling
      link: 13_interrupts.html
  - section: Advanced Concepts
    content:
    - name: Memory Management - Manual vs Automatic
      link: 14_memory_management.html
    - name: Object-Oriented Programming
      link: 15_object_oriented.html
    - name: Arduino Libraries vs Python Modules
      link: 16_libraries_vs_modules.html
    - name: File Systems - A New Capability in Python
      link: 17_file_systems.html
  - section: Arduino Uno Q Dual Processor
    content:
    - name: Understanding the Dual-Processor System
      link: 18_understanding_dual_processor.html
    - name: MCU-MPU Communication Patterns
      link: 19_mpu_mcu_communication.html
    - name: Decision Matrix - When to Use What
      link: 20_when_to_use_what.html
  - section: Capstone Project
    content:
    - name: Capstone Project - Wall-Following Robot
      link: 21_project_overview.html
    - name: Building the C++ Version
      link: 22_building_c_version.html
    - name: Building the Python Version
      link: 23_building_python_version.html
    - name: Hybrid Approach - Best of Both Worlds
      link: 24_hybrid_approach.html
    - name: Course Summary and Next Steps
      link: 25_summary.html
---


## Introduction

Time to put everything together! You'll build a wall-following robot three ways:
1. Pure Arduino C++
2. Pure MicroPython
3. Hybrid (C++ + Python on Arduino Uno Q)

## Project Goals

Build a robot that:
- Follows walls using distance sensors
- Avoids obstacles
- Maintains consistent distance from wall
- Can be controlled remotely (hybrid version)

## Hardware Needed

### Essential:
- Robot chassis with 2 motors
- Motor driver (L298N or similar)
- 2-3 ultrasonic sensors (HC-SR04)
- Microcontroller (Pico, ESP32, or Arduino Uno Q)
- Battery pack (7.4V LiPo or 6× AA)
- Breadboard and jumper wires

### Optional:
- LEDs for status indication
- Buzzer for alerts
- OLED display

## Wall-Following Algorithm

```python
while True:
    left_dist = read_left_sensor()
    front_dist = read_front_sensor()
    
    if front_dist < 20:  # Obstacle ahead
        turn_right()
    elif left_dist < 15:  # Too close to wall
        turn_slight_right()
    elif left_dist > 25:  # Too far from wall
        turn_slight_left()
    else:
        move_forward()
```

## What You'll Learn

- Implementing same algorithm in different languages
- Performance comparison
- Code complexity comparison
- When hybrid approach adds value

Ready? Let's build!

---

> **Previous:** [When to Use What](/learn/arduino_to_python/20_when_to_use_what.html) |
> **Next:** [Building C++ Version](/learn/arduino_to_python/22_building_c_version.html)
>
> **Course Home:** [Arduino to Python](/learn/arduino_to_python/)
