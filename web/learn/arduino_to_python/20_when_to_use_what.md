---
layout: lesson
title: Decision Matrix - When to Use What
author: Kevin McAleer
type: page
cover: /learn/arduino_to_python/assets/when_to_use.jpg
date: 2025-01-20
previous: 19_mpu_mcu_communication.html
next: 21_project_overview.html
description: Choosing the right language and processor for your project
percent: 63
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


![When to Use What cover image](assets/when_to_use.jpg){:class="cover"}

## Introduction

You now understand both C++ and Python, plus the dual-processor architecture. But how do you decide which to use? This lesson provides a practical decision framework.

## Quick Decision Tree

```
Does your project need microsecond timing?
├─ YES → Use C++ on MCU
└─ NO  → Continue...

Does it need WiFi/Bluetooth?
├─ YES → Use Python on MPU (or hybrid)
└─ NO  → Continue...

Is RAM very limited (<32KB)?
├─ YES → Use C++
└─ NO  → Continue...

Do you need rapid prototyping?
├─ YES → Start with Python
└─ NO  → Either works

Need both real-time AND WiFi?
└─ YES → Use BOTH (hybrid system)
```

## Case Study 1: Line-Following Robot

**Requirements:**
- Read 8 line sensors at 100 Hz
- PID motor control
- Simple algorithm

**Decision: C++ only**

**Why:**
- Real-time sensor reading
- PID needs consistent timing
- Algorithm is simple enough for C++
- No networking needed
- Fits in 32KB RAM

## Case Study 2: Weather Station

**Requirements:**
- Read temp/humidity every 10 seconds
- Upload to cloud via WiFi
- Display on OLED

**Decision: Python on MPU**

**Why:**
- No real-time requirements
- WiFi needed
- Easy data formatting with f-strings
- Can use MicroPython libraries

## Case Study 3: Maze-Solving Robot

**Requirements:**
- Read distance sensors at 50 Hz
- Complex pathfinding algorithm
- Motor control with encoders
- Web interface for monitoring

**Decision: Hybrid (Both)**

**MCU (C++):**
- Sensor reading
- Motor PID control
- Emergency stop

**MPU (Python):**
- Maze algorithm
- Web server
- Data visualization

## Try It Yourself

For each project, decide: C++, Python, or Both?

1. **Smart doorbell** - Camera, motion sensor, WiFi, push notifications
2. **Quadcopter** - IMU at 1kHz, motor ESCs, maybe GPS
3. **Plant watering** - Soil sensor daily, pump control, log to file
4. **Robot arm** - 6 servos, inverse kinematics, teach mode

---

> **Previous:** [MCU-MPU Communication](/learn/arduino_to_python/19_mpu_mcu_communication.html) |
> **Next:** [Project Overview](/learn/arduino_to_python/21_project_overview.html)
>
> **Course Home:** [Arduino to Python](/learn/arduino_to_python/)
