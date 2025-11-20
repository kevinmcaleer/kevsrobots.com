---
layout: lesson
title: Hybrid Approach - Best of Both Worlds
author: Kevin McAleer
type: page
cover: /learn/arduino_to_python/assets/hybrid.jpg
date: 2025-01-20
previous: 23_building_python_version.html
next: 25_summary.html
description: Using C++ and Python together on Arduino Uno Q
percent: 75
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


![Hybrid Approach cover image](assets/hybrid.jpg){:class="cover"}

## Hybrid System Design

For Arduino Uno Q, split responsibilities:

**MCU (C++) handles:**
- Sensor reading at 20 Hz
- Motor control
- Emergency stop

**MPU (Python) handles:**
- Wall-following algorithm
- WiFi web interface
- Data logging

## MCU Code (C++)

```cpp
void loop() {
  float leftDist = readDistance(TRIG_LEFT, ECHO_LEFT);
  float frontDist = readDistance(TRIG_FRONT, ECHO_FRONT);

  // Send to MPU
  Serial1.print("SENSORS:");
  Serial1.print(leftDist);
  Serial1.print(",");
  Serial1.println(frontDist);

  // Receive commands from MPU
  if (Serial1.available()) {
    String cmd = Serial1.readStringUntil('\n');
    if (cmd.startsWith("MOTOR:")) {
      int left = cmd.substring(6, cmd.indexOf(',')).toInt();
      int right = cmd.substring(cmd.indexOf(',') + 1).toInt();
      setMotors(left, right);
    }
  }

  delay(50);
}
```

## MPU Code (Python)

```python
from machine import UART
import time

uart = UART(1, baudrate=115200)

while True:
    if uart.any():
        data = uart.readline().decode('utf-8').strip()
        if data.startsWith('SENSORS:'):
            left, front = map(float, data[8:].split(','))

            # Decision algorithm
            if front < 20:
                left_speed, right_speed = 100, -100  # Turn
            elif left < 15:
                left_speed, right_speed = 150, 100   # Slight right
            elif left > 25:
                left_speed, right_speed = 100, 150   # Slight left
            else:
                left_speed, right_speed = 150, 150   # Forward

            # Send command
            uart.write(f'MOTOR:{left_speed},{right_speed}\n')

    time.sleep(0.01)
```

## Advantages of Hybrid

1. Real-time sensor reading (C++)
2. Complex algorithm (Python)
3. Add WiFi control easily
4. Best tool for each task

---

> **Previous:** [Building Python Version](/learn/arduino_to_python/23_building_python_version.html) |
> **Next:** [Course Summary](/learn/arduino_to_python/25_summary.html)
>
> **Course Home:** [Arduino to Python](/learn/arduino_to_python/)
