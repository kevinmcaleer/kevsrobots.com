---
layout: lesson
title: Building the Python Version
author: Kevin McAleer
type: page
cover: /learn/arduino_to_python/assets/python_version.jpg
date: 2025-01-20
previous: 22_building_c_version.html
next: 24_hybrid_approach.html
description: Implementing wall-following robot in MicroPython
percent: 72
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


![Python Version cover image](assets/python_version.jpg){:class="cover"}

## Complete MicroPython Implementation

```python
from machine import Pin, PWM
import time

class UltrasonicSensor:
    def __init__(self, trig_pin, echo_pin):
        self.trig = Pin(trig_pin, Pin.OUT)
        self.echo = Pin(echo_pin, Pin.IN)

    def read_distance(self):
        self.trig.value(0)
        time.sleep_us(2)
        self.trig.value(1)
        time.sleep_us(10)
        self.trig.value(0)

        timeout = 30000
        start = time.ticks_us()
        while self.echo.value() == 0:
            if time.ticks_diff(time.ticks_us(), start) > timeout:
                return 0

        pulse_start = time.ticks_us()

        while self.echo.value() == 1:
            if time.ticks_diff(time.ticks_us(), pulse_start) > timeout:
                return 0

        pulse_end = time.ticks_us()
        duration = time.ticks_diff(pulse_end, pulse_start)
        return duration * 0.034 / 2

class Robot:
    def __init__(self):
        self.motor_left_fwd = PWM(Pin(5))
        self.motor_left_bwd = PWM(Pin(6))
        self.motor_right_fwd = PWM(Pin(9))
        self.motor_right_bwd = PWM(Pin(10))

        for motor in [self.motor_left_fwd, self.motor_left_bwd,
                      self.motor_right_fwd, self.motor_right_bwd]:
            motor.freq(1000)

        self.sensor_left = UltrasonicSensor(2, 3)
        self.sensor_front = UltrasonicSensor(7, 8)

        self.base_speed = 150
        self.turn_speed = 100

    def move_forward(self):
        speed = int(self.base_speed / 255 * 65535)
        self.motor_left_fwd.duty_u16(speed)
        self.motor_right_fwd.duty_u16(speed)
        self.motor_left_bwd.duty_u16(0)
        self.motor_right_bwd.duty_u16(0)

    def turn_right(self):
        speed = int(self.turn_speed / 255 * 65535)
        self.motor_left_fwd.duty_u16(speed)
        self.motor_right_bwd.duty_u16(speed)
        self.motor_left_bwd.duty_u16(0)
        self.motor_right_fwd.duty_u16(0)

    def run(self):
        while True:
            left_dist = self.sensor_left.read_distance()
            front_dist = self.sensor_front.read_distance()

            if 0 < front_dist < 20:
                self.turn_right()
                time.sleep(0.5)
            elif 0 < left_dist < 15:
                # Too close
                speed_left = int(self.base_speed / 255 * 65535)
                speed_right = int((self.base_speed - 50) / 255 * 65535)
                self.motor_left_fwd.duty_u16(speed_left)
                self.motor_right_fwd.duty_u16(speed_right)
            elif left_dist > 25 or left_dist == 0:
                # Too far
                speed_left = int((self.base_speed - 50) / 255 * 65535)
                speed_right = int(self.base_speed / 255 * 65535)
                self.motor_left_fwd.duty_u16(speed_left)
                self.motor_right_fwd.duty_u16(speed_right)
            else:
                self.move_forward()

            time.sleep(0.05)

# Run robot
robot = Robot()
robot.run()
```

## Comparison with C++

**Python advantages:**
- Object-oriented structure
- More readable
- Easier to modify

**C++ advantages:**
- Slightly faster
- Uses less RAM

Both work great for this application!

---

> **Previous:** [Building C++ Version](/learn/arduino_to_python/22_building_c_version.html) |
> **Next:** [Hybrid Approach](/learn/arduino_to_python/24_hybrid_approach.html)
>
> **Course Home:** [Arduino to Python](/learn/arduino_to_python/)
