---
layout: lesson
title: Understanding the Dual-Processor System
author: Kevin McAleer
type: page
cover: /learn/arduino_to_python/assets/dual_processor.jpg
date: 2025-01-20
previous: 17_file_systems.html
next: 19_mpu_mcu_communication.html
description: Deep dive into Arduino Uno R4's MCU and MPU architecture
percent: 57
duration: 8
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


![Understanding Dual Processor cover image](assets/dual_processor.jpg){:class="cover"}

## Introduction

We introduced the Arduino Uno Q's dual-processor architecture earlier in the course. Now it's time to understand it in depth so you can make informed decisions about how to use both processors effectively.

The Q isn't just "an Arduino with WiFi" - it's a sophisticated system with two independent processors that can run different code simultaneously. Understanding their strengths, limitations, and communication methods is key to building powerful hybrid systems.

## Architecture Review

```
Arduino Uno Q WiFi
┌────────────────────────────────────────────┐
│                                            │
│  ┌──────────────┐        ┌──────────────┐ │
│  │ STM32U585    │ UART   │ Qualcomm     │ │
│  │ (MCU)        │◄──────►│ QRB2210(MPU) │ │
│  │              │        │              │ │
│  │ • 160 MHz    │        │ • 2.0 GHz    │ │
│  │ • 2MB Flash  │        │ • 16GB eMMC  │ │
│  │ • 786KB RAM  │        │ • 2GB RAM    │ │
│  │ • Arduino C++│        │ • Python     │ │
│  │ • Real-time  │        │ • WiFi 5/BT  │ │
│  └──────────────┘        └──────────────┘ │
│         ↓                       ↓          │
│    Digital I/O            WiFi Antenna     │
└────────────────────────────────────────────┘
```

## Processor Characteristics

### STM32U585 (Main MCU)

**Purpose:** Real-time control and hardware interfacing

**Strengths:**
- Precise timing (microsecond level)
- Direct GPIO control
- Low latency interrupt response
- Deterministic execution
- No garbage collection pauses

**Best for:**
- Motor control with PID
- Servo positioning
- Sensor sampling at high rates
- Time-critical safety features
- PWM generation
- I2C/SPI device control

**Limitations:**
- Limited RAM compared to MPU (786 KB)
- No networking hardware
- Lower clock speed compared to MPU (160 MHz)

### QRB2210 (Co-Processor MPU)

**Purpose:** High-level processing and connectivity

**Strengths:**
- Fast processor (2.0 GHz quad-core ARM Cortex-A53)
- Large RAM (2 GB LPDDR4)
- Built-in dual-band WiFi 5
- Bluetooth 5.1
- Can run Python or C++

**Best for:**
- WiFi communication
- Web servers
- Cloud connectivity
- Complex algorithms
- Data processing
- MQTT / HTTP clients
- Machine learning inference

**Limitations:**
- Not connected to most GPIO pins
- Sharing with WiFi stack reduces available resources
- Less deterministic (WiFi interrupts)

## Task Allocation Strategy

### Decision Matrix

| Task Type | MCU (STM32U585) | MPU (QRB2210) | Why? |
|-----------|-----------------|---------------|------|
| Read sensors < 100 Hz | ✅ Preferred | ⚠️ Possible | MCU has direct hardware access |
| Read sensors > 1 kHz | ✅ Required | ❌ No | Need microsecond timing |
| Motor PID control | ✅ Required | ❌ No | Real-time critical |
| LED blinking | ✅ Either | ✅ Either | Not critical |
| WiFi communication | ❌ No hardware | ✅ Required | Only MPU has WiFi |
| Parse JSON data | ⚠️ Limited RAM | ✅ Preferred | More RAM, easier in Python |
| Emergency stop | ✅ Required | ❌ Too slow | Must be instant |
| Web dashboard | ❌ No WiFi | ✅ Required | Needs network |
| Store data logs | ⚠️ No filesystem | ✅ Preferred | MPU has filesystem |

## Communication Channels

The two processors communicate via **UART (serial)**:

```
MCU (C++)                    MPU (Python)
    │                            │
    │  "SENSOR:425\n"           │
    ├──────────────────────────►│
    │                            │ Process & decide
    │    "MOTOR:180,FWD\n"       │
    │◄────────────────────────────┤
    │                            │
Execute                      Continue
command                      monitoring
```

**Communication characteristics:**
- **Baud rate:** Typically 115200 or higher
- **Latency:** ~1ms for small messages
- **Throughput:** ~11 KB/sec at 115200 baud
- **Reliability:** Very high (hardware UART)

## Power Management

### Power Consumption

| State | STM32U585 | QRB2210 | Total |
|-------|-----------|---------|-------|
| **Both active** | ~50 mA | ~300 mA | ~350 mA |
| **MCU active, MPU sleep** | ~50 mA | ~10 mA | ~60 mA |
| **MCU active, MPU deep sleep** | ~50 mA | ~20 µA | ~50 mA |
| **Both sleep** | ~2 mA | ~20 µA | ~2 mA |

**Battery life example (2000 mAh battery):**
- Both active: ~5.7 hours
- MCU only (MPU deep sleep): ~40 hours
- Both sleep: ~1000 hours (41 days)

### Power Management Strategy

```python
# QRB2210 (Python) - Sleep when idle
import machine
import time

def do_network_task():
    # Check server for commands
    # Upload sensor data
    pass

while True:
    do_network_task()

    # Sleep for 5 minutes
    machine.deepsleep(300000)  # milliseconds

# MCU (C++) keeps running sensors and motors
```

## Memory Architecture

### Separate Memory Spaces

```
┌─────────────────┐    ┌─────────────────┐
│ STM32U585 Memory│    │ QRB2210 Memory  │
│                 │    │                 │
│ int sensor = 42;│    │ sensor = 99     │
│                 │    │                 │
└─────────────────┘    └─────────────────┘
       ↑                      ↑
       │                      │
   Cannot access          Cannot access
   QRB2210 memory         STM32U585 memory
```

**Key implications:**
1. **No shared variables** - Data must be sent via UART
2. **Independent crashes** - If Python crashes, C++ keeps running
3. **Separate debugging** - Debug each processor independently
4. **Different toolchains** - Arduino IDE for MCU, Thonny for MPU

## Real-World System Designs

### Example 1: WiFi-Controlled Robot

**MCU (STM32U585) Responsibilities:**
- Read line sensors (10 Hz)
- Control motors with PID (100 Hz)
- Handle emergency stop button (interrupt)
- Execute movement commands
- Send sensor status to MPU

**MPU (QRB2210) Responsibilities:**
- WiFi AP or client mode
- Web server for control interface
- Receive commands from app
- Forward to MCU via UART
- Log telemetry data

**Message Protocol:**
```
MCU → MPU:  "STATUS:sensors=0110,speed=180,battery=7.4\n"
MPU → MCU:  "CMD:forward,150\n"
MPU → MCU:  "CMD:turn_left,100\n"
MCU → MPU:  "ALERT:low_battery\n"
```

### Example 2: Environmental Monitor

**MCU (STM32U585) Responsibilities:**
- Read temperature/humidity (1 Hz)
- Read air quality sensor (1 Hz)
- Control fan based on temperature
- Store readings in buffer (last 60)

**MPU (QRB2210) Responsibilities:**
- Connect to home WiFi
- Publish to MQTT broker (Home Assistant)
- Sync time with NTP server
- Receive threshold settings from cloud
- Log to SD card (optional)

**Data Flow:**
```
MCU reads sensors → Buffer → QRB2210 polls every 30s
                                ↓
                          Publish to MQTT
                                ↓
                          Home Assistant Dashboard
```

### Example 3: Autonomous Maze Solver

**MCU (STM32U585) Responsibilities:**
- Read 5× ultrasonic sensors (20 Hz)
- Execute motor commands precisely
- Maintain encoder position tracking
- Emergency obstacle detection

**MPU (QRB2210) Responsibilities:**
- Run maze-solving algorithm (A*)
- Build map of environment
- Calculate optimal path
- Send movement commands to MCU
- Visualize map on web interface

**Why split this way:**
- MCU handles real-time safety (obstacle avoidance)
- MPU handles complex algorithm (pathfinding)
- MCU never blocks waiting for algorithm
- System remains responsive

## Try It Yourself

### Exercise 1: Task Allocation
For each project, decide which processor handles which tasks:

**Project: Smart Thermostat**
- Read temperature sensor
- Control heating relay
- Display on LCD
- WiFi connection to phone app
- Web configuration interface
- Log temperature history

**Project: Line-Following Robot**
- Read 8× line sensors
- PID motor control
- Bluetooth control from app
- Battery monitoring
- Speed adjustment

### Exercise 2: Communication Protocol Design
Design a message protocol for a robot with:
- 4 distance sensors
- 2 motors with encoders
- 1 gripper servo
- Battery voltage monitoring

Define messages for:
- MCU → MPU: Sensor updates
- MPU → MCU: Control commands
- Error reporting
- Status requests

### Exercise 3: Power Budget
Calculate battery life for:
- 5000 mAh battery
- MCU always on (50 mA)
- MPU wakes every 10 minutes for 30 seconds
- MPU active: 300 mA
- MPU deep sleep: 20 µA

## Summary

**Key Takeaways:**
- **Two independent computers** on one board
- **MCU for real-time**, MPU for intelligence
- **Communicate via UART** messages
- **Separate memory** - no shared variables
- **Power management** - sleep MPU when idle
- **Reliability** - MCU keeps working if MPU crashes

**Design Principles:**
1. Safety-critical tasks → MCU
2. Network tasks → MPU
3. High-frequency control → MCU
4. Complex algorithms → MPU
5. Simple tasks → Either (choose based on workload)

**Benefits:**
- Best of both worlds
- Real-time + Intelligence
- Graceful degradation
- Power efficiency
- Parallel processing

In the next lesson, we'll implement the communication between MCU and MPU with practical examples.

---

> **Previous Lesson:** [File Systems](/learn/arduino_to_python/17_file_systems.html) |
> **Next Lesson:** [MCU-MPU Communication](/learn/arduino_to_python/19_mpu_mcu_communication.html)
>
> **Course Home:** [Arduino to Python](/learn/arduino_to_python/)
