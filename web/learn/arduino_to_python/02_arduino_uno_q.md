---
layout: lesson
title: Understanding the Arduino Uno Q Architecture
author: Kevin McAleer
type: page
cover: /learn/arduino_to_python/assets/uno_q_architecture.jpg
date: 2025-01-20
previous: 01_why_python.html
next: 03_setup_environment.html
description: Exploring the dual-processor design and when to use each brain
percent: 9
duration: 11
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


![Arduino Uno Q Architecture cover image](assets/uno_q_architecture.jpg){:class="cover"}

## Introduction

The Arduino Uno Q represents a fundamental shift in Arduino's design philosophy. For 15 years, Arduino boards had one processor doing everything. The Q breaks that pattern by including **two separate processors** - essentially two computers on one board.

This isn't just a spec upgrade; it's a architectural revolution that lets you run C++ and Python **simultaneously** on the same hardware, each doing what it does best.

In this lesson, you'll understand exactly how this dual-processor system works, how the two brains communicate, and when to leverage this powerful architecture.

## The Traditional Single-Processor Model

Let's start with what you already know. A traditional Arduino (Uno R3, Nano, Mega) has:

```
┌─────────────────────────────┐
│  ATmega328P (or similar)    │
│                             │
│  • Runs Arduino C++ code    │
│  • Direct pin control       │
│  • Real-time operations     │
│  • 16 MHz, 2KB RAM          │
└─────────────────────────────┘
         ↓
    GPIO Pins → Hardware
```

**One processor does everything:**
- Read sensors
- Run algorithms
- Control motors
- Handle communication
- Manage timing

This works beautifully for many projects. But it has limitations:
- Limited RAM (2KB on Uno R3)
- No built-in WiFi or advanced connectivity
- Can't easily run high-level languages like Python
- Processing power constrained

## The Dual-Processor Revolution

The Arduino Uno Q WiFi takes a completely different approach:

```
┌──────────────────────────────────────────┐
│                Arduino Uno Q             │
│                                          │
│  ┌─────────────┐      ┌──────────────┐   │
│  │ STM32U585   │◄────►│  Qualcomm    │   │
│  │             │ UART │  Dragonwing  │   │
│  │             │      │  QRB2210     │   │
│  │ • Arduino   │      │ • WiFi 5     │   │
│  │   C++ code  │      │ • Python     │   │
│  │ • Real-time │      │ • Processing │   │
│  │ • 160 MHz   │      │ • 2.0 GHz    │   │
│  │ • 786KB RAM │      │ • 2GB RAM    │   │
│  └─────────────┘      └──────────────┘   │
│         ↓                                │
│    Most GPIO pins                        │
└──────────────────────────────────────────┘
```

**Two processors, each with a specific role:**

### Main MCU: STM32U585 (The Real-Time Brain)
This is the primary microcontroller that handles Arduino C++ code and direct hardware control.

**Specifications:**
- 160 MHz ARM Cortex-M4 processor
- 2 MB flash memory
- 786 KB SRAM
- Runs Arduino C++ code
- Connects to most GPIO pins

**Best for:**
- Real-time motor control
- Precise timing operations
- Direct sensor reading
- Interrupt handling
- Traditional Arduino tasks

### Co-Processor: Qualcomm Dragonwing QRB2210 (The Smart Brain)
This is a powerful microprocessor that can run Python and handles WiFi/Bluetooth connectivity.

**Specifications:**
- 2.0 GHz quad-core ARM Cortex-A53 processor
- 16 GB eMMC
- 2 GB LPDDR4
- Dual-band WiFi 5 (2.4/5 GHz)
- Bluetooth 5.1
- Runs full Python

**Best for:**
- WiFi and Bluetooth communication
- Web servers and API calls
- Complex data processing
- Python-based algorithms
- High-level decision making

## How the Two Processors Communicate

The two processors communicate via **UART (serial communication)**. Think of it as a high-speed conversation:

```
STM32U585 (C++)              QRB2210 (Python)
     │                            │
     │  "SENSOR:425"              │
     ├───────────────────────────►│
     │                            │ Process data
     │                            │ Make decision
     │    "MOTOR:80,CCW"          │
     │◄────────────────────────── │
     │                            │
   Execute                     Continue
   command                     monitoring
```

**Example C++ code on STM32U585:**
```cpp
// STM32U585 - Send sensor data to QRB2210
void loop() {
  int sensorValue = analogRead(A0);

  // Send to QRB2210 via Serial1
  Serial1.print("SENSOR:");
  Serial1.println(sensorValue);

  // Listen for commands from QRB2210
  if (Serial1.available()) {
    String command = Serial1.readStringUntil('\n');
    // Parse and execute command
    executeCommand(command);
  }

  delay(100);
}
```

**Example Python code on QRB2210:**
```python
# QRB2210 - Receive sensor data, send commands
from machine import UART
import time

uart = UART(1, baudrate=115200)

while True:
    if uart.any():
        data = uart.readline().decode('utf-8').strip()

        if data.startswith('SENSOR:'):
            value = int(data.split(':')[1])

            # Process with Python (easy string manipulation!)
            if value > 512:
                # Send command back to STM32U585
                uart.write('MOTOR:80,CCW\n')
            else:
                uart.write('MOTOR:40,CW\n')

    time.sleep(0.1)
```

## Understanding the Memory Hierarchy

The two processors have **separate, independent memory spaces**:

```
STM32U585 Memory             QRB2210 Memory
┌────────────────┐           ┌────────────────┐
│  Your C++ code │           │ Your Python    │
│  and variables │           │ code and data  │
│                │           │                │
│  786 KB RAM    │           │   2 GB RAM     │
└────────────────┘           └────────────────┘
       ↑                            ↑
       │                            │
  No direct access          No direct access
```

**Critical implications:**
1. **Variables are not shared** - You can't access a C++ variable from Python
2. **Communication is explicit** - Data must be sent via serial messages
3. **Each side manages its own memory** - No shared heap or stack
4. **Design requires planning** - You must define your communication protocol

This is actually a **benefit** for robustness:
- A crash in Python code doesn't affect real-time C++ motor control
- Each processor can be debugged independently
- Clear separation of concerns

## Power Management

Both processors share the same power supply, but can operate semi-independently:

**Power options:**
- USB power (5V, both processors run)
- External DC power via barrel jack (7-12V, both processors run)
- Battery power with low-power modes (QRB2210 can sleep while STM32U585 stays active)

**Typical power consumption:**
- Both active: ~300-600 mA
- QRB2210 in light sleep: ~150 mA
- QRB2210 in deep sleep: ~10 mA
- STM32U585 only: ~30-70 mA

For battery-powered projects, you can put the QRB2210 to sleep and wake it periodically:

```python
# QRB2210 deep sleep (Python)
import machine

# Do some processing...
process_data()

# Sleep for 60 seconds, STM32U585 keeps running
machine.deepsleep(60000)  # milliseconds
```

The STM32U585 continues operating during QRB2210 sleep, maintaining real-time control.

## Bootloading and Programming

Each processor is programmed separately:

### Programming the STM32U585 (C++)
1. Connect via USB
2. Select "Arduino Uno Q WiFi" in Arduino IDE
3. Upload sketch as normal
4. Your C++ code runs on the STM32U585

### Programming the QRB2210 (Python)
1. Put QRB2210 in "download mode" (varies by board)
2. Use Thonny, ampy, or esptool to upload Python files
3. Your Python code runs on the QRB2210

**Important:** Most users start by only programming the STM32U585 with C++. The QRB2210 comes with firmware that handles WiFi/BLE commands from the C++ side. You only need to program the QRB2210 directly when you want custom Python code.

## When to Use Dual Processor vs Single Processor

### Use Both Processors When:
- You need WiFi/BLE **and** real-time control
- Complex data processing alongside hardware control
- You want to leverage Python libraries with Arduino hardware
- Web interface with precise motor control
- Data logging with time-critical operations

**Example projects:**
- Robot with computer vision (Python) and PID motor control (C++)
- IoT sensor node with local control and cloud reporting
- 3D printer with web interface and stepper control

### Use Only STM32U585 (C++) When:
- No WiFi/Bluetooth required
- Project is simple and fits in 786KB RAM
- Maximum power efficiency needed
- Pure real-time control application
- Traditional Arduino project

**Example projects:**
- Line-following robot
- LED matrix display
- Basic servo control
- Temperature monitoring

### Use Only QRB2210 (Python) When:
- Real-time precision not critical
- Leveraging WiFi/BLE is the main goal
- Rapid prototyping phase
- You prefer Python for everything

**Example projects:**
- Weather station with web dashboard
- Bluetooth remote control
- API data fetching and display

## Real-World Architecture Examples

### Example 1: Smart Robot Vacuum

**STM32U585 (C++) handles:**
- Motor speed control with PID loops (1kHz)
- Wheel encoder reading
- Cliff sensor monitoring (must be instant!)
- Emergency stop
- Brush motor control

**QRB2210 (Python) handles:**
- SLAM algorithm for mapping
- Path planning
- WiFi for smartphone app
- Scheduling cleaning times
- Uploading maps to cloud

**Why split:** Motor control needs microsecond timing. Mapping algorithms are complex but don't need real-time speed. Perfect fit for dual processor.

### Example 2: Environmental Monitor

**STM32U585 (C++) handles:**
- Reading temperature/humidity sensor
- Controlling fan based on temperature
- Local LCD display updates
- Data buffering

**QRB2210 (Python) handles:**
- WiFi connection to time server
- Publishing data to MQTT broker
- Checking for firmware updates
- Web server for configuration

**Why split:** Sensor reading and fan control need reliable timing. Web services and WiFi are easier in Python. Best of both worlds.

### Example 3: CNC Plotter

**STM32U585 (C++) handles:**
- Stepper motor pulse generation (precise timing!)
- Endstop monitoring
- Emergency stop button
- Real-time position tracking

**QRB2210 (Python) handles:**
- Parsing G-code files from SD card
- WiFi file uploads
- Web-based control interface
- Job queue management
- Progress reporting

**Why split:** Stepper timing must be precise (microseconds matter). G-code parsing is complex string manipulation (Python's strength).

## The Communication Protocol Design Pattern

For dual-processor projects, establish a clear protocol:

### Simple Command Protocol
```
Format: COMMAND:param1,param2,param3\n

Examples:
  C++ → Python: "SENSOR:425\n"
  Python → C++: "MOTOR:255,FWD\n"
  C++ → Python: "BUTTON:1\n"
  Python → C++: "LED:0,128,255\n"  # RGB values
```

### JSON Protocol (More Complex Projects)
```python
# Python side
import json

command = {
    'type': 'motor_control',
    'left': 80,
    'right': 75,
    'duration': 1000
}

uart.write(json.dumps(command) + '\n')
```

```cpp
// C++ side
#include <ArduinoJson.h>

String jsonStr = Serial1.readStringUntil('\n');
DynamicJsonDocument doc(256);
deserializeJson(doc, jsonStr);

int left = doc["left"];
int right = doc["right"];
```

## Try It Yourself

### Exercise 1: Architecture Analysis
For each task below, decide which processor should handle it:

1. Reading an ultrasonic sensor 50 times per second
2. Parsing JSON weather data from an API
3. Controlling a servo to sweep 0-180 degrees
4. Logging sensor data to a CSV file
5. Implementing PID control for a motor
6. Connecting to a WiFi network
7. Triggering an action when a button is pressed
8. Running a neural network inference

### Exercise 2: Communication Protocol Design
Design a communication protocol for a weather station robot:
- Temperature, humidity, pressure sensors on Renesas
- ESP32 publishes to MQTT broker
- Phone app can request immediate sensor reading
- Alarm thresholds set via web interface

What messages would each processor send? What format would you use?

### Exercise 3: Your Project Architecture
Think about a project you want to build. Draw a diagram showing:
- Which tasks go on Renesas (C++)
- Which tasks go on ESP32 (Python)
- What data is communicated between them
- Message format for communication

## Common Questions

### "Can I Run Python on the STM32U585?"
**No.** The STM32U585 runs Arduino C++ code only. Python runs on the QRB2210. This is by design - the STM32U585 is optimized for real-time control.

### "Can the Processors Access the Same Pins?"
**Mostly no.** Most GPIO pins connect to the STM32U585. Some specific pins connect to the QRB2210. Check the pinout diagram. This prevents conflicts.

### "What If One Processor Crashes?"
They're independent. If Python crashes on the QRB2210, your C++ motor control on the STM32U585 keeps running. This is a huge safety benefit for robots.

### "Is This Like Arduino Yún?"
Similar concept, but the Yún used Linux on the secondary processor. The Q's QRB2210 is more integrated, lower latency, and can run Python or C++.

### "Do I Need to Use Both?"
**No!** You can use the Q like a traditional Arduino, programming only the STM32U585. The QRB2210 can sit idle, or run pre-installed firmware for WiFi support.

## Summary

The Arduino Uno Q's dual-processor architecture represents modern embedded system design:

**STM32U585 (Main MCU):**
- Arduino C++ code
- Real-time control
- Direct hardware access
- 160 MHz, 786KB RAM
- Most GPIO pins

**QRB2210 (Co-Processor):**
- Python code capability
- WiFi 5 and Bluetooth 5.1
- High-level processing
- 2.0 GHz quad-core, 2GB RAM
- Networking features

**Communication:**
- UART serial connection
- Separate memory spaces
- Explicit message passing
- Designed communication protocols

**When to Use Both:**
- Complex projects needing both real-time control and high-level processing
- WiFi/BLE with precise timing requirements
- Leveraging each language's strengths

In the next lesson, we'll get hands-on and set up your development environment for both Arduino C++ and MicroPython, then write your first dual-language program.

---

> **Previous Lesson:** [Why Python?](/learn/arduino_to_python/01_why_python.html) |
> **Next Lesson:** [Setup Environment](/learn/arduino_to_python/03_setup_environment.html)
>
> **Course Home:** [Arduino to Python](/learn/arduino_to_python/)
