---
layout: lesson
title: MCU-MPU Communication Patterns
author: Kevin McAleer
type: page
cover: /learn/arduino_to_python/assets/mcu_mpu_comm.jpg
date: 2025-01-20
previous: 18_understanding_dual_processor.html
next: 20_when_to_use_what.html
description: Implementing reliable communication between processors
percent: 60
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


![MCU-MPU Communication cover image](assets/mcu_mpu_comm.jpg){:class="cover"}

## Introduction

Now that you understand the dual-processor architecture, let's implement the communication between them. You'll learn practical patterns for sending commands, receiving data, and handling errors.

## Basic Communication Setup

### MCU Side (Arduino C++)
```cpp
void setup() {
  Serial.begin(115200);  // USB debug
  Serial1.begin(115200); // UART to ESP32
}

void loop() {
  // Send sensor data to ESP32
  int sensor = analogRead(A0);
  Serial1.print("SENSOR:");
  Serial1.println(sensor);
  
  // Check for commands from ESP32
  if (Serial1.available()) {
    String cmd = Serial1.readStringUntil('\n');
    handleCommand(cmd);
  }
  
  delay(100);
}

void handleCommand(String cmd) {
  if (cmd.startsWith("MOTOR:")) {
    int speed = cmd.substring(6).toInt();
    analogWrite(9, speed);
  }
}
```

### MPU Side (MicroPython)
```python
from machine import UART
import time

uart = UART(1, baudrate=115200)

while True:
    # Receive sensor data from MCU
    if uart.any():
        data = uart.readline().decode('utf-8').strip()
        if data.startswith('SENSOR:'):
            value = int(data.split(':')[1])
            print(f"Sensor: {value}")
    
    # Send command to MCU
    uart.write('MOTOR:180\n')
    
    time.sleep(1)
```

## Message Protocol Design

### Simple Protocol
```
Format: COMMAND:param1,param2,param3\n

Examples:
MCU → MPU: "SENSOR:425\n"
MCU → MPU: "BATTERY:7.4\n"
MPU → MCU: "MOTOR:180,FWD\n"
MPU → MCU: "LED:255,0,128\n"
```

### JSON Protocol (More Flexible)
```python
# MPU sends JSON
import json

command = {
    'type': 'motor',
    'left': 180,
    'right': 150,
    'duration': 1000
}

uart.write(json.dumps(command) + '\n')
```

```cpp
// MCU receives JSON
#include <ArduinoJson.h>

String jsonStr = Serial1.readStringUntil('\n');
DynamicJsonDocument doc(256);
deserializeJson(doc, jsonStr);

int left = doc["left"];
int right = doc["right"];
```

## Try It Yourself

Create a system where:
1. MCU reads 3 sensors and sends to MPU
2. MPU processes and sends motor commands
3. Implement error handling

---

> **Previous:** [Understanding Dual Processor](/learn/arduino_to_python/18_understanding_dual_processor.html) |
> **Next:** [When to Use What](/learn/arduino_to_python/20_when_to_use_what.html)
>
> **Course Home:** [Arduino to Python](/learn/arduino_to_python/)
