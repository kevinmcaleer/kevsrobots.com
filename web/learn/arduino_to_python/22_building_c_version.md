---
layout: lesson
title: Building the C++ Version
author: Kevin McAleer
type: page
cover: /learn/arduino_to_python/assets/cpp_version.jpg
date: 2025-01-20
previous: 21_project_overview.html
next: 23_building_python_version.html
description: Implementing wall-following robot in Arduino C++
percent: 69
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


![C++ Version cover image](assets/cpp_version.jpg){:class="cover"}

## Complete Arduino C++ Implementation

```cpp
// Motor pins
const int MOTOR_LEFT_FWD = 5;
const int MOTOR_LEFT_BWD = 6;
const int MOTOR_RIGHT_FWD = 9;
const int MOTOR_RIGHT_BWD = 10;

// Sensor pins
const int TRIG_LEFT = 2;
const int ECHO_LEFT = 3;
const int TRIG_FRONT = 7;
const int ECHO_FRONT = 8;

const int BASE_SPEED = 150;
const int TURN_SPEED = 100;

void setup() {
  pinMode(MOTOR_LEFT_FWD, OUTPUT);
  pinMode(MOTOR_LEFT_BWD, OUTPUT);
  pinMode(MOTOR_RIGHT_FWD, OUTPUT);
  pinMode(MOTOR_RIGHT_BWD, OUTPUT);

  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

  Serial.begin(9600);
}

float readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  return duration * 0.034 / 2;
}

void moveForward() {
  analogWrite(MOTOR_LEFT_FWD, BASE_SPEED);
  analogWrite(MOTOR_RIGHT_FWD, BASE_SPEED);
  analogWrite(MOTOR_LEFT_BWD, 0);
  analogWrite(MOTOR_RIGHT_BWD, 0);
}

void turnRight() {
  analogWrite(MOTOR_LEFT_FWD, TURN_SPEED);
  analogWrite(MOTOR_RIGHT_BWD, TURN_SPEED);
  analogWrite(MOTOR_LEFT_BWD, 0);
  analogWrite(MOTOR_RIGHT_FWD, 0);
}

void loop() {
  float leftDist = readDistance(TRIG_LEFT, ECHO_LEFT);
  float frontDist = readDistance(TRIG_FRONT, ECHO_FRONT);

  if (frontDist < 20 && frontDist > 0) {
    turnRight();
    delay(500);
  } else if (leftDist < 15 && leftDist > 0) {
    // Too close - turn right slightly
    analogWrite(MOTOR_LEFT_FWD, BASE_SPEED);
    analogWrite(MOTOR_RIGHT_FWD, BASE_SPEED - 50);
  } else if (leftDist > 25 || leftDist == 0) {
    // Too far - turn left slightly
    analogWrite(MOTOR_LEFT_FWD, BASE_SPEED - 50);
    analogWrite(MOTOR_RIGHT_FWD, BASE_SPEED);
  } else {
    moveForward();
  }

  delay(50);
}
```

## Test and Tune

1. Upload code
2. Place robot near wall
3. Adjust speed constants
4. Fine-tune distance thresholds

---

> **Previous:** [Project Overview](/learn/arduino_to_python/21_project_overview.html) |
> **Next:** [Building Python Version](/learn/arduino_to_python/23_building_python_version.html)
>
> **Course Home:** [Arduino to Python](/learn/arduino_to_python/)
