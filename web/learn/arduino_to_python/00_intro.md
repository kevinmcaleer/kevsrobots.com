---
layout: lesson
title: Introduction to Arduino to MicroPython
author: Kevin McAleer
type: page
cover: /learn/arduino_to_python/assets/intro.jpg
date: 2025-11-20
next: 01_why_micropython.html
description: Quick start guide for Arduino developers learning MicroPython
percent: 8
duration: 3
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


![Introduction cover image](assets/banner01.jpg){:class="cover"}

## Welcome!

You already know how to make LEDs blink, read sensors, and control motors with Arduino C++. Now it's time to learn MicroPython - a powerful alternative that can make your projects faster to develop and easier to maintain.

This course is designed specifically for Arduino developers like you. We'll use side-by-side code comparisons so you can see exactly how your Arduino knowledge translates to MicroPython.

## What You'll Learn

By the end of this course, you'll be able to:

- ✅ Write MicroPython code for microcontrollers
- ✅ Control hardware (pins, ADC, PWM) with Python
- ✅ Choose between C++ and Python for your projects
- ✅ Build IoT projects with WiFi
- ✅ Create robot control systems in Python

## Target Hardware

This course works with any of these boards:

- **Arduino Nano ESP32** - Arduino with WiFi and MicroPython support
- **Raspberry Pi Pico W** - Popular, affordable, WiFi-enabled
- **Raspberry Pi Pico 2W** - Faster version with WiFi

All examples will work on these boards with minimal changes.

## Prerequisites

**You should already know:**
- Arduino C++ programming
- Basic electronics (LEDs, resistors, breadboards)
- How to upload sketches to Arduino

**You do NOT need:**
- Python experience (we'll teach you!)
- Linux or command line knowledge
- Advanced programming skills

## What You'll Need

**Hardware:**
- One of the boards listed above
- USB cable
- LEDs, resistors, breadboard
- (Optional) Sensors and modules you already have

**Software:**
- **Thonny IDE** - Free, beginner-friendly editor
- **MicroPython firmware** - Already installed if you followed our [MicroPython Installation Course](/learn/how_to_install_micropython/)

**Not installed yet?** Check out our quick installation guide: [How to Install MicroPython](/learn/how_to_install_micropython/)

## Course Structure

**Section 1**: Getting Started (2 lessons)
Quick introduction and why MicroPython matters

**Section 2**: Language Fundamentals (4 lessons)
Learn Python syntax with Arduino comparisons

**Section 3**: Hardware Basics (3 lessons)
Control pins, ADC, PWM, and timing

**Section 4**: Practical Projects (3 lessons)
Build real IoT and robotics projects

**Total time:** 2-3 hours for complete course

## How This Course Works

### Side-by-Side Comparisons

Every concept shows Arduino C++ and MicroPython side by side:

**Arduino C++:**
```cpp
void setup() {
  pinMode(13, OUTPUT);
}

void loop() {
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  delay(1000);
}
```

**MicroPython:**
```python
from machine import Pin
import time

led = Pin(25, Pin.OUT)

while True:
    led.on()
    time.sleep(1)
    led.off()
    time.sleep(1)
```

### Try It Yourself

Each lesson includes hands-on exercises you can do with your board.

### Keep It Simple

We focus on practical skills, not theory. You'll learn by doing.

## Why Learn MicroPython?

**Faster development:**
- No compile time - run code instantly
- Interactive REPL for testing
- Change code without re-uploading

**Easier to read:**
- Less boilerplate
- Clearer syntax
- Better for beginners teaching others

**Powerful features:**
- Built-in WiFi libraries (on ESP32/Pico W)
- File system for data logging
- Easy string manipulation

**When to still use Arduino C++:**
- Need microsecond timing
- Very limited RAM
- Existing Arduino libraries you need

## Ready to Start?

In the next lesson, we'll explore why MicroPython is worth learning and when to use it versus Arduino C++.

Let's go!
