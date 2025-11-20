---
title: Introduction to Arduino to MicroPython
description: Quick start guide for Arduino developers learning MicroPython
layout: lesson
type: page
cover: /learn/arduino_to_python/assets/intro.jpg
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
