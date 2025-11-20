---
title: Why MicroPython?
description: Understand when to use MicroPython vs C++ for your microcontroller projects and make the right choice for your needs
layout: lesson
type: page
cover: /learn/arduino_to_python/assets/cover.jpg
---

## Why Learn MicroPython?

If you're coming from Arduino's C++ world, you might wonder why you'd switch to MicroPython. The answer isn't "always use Python" - it's about picking the right tool for your project.

MicroPython brings Python's simplicity to microcontrollers like the Arduino Nano ESP32, Raspberry Pi Pico W, and Pico 2W. You get rapid development, interactive debugging, and readable code without sacrificing too much performance.

## The Big Difference

Here's the core philosophical shift:

**Arduino C++**: Compile code on your computer, upload binary to board, hope it works, repeat.

**MicroPython**: Upload code as text files, run instantly, test in real-time with REPL, fix bugs on the fly.

That REPL (Read-Eval-Print Loop) is a game changer. You can type commands directly to your microcontroller and see instant results - like having a conversation with your hardware.

## When to Use MicroPython

**Choose MicroPython when:**

- You're prototyping and want fast iteration
- You need to debug interactively (REPL is amazing)
- You're building IoT devices with WiFi/networking
- You want to process data or handle complex logic
- You're teaching programming or building educational projects
- Development speed matters more than peak performance
- You're comfortable with Python syntax

**Example perfect projects:**
- WiFi weather station
- Home automation controller
- Sensor data logger
- Web-controlled robot
- Environmental monitor

## When to Stick with C++

**Choose Arduino C++ when:**

- You need absolute maximum performance
- You're working with time-critical operations (microsecond precision)
- You need the smallest possible memory footprint
- You're using specialized Arduino libraries not available in MicroPython
- You're building battery-powered devices where every milliamp counts
- You need to integrate with existing C++ codebases

**Example perfect projects:**
- High-speed motor control (PID loops)
- Audio processing
- LED matrix animations (tight timing)
- Ultra-low-power battery devices

## The Performance Trade-off

Let's be honest: MicroPython is slower than C++. But here's the secret - for most projects, you won't notice.

**Speed comparison:**
- C++: Blink LED every 1ms? No problem.
- MicroPython: Blink LED every 1ms? Also no problem (usually).

**Where you'll notice the difference:**
- Processing thousands of sensor readings per second
- Bit-banging complex protocols
- Mathematical operations in tight loops
- Audio/video processing

For reading a temperature sensor every second and sending it over WiFi? MicroPython wins on development time.

## Code Comparison: First Impressions

Let's see what the same simple program looks like:

**Arduino C++:**
```cpp
// Blink built-in LED
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}
```

**MicroPython:**
```python
# Blink built-in LED
from machine import Pin
import time

led = Pin(25, Pin.OUT)  # Pin 25 on Pico

while True:
    led.on()
    time.sleep(1)
    led.off()
    time.sleep(1)
```

Notice:
- No semicolons in Python
- No `setup()` and `loop()` - just regular Python code
- More readable function names (`led.on()` vs `digitalWrite`)
- Simpler overall structure

## Hardware Compatibility

This course focuses on three excellent MicroPython-compatible boards:

**Arduino Nano ESP32**
- Dual-core ESP32-S3 processor
- Built-in WiFi and Bluetooth
- Familiar Arduino form factor
- Perfect for IoT projects

**Raspberry Pi Pico W**
- RP2040 processor with WiFi
- Ultra-affordable (around $6)
- Massive community support
- Great for learning

**Raspberry Pi Pico 2W**
- Faster RP2350 processor with WiFi
- More memory and features
- Backwards compatible with Pico W
- Future-proof choice

All three run MicroPython beautifully and share similar APIs.

> ## What About CircuitPython?
>
> You might have heard of **CircuitPython** - another Python implementation for microcontrollers. Here's what you need to know:
>
> ### What is CircuitPython?
> - A beginner-friendly fork of MicroPython created by Adafruit
> - Designed specifically for education and maker projects
> - Focuses on simplicity and ease of use over raw performance
>
> **Key Differences:**
> - **File system**: CircuitPython shows up as a USB drive (drag-and-drop code!)
> - **File names**: Uses `code.py` instead of `main.py`
> - **Libraries**: Pre-installed libraries for Adafruit hardware
> - **Development**: Optimized for beginners, simpler workflow
> - **Hardware**: Best support for Adafruit boards, good on Pico too
>
> **Pros of CircuitPython:**
> - ✅ Easier for absolute beginners (drag-and-drop files)
> - ✅ Excellent hardware abstraction
> - ✅ Great documentation and community support
> - ✅ Pre-built libraries for common sensors
>
> **Cons of CircuitPython:**
> - ❌ Slower than MicroPython (prioritizes ease over speed)
> - ❌ Larger memory footprint
> - ❌ Less flexible for advanced use cases
> - ❌ Smaller device support (focused on specific boards)
>
> **Which Should You Learn?**
> - **MicroPython**: More versatile, works on more boards, better performance
> - **CircuitPython**: Easier onboarding, better for teaching kids, excellent for Adafruit hardware
>
> The good news: they're ~95% compatible. Learn one, and you mostly know the other!
>
> This course focuses on **MicroPython** because it works on Arduino Nano ESP32, all Raspberry Pi Pico variants, and gives you more flexibility. But everything you learn applies to CircuitPython too!

## Decision Guide

Ask yourself these questions:

**1. Do I need to finish this project quickly?**
- Yes → MicroPython (faster development)
- No → Either works

**2. Will I need to debug complex behavior?**
- Yes → MicroPython (REPL makes debugging easier)
- No → Either works

**3. Am I doing time-critical operations under 1ms?**
- Yes → C++ (better timing precision)
- No → Either works

**4. Will this run on battery for months?**
- Yes → C++ (lower power consumption)
- No → Either works

**5. Am I more comfortable with Python or C++?**
- Python → MicroPython (stick with what you know)
- C++ → C++ or try MicroPython (learning curve is gentle)

## What You'll Learn in This Course

This course will teach you:

- How Python syntax differs from C++ (no semicolons, indentation matters)
- Converting Arduino code patterns to MicroPython
- Pin control, analog reading, PWM in Python
- Timing and delays (seconds vs milliseconds - important!)
- Serial communication and debugging with REPL
- Building an IoT temperature monitor project
- Building a simple robot control project

By the end, you'll be able to translate any Arduino project into MicroPython and know when each approach is best.

## Try It Yourself

**Exercise 1: Evaluate Your Next Project**

Think of a project you want to build. Answer these questions:
- Does it need WiFi or networking?
- Will you need to debug complex behavior?
- Does it need microsecond-level timing?
- Are you more comfortable in Python or C++?

Based on your answers, which language would you choose?

**Exercise 2: Explore the REPL**

If you have a Pico, Pico 2W, or Nano ESP32 with MicroPython installed:
1. Connect via USB
2. Open a serial terminal (Thonny, PuTTY, or screen)
3. Type `print("Hello from MicroPython!")` and press Enter
4. Try `1 + 1` and see the instant result

This is the REPL in action - something you can't do with Arduino C++!

**Exercise 3: Compare Code**

Look at an Arduino sketch you've written before. Identify:
- Where are the semicolons?
- Where are the curly braces `{}`?
- What does `setup()` do? What does `loop()` do?

In the next lessons, you'll learn how to convert each of these to MicroPython.

