---
layout: lesson
title: Digital Pin Control
author: Kevin McAleer
type: page
cover: /learn/arduino_to_python/assets/cover.jpg
date: 2025-01-20
previous: 08_arrays_vs_lists.html
next: 10_analog_io.html
description: Master MicroPython's Pin class and learn how it differs from Arduino's
  pinMode(), digitalWrite(), and digitalRead() functions.
percent: 30
duration: 9
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


![Breadboard with LED circuits controlled by microcontroller](assets/pin_control.jpg){:class="cover"}

## From Functions to Objects

This is where Arduino developers encounter a fundamental paradigm shift: Arduino uses **global functions** (`digitalWrite()`, `pinMode()`), while MicroPython uses **object-oriented programming** with the `Pin` class.

Don't worry - it's actually simpler once you see it in action.

---

## Basic LED Blink - Side by Side

Let's start with the classic "Hello World" of microcontrollers.

**Arduino C++:**
```cpp
// The way you know
const int LED_PIN = 13;

void setup() {
  pinMode(LED_PIN, OUTPUT);  // Configure pin as output
}

void loop() {
  digitalWrite(LED_PIN, HIGH);  // Turn LED on
  delay(1000);                   // Wait 1 second
  digitalWrite(LED_PIN, LOW);    // Turn LED off
  delay(1000);                   // Wait 1 second
}
```

**MicroPython:**
```python
# The Python way
from machine import Pin
import time

led = Pin(13, Pin.OUT)  # Create Pin object, set as output

while True:
    led.value(1)      # Turn LED on
    time.sleep(1)     # Wait 1 second
    led.value(0)      # Turn LED off
    time.sleep(1)     # Wait 1 second
```

**Or the more Pythonic way:**
```python
from machine import Pin
import time

led = Pin(13, Pin.OUT)

while True:
    led.on()          # Explicit on() method
    time.sleep(1)
    led.off()         # Explicit off() method
    time.sleep(1)
```

**Or even more concise using toggle:**
```python
from machine import Pin
import time

led = Pin(13, Pin.OUT)

while True:
    led.toggle()      # Flip the current state
    time.sleep(1)
```

---

## Key Differences Explained

### 1. Import Statements vs #include

**Arduino C++:**
```cpp
// Arduino functions are globally available
// No include needed for basic pinMode/digitalWrite
```

**MicroPython:**
```python
# Must import the Pin class
from machine import Pin

# Or import the whole module
import machine
led = machine.Pin(13, machine.Pin.OUT)
```

**Why:** Python uses explicit imports to keep the namespace clean and show dependencies clearly.

---

### 2. Creating a Pin Object

**Arduino C++:**
```cpp
// Just use the pin number directly
pinMode(13, OUTPUT);
digitalWrite(13, HIGH);
```

**MicroPython:**
```python
# Create a Pin object first
led = Pin(13, Pin.OUT)

# Then use the object's methods
led.value(1)
```

**Why:** The object-oriented approach means each pin is a self-contained object with its own state and methods. This is cleaner when working with multiple pins.

---

### 3. Pin Modes

| Arduino C++ | MicroPython | Purpose |
|-------------|-------------|---------|
| `OUTPUT` | `Pin.OUT` | Pin outputs voltage |
| `INPUT` | `Pin.IN` | Pin reads voltage |
| `INPUT_PULLUP` | `Pin.PULL_UP` | Input with internal pull-up resistor |
| N/A | `Pin.PULL_DOWN` | Input with pull-down (not on all boards) |

**Arduino C++ example:**
```cpp
pinMode(BUTTON_PIN, INPUT_PULLUP);
```

**MicroPython equivalent:**
```python
button = Pin(14, Pin.IN, Pin.PULL_UP)
```

---

## Reading Digital Inputs - Button Example

**Arduino C++:**
```cpp
const int BUTTON_PIN = 2;
const int LED_PIN = 13;

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  int buttonState = digitalRead(BUTTON_PIN);

  if (buttonState == LOW) {  // Pressed (pulled to ground)
    digitalWrite(LED_PIN, HIGH);
    Serial.println("Button pressed!");
  } else {
    digitalWrite(LED_PIN, LOW);
  }

  delay(50);  // Debounce delay
}
```

**MicroPython:**
```python
from machine import Pin
import time

button = Pin(2, Pin.IN, Pin.PULL_UP)
led = Pin(13, Pin.OUT)

while True:
    button_state = button.value()

    if button_state == 0:  # Pressed (pulled to ground)
        led.on()
        print("Button pressed!")
    else:
        led.off()

    time.sleep(0.05)  # Debounce delay (50ms)
```

**More Pythonic version with direct boolean:**
```python
from machine import Pin
import time

button = Pin(2, Pin.IN, Pin.PULL_UP)
led = Pin(13, Pin.OUT)

while True:
    # Pin value can be used directly as boolean
    if not button.value():  # Active low (0 when pressed)
        led.on()
        print("Button pressed!")
    else:
        led.off()

    time.sleep(0.05)
```

---

## Real-World Robot Example: Motor Direction Control

Let's control a motor with two direction pins - a common robotics scenario.

**Arduino C++:**
```cpp
// Motor driver control pins
const int MOTOR_EN = 9;   // Enable (PWM)
const int MOTOR_IN1 = 7;  // Direction pin 1
const int MOTOR_IN2 = 8;  // Direction pin 2

void setup() {
  pinMode(MOTOR_EN, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
}

void motorForward(int speed) {
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_EN, speed);  // 0-255
}

void motorReverse(int speed) {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  analogWrite(MOTOR_EN, speed);
}

void motorStop() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_EN, 0);
}

void loop() {
  motorForward(200);
  delay(2000);
  motorStop();
  delay(1000);
  motorReverse(150);
  delay(2000);
  motorStop();
  delay(1000);
}
```

**MicroPython:**
```python
from machine import Pin, PWM
import time

# Motor driver control pins
motor_enable = PWM(Pin(9))
motor_in1 = Pin(7, Pin.OUT)
motor_in2 = Pin(8, Pin.OUT)

motor_enable.freq(1000)  # 1kHz PWM frequency

def motor_forward(speed):
    """Run motor forward at specified speed (0-65535)"""
    motor_in1.on()
    motor_in2.off()
    motor_enable.duty_u16(speed)

def motor_reverse(speed):
    """Run motor in reverse at specified speed (0-65535)"""
    motor_in1.off()
    motor_in2.on()
    motor_enable.duty_u16(speed)

def motor_stop():
    """Stop motor"""
    motor_in1.off()
    motor_in2.off()
    motor_enable.duty_u16(0)

# Main control loop
while True:
    motor_forward(52428)  # ~80% speed (65535 * 0.8)
    time.sleep(2)
    motor_stop()
    time.sleep(1)
    motor_reverse(39321)  # ~60% speed
    time.sleep(2)
    motor_stop()
    time.sleep(1)
```

**Object-oriented version (more advanced):**
```python
from machine import Pin, PWM
import time

class Motor:
    """Motor controller class"""

    def __init__(self, enable_pin, in1_pin, in2_pin):
        """Initialize motor with control pins"""
        self.enable = PWM(Pin(enable_pin))
        self.in1 = Pin(in1_pin, Pin.OUT)
        self.in2 = Pin(in2_pin, Pin.OUT)
        self.enable.freq(1000)

    def forward(self, speed):
        """Run motor forward (speed: 0-65535)"""
        self.in1.on()
        self.in2.off()
        self.enable.duty_u16(speed)

    def reverse(self, speed):
        """Run motor reverse (speed: 0-65535)"""
        self.in1.off()
        self.in2.on()
        self.enable.duty_u16(speed)

    def stop(self):
        """Stop motor"""
        self.in1.off()
        self.in2.off()
        self.enable.duty_u16(0)

    def brake(self):
        """Active brake (both pins high)"""
        self.in1.on()
        self.in2.on()
        self.enable.duty_u16(0)

# Create motor object
motor = Motor(enable_pin=9, in1_pin=7, in2_pin=8)

# Use it
while True:
    motor.forward(52428)
    time.sleep(2)
    motor.stop()
    time.sleep(1)
    motor.reverse(39321)
    time.sleep(2)
    motor.stop()
    time.sleep(1)
```

**Key advantages of the object-oriented approach:**
- Encapsulation - motor logic is self-contained
- Reusability - create multiple motor objects easily
- Cleaner code - `motor.forward(speed)` is more readable than setting three pins
- Easier debugging - motor state is managed in one place

---

## Multiple Pins: LED Pattern Example

**Arduino C++:**
```cpp
const int LEDS[] = {2, 3, 4, 5, 6};
const int NUM_LEDS = 5;

void setup() {
  for (int i = 0; i < NUM_LEDS; i++) {
    pinMode(LEDS[i], OUTPUT);
  }
}

void loop() {
  // Knight Rider pattern
  for (int i = 0; i < NUM_LEDS; i++) {
    digitalWrite(LEDS[i], HIGH);
    delay(100);
    digitalWrite(LEDS[i], LOW);
  }

  for (int i = NUM_LEDS - 1; i >= 0; i--) {
    digitalWrite(LEDS[i], HIGH);
    delay(100);
    digitalWrite(LEDS[i], LOW);
  }
}
```

**MicroPython:**
```python
from machine import Pin
import time

# Create list of LED Pin objects
leds = [Pin(pin_num, Pin.OUT) for pin_num in [2, 3, 4, 5, 6]]

while True:
    # Knight Rider pattern
    for led in leds:
        led.on()
        time.sleep(0.1)
        led.off()

    for led in reversed(leds):
        led.on()
        time.sleep(0.1)
        led.off()
```

**Python advantage:** List comprehension creates all Pin objects in one line! And iterating is cleaner with `for led in leds`.

---

## Pin State Tracking

**Arduino C++:**
```cpp
// Arduino doesn't track state - you must do it manually
int ledState = LOW;

digitalWrite(LED_PIN, ledState);
ledState = !ledState;  // Toggle for next time
```

**MicroPython:**
```python
# Pin object can read its own output state
led = Pin(13, Pin.OUT)

led.on()
print(f"LED is: {led.value()}")  # Prints: LED is: 1

# Or just toggle without tracking
led.toggle()
```

**Why this matters:** In Arduino, reading an output pin's state requires storing it separately. MicroPython Pin objects know their current state.

---

## Try It Yourself

**Exercise 1: Three-LED Traffic Light**
```python
from machine import Pin
import time

# Create your traffic light LEDs
red = Pin(2, Pin.OUT)
yellow = Pin(3, Pin.OUT)
green = Pin(4, Pin.OUT)

# TODO: Implement traffic light sequence:
# 1. Green for 5 seconds
# 2. Yellow for 2 seconds
# 3. Red for 5 seconds
# 4. Repeat

# Your code here
```

**Exercise 2: Button-Controlled LED Brightness**
```python
from machine import Pin, PWM
import time

# Setup
button = Pin(2, Pin.IN, Pin.PULL_UP)
led = PWM(Pin(13))
led.freq(1000)

brightness = 0
brightness_step = 13107  # 65535 / 5 steps

# TODO: Each button press increases brightness
# After max brightness, go back to off
# Hint: Track last button state to detect press event

# Your code here
```

**Exercise 3: Morse Code Blinker**
```python
from machine import Pin
import time

led = Pin(13, Pin.OUT)

# Morse code timing (units)
DOT = 0.2
DASH = DOT * 3
SYMBOL_GAP = DOT
LETTER_GAP = DOT * 3

def dot():
    led.on()
    time.sleep(DOT)
    led.off()
    time.sleep(SYMBOL_GAP)

def dash():
    led.on()
    time.sleep(DASH)
    led.off()
    time.sleep(SYMBOL_GAP)

# TODO: Implement a function to blink your initials
# Example: "SOS" = dot() dot() dot() space dash() dash() dash() space dot() dot() dot()

# Your code here
```

---

## Common Issues

**Problem: Pin number differences between boards**
```python
# Works on Pico
led = Pin(25, Pin.OUT)  # Onboard LED

# Doesn't work on ESP32 (onboard LED is pin 2)
```
**Solution:** Check your specific board's pinout diagram. Pin numbers aren't standardized like Arduino (13 = onboard LED).

**Why:** Different microcontrollers have different GPIO layouts.

---

**Problem: Forgetting to import Pin**
```python
led = Pin(13, Pin.OUT)  # NameError: name 'Pin' is not defined
```
**Solution:** Always start MicroPython programs with `from machine import Pin`.

**Why:** Unlike Arduino where functions are global, Python requires explicit imports.

---

**Problem: Using Arduino constants**
```python
# This won't work!
led = Pin(13, OUTPUT)  # NameError: name 'OUTPUT' is not defined
```
**Solution:** Use `Pin.OUT`, `Pin.IN`, etc. (note the `Pin.` prefix).

**Why:** These are attributes of the Pin class, not global constants.

---

**Problem: Pin already in use**
```python
led1 = Pin(13, Pin.OUT)
led2 = Pin(13, Pin.OUT)  # Might cause errors or unexpected behavior
```
**Solution:** Each physical pin should only have one Pin object. Reuse the existing object or choose different pins.

**Why:** Multiple Pin objects controlling the same physical pin can conflict.

---

## Summary: Arduino to MicroPython Pin Control

| Arduino C++ | MicroPython | Notes |
|-------------|-------------|-------|
| `pinMode(13, OUTPUT)` | `Pin(13, Pin.OUT)` | Creates output pin |
| `pinMode(2, INPUT)` | `Pin(2, Pin.IN)` | Creates input pin |
| `pinMode(2, INPUT_PULLUP)` | `Pin(2, Pin.IN, Pin.PULL_UP)` | Input with pull-up |
| `digitalWrite(13, HIGH)` | `led.value(1)` or `led.on()` | Set pin high |
| `digitalWrite(13, LOW)` | `led.value(0)` or `led.off()` | Set pin low |
| `digitalRead(2)` | `button.value()` | Read pin state |
| No direct equivalent | `led.toggle()` | Flip current state |
| Manual tracking | `led.value()` | Read output pin state |

**Key takeaways:**
- MicroPython uses **object-oriented Pin class** instead of global functions
- Must **import Pin** from machine module
- Pin modes use **Pin.OUT**, **Pin.IN**, **Pin.PULL_UP** (not Arduino constants)
- Use **on()/off()** methods or **value(0/1)** to control pins
- Can **toggle()** pins and **read output state** easily
- **Object-oriented approach** makes managing multiple pins cleaner

---

> **Next Lesson**: [Analog I/O](/learn/arduino_to_python/10_analog_io.html) - analogRead(), analogWrite(), and PWM in MicroPython
>
> **Previous Lesson**: [Arrays vs Lists](/learn/arduino_to_python/08_arrays_vs_lists.html)

---
