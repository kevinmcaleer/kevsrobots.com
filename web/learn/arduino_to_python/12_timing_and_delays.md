---
layout: lesson
title: Timing and Delays - delay(), millis(), and Non-Blocking Code
author: Kevin McAleer
type: page
cover: /learn/arduino_to_python/assets/timing_delays.jpg
date: 2025-01-20
previous: 11_serial_communication.html
next: 13_interrupts.html
description: Mastering time functions and implementing non-blocking delays in MicroPython
percent: 39
duration: 10
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


![Timing and Delays cover image](assets/timing_delays.jpg){:class="cover"}

## Introduction

You've probably used `delay(1000)` countless times to pause your Arduino code for one second. It's simple and works great for basic projects. But you've also likely discovered its major flaw: it blocks everything. Your robot can't read sensors, respond to buttons, or do anything else while waiting.

That's why you learned about `millis()` and the "blink without delay" pattern - a fundamental Arduino technique for non-blocking timing.

MicroPython has similar timing functions, but with important differences. In this lesson, you'll learn how to convert Arduino timing code to Python, master non-blocking delays, and discover Python's `time.ticks_ms()` function for precise timing.

## The Critical Difference: Seconds vs Milliseconds

**This is the #1 source of bugs when switching to Python!**

### Arduino C++ (Milliseconds)
```cpp
delay(1000);      // Wait 1000 milliseconds (1 second)
delay(500);       // Wait 500 milliseconds (0.5 seconds)
delayMicroseconds(100);  // Wait 100 microseconds
```

### Python (Seconds!)
```python
import time

time.sleep(1)      # Wait 1 second
time.sleep(0.5)    # Wait 0.5 seconds (500ms)
time.sleep(0.001)  # Wait 1 millisecond
time.sleep_us(100) # Wait 100 microseconds
```

**Remember:** Python's `sleep()` uses **seconds**, Arduino's `delay()` uses **milliseconds**!

## Blocking Delays: Simple but Limited

### Arduino C++ (delay)
```cpp
void loop() {
  digitalWrite(LED_PIN, HIGH);
  delay(1000);  // Wait 1 second - blocks everything!
  digitalWrite(LED_PIN, LOW);
  delay(1000);
}
```

### Python (sleep)
```python
from machine import Pin
import time

led = Pin(25, Pin.OUT)

while True:
    led.on()
    time.sleep(1)  # Wait 1 second - blocks everything!
    led.off()
    time.sleep(1)
```

**Both have the same problem:** Nothing else can happen during the delay. Your program is frozen.

## Getting Current Time: millis() vs ticks_ms()

### Arduino C++ (millis and micros)
```cpp
void setup() {
  Serial.begin(9600);
}

void loop() {
  // Get time since program started (milliseconds)
  unsigned long currentMillis = millis();
  Serial.println(currentMillis);

  // Get time in microseconds
  unsigned long currentMicros = micros();
  Serial.println(currentMicros);

  delay(1000);
}
```

### Python (ticks_ms and ticks_us)
```python
import time

while True:
    # Get time since program started (milliseconds)
    current_ms = time.ticks_ms()
    print(current_ms)

    # Get time in microseconds
    current_us = time.ticks_us()
    print(current_us)

    time.sleep(1)
```

**Key difference:** Arduino uses `millis()`, Python uses `time.ticks_ms()`.

## Non-Blocking Blink: The Classic Pattern

This is essential knowledge for any microcontroller programmer!

### Arduino C++ (Blink Without Delay)
```cpp
const int LED_PIN = 13;
const long INTERVAL = 1000;  // ms

unsigned long previousMillis = 0;
int ledState = LOW;

void setup() {
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  unsigned long currentMillis = millis();

  // Check if interval has elapsed
  if (currentMillis - previousMillis >= INTERVAL) {
    previousMillis = currentMillis;  // Save time

    // Toggle LED
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
  }

  // Can do other things here!
}
```

### Python (Equivalent Pattern)
```python
from machine import Pin
import time

LED_PIN = 25
INTERVAL = 1000  # ms

led = Pin(LED_PIN, Pin.OUT)
previous_ms = 0
led_state = False

while True:
    current_ms = time.ticks_ms()

    # Check if interval has elapsed
    if time.ticks_diff(current_ms, previous_ms) >= INTERVAL:
        previous_ms = current_ms

        # Toggle LED
        led_state = not led_state
        led.value(led_state)

    # Can do other things here!
    time.sleep(0.01)  # Small delay to prevent tight loop
```

**Critical detail:** Use `time.ticks_diff()` for proper tick comparison!

## Why ticks_diff() Matters

The tick counter wraps around after ~49 days. Direct subtraction can give wrong results!

### Arduino C++ (Works but can overflow)
```cpp
unsigned long currentMillis = millis();
if (currentMillis - previousMillis >= 1000) {
  // This works in most cases, but technically has overflow issues
}
```

### Python (Correct way with ticks_diff)
```python
current_ms = time.ticks_ms()

# Wrong - can fail on wraparound!
# if current_ms - previous_ms >= 1000:

# Correct - handles wraparound properly
if time.ticks_diff(current_ms, previous_ms) >= 1000:
    # This always works correctly
    pass
```

**Best practice:** Always use `time.ticks_diff(current, previous)` for tick comparisons.

## Real-World Example: Multi-Tasking Robot

### Arduino C++ (Multiple Non-Blocking Timers)
```cpp
unsigned long ledPreviousMillis = 0;
unsigned long sensorPreviousMillis = 0;
unsigned long motorPreviousMillis = 0;

const long LED_INTERVAL = 500;     // Blink every 500ms
const long SENSOR_INTERVAL = 100;  // Read every 100ms
const long MOTOR_INTERVAL = 50;    // Update every 50ms

void loop() {
  unsigned long currentMillis = millis();

  // LED task (500ms)
  if (currentMillis - ledPreviousMillis >= LED_INTERVAL) {
    ledPreviousMillis = currentMillis;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }

  // Sensor task (100ms)
  if (currentMillis - sensorPreviousMillis >= SENSOR_INTERVAL) {
    sensorPreviousMillis = currentMillis;
    int sensor = analogRead(A0);
    Serial.println(sensor);
  }

  // Motor task (50ms - PID control)
  if (currentMillis - motorPreviousMillis >= MOTOR_INTERVAL) {
    motorPreviousMillis = currentMillis;
    updateMotorSpeed();
  }
}
```

### Python (Same Multi-Tasking)
```python
from machine import Pin, ADC
import time

# Setup
led = Pin(25, Pin.OUT)
sensor = ADC(Pin(26))

# Timing variables
led_previous_ms = 0
sensor_previous_ms = 0
motor_previous_ms = 0

LED_INTERVAL = 500     # ms
SENSOR_INTERVAL = 100  # ms
MOTOR_INTERVAL = 50    # ms

def update_motor_speed():
    # Motor control logic
    pass

while True:
    current_ms = time.ticks_ms()

    # LED task (500ms)
    if time.ticks_diff(current_ms, led_previous_ms) >= LED_INTERVAL:
        led_previous_ms = current_ms
        led.toggle()

    # Sensor task (100ms)
    if time.ticks_diff(current_ms, sensor_previous_ms) >= SENSOR_INTERVAL:
        sensor_previous_ms = current_ms
        value = sensor.read_u16()
        print(value)

    # Motor task (50ms - PID control)
    if time.ticks_diff(current_ms, motor_previous_ms) >= MOTOR_INTERVAL:
        motor_previous_ms = current_ms
        update_motor_speed()

    # Small sleep to prevent tight loop
    time.sleep(0.001)
```

## Timeout Pattern: Waiting with a Limit

### Arduino C++ (Timeout While Waiting for Sensor)
```cpp
bool waitForSensor(int timeout) {
  unsigned long startTime = millis();

  while (digitalRead(SENSOR_PIN) == LOW) {
    if (millis() - startTime > timeout) {
      return false;  // Timeout!
    }
  }

  return true;  // Sensor triggered
}

void loop() {
  if (waitForSensor(5000)) {
    Serial.println("Sensor triggered!");
  } else {
    Serial.println("Timeout!");
  }
}
```

### Python (Same Timeout Pattern)
```python
from machine import Pin
import time

def wait_for_sensor(sensor_pin, timeout_ms):
    """Wait for sensor with timeout"""
    start_ms = time.ticks_ms()

    while sensor_pin.value() == 0:
        if time.ticks_diff(time.ticks_ms(), start_ms) > timeout_ms:
            return False  # Timeout!

        time.sleep(0.001)  # Small delay

    return True  # Sensor triggered

# Usage
sensor = Pin(15, Pin.IN, Pin.PULL_UP)

if wait_for_sensor(sensor, 5000):
    print("Sensor triggered!")
else:
    print("Timeout!")
```

## Button Debouncing with Timing

### Arduino C++ (Debounce with millis)
```cpp
const int BUTTON_PIN = 2;
const int LED_PIN = 13;
const long DEBOUNCE_DELAY = 50;  // ms

int buttonState = HIGH;
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
int ledState = LOW;

void loop() {
  int reading = digitalRead(BUTTON_PIN);

  // Check if button state changed
  if (reading != lastButtonState) {
    lastDebounceTime = millis();  // Reset timer
  }

  // If reading stable for DEBOUNCE_DELAY, accept it
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (reading != buttonState) {
      buttonState = reading;

      // If button pressed (LOW), toggle LED
      if (buttonState == LOW) {
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
      }
    }
  }

  lastButtonState = reading;
}
```

### Python (Same Debounce Logic)
```python
from machine import Pin
import time

BUTTON_PIN = 15
LED_PIN = 25
DEBOUNCE_DELAY = 50  # ms

button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_UP)
led = Pin(LED_PIN, Pin.OUT)

button_state = 1
last_button_state = 1
last_debounce_time = 0
led_state = False

while True:
    reading = button.value()

    # Check if button state changed
    if reading != last_button_state:
        last_debounce_time = time.ticks_ms()

    # If reading stable for DEBOUNCE_DELAY, accept it
    if time.ticks_diff(time.ticks_ms(), last_debounce_time) > DEBOUNCE_DELAY:
        if reading != button_state:
            button_state = reading

            # If button pressed (0 with pull-up), toggle LED
            if button_state == 0:
                led_state = not led_state
                led.value(led_state)

    last_button_state = reading
    time.sleep(0.001)
```

## Measuring Execution Time

### Arduino C++ (Timing Code Execution)
```cpp
void loop() {
  unsigned long startTime = micros();

  // Code to measure
  complexCalculation();

  unsigned long endTime = micros();
  unsigned long duration = endTime - startTime;

  Serial.print("Execution time: ");
  Serial.print(duration);
  Serial.println(" us");

  delay(1000);
}
```

### Python (Same Timing Measurement)
```python
import time

def complex_calculation():
    # Some expensive operation
    result = sum(x**2 for x in range(1000))
    return result

while True:
    start_us = time.ticks_us()

    # Code to measure
    complex_calculation()

    end_us = time.ticks_us()
    duration = time.ticks_diff(end_us, start_us)

    print(f"Execution time: {duration} us")

    time.sleep(1)
```

## Class-Based Timer (Python Advantage)

Python's object-oriented approach makes timers reusable:

```python
import time

class Timer:
    def __init__(self, interval_ms):
        self.interval_ms = interval_ms
        self.last_time = time.ticks_ms()

    def elapsed(self):
        """Check if interval has elapsed"""
        current = time.ticks_ms()
        if time.ticks_diff(current, self.last_time) >= self.interval_ms:
            self.last_time = current
            return True
        return False

    def reset(self):
        """Reset timer"""
        self.last_time = time.ticks_ms()

# Usage - much cleaner!
led_timer = Timer(500)
sensor_timer = Timer(100)
motor_timer = Timer(50)

while True:
    if led_timer.elapsed():
        led.toggle()

    if sensor_timer.elapsed():
        value = sensor.read_u16()
        print(value)

    if motor_timer.elapsed():
        update_motor_speed()

    time.sleep(0.001)
```

## Try It Yourself

### Exercise 1: Morse Code SOS
Blink an LED in Morse code SOS pattern (··· --- ···) using non-blocking timing:
- Dot: 200ms on, 200ms off
- Dash: 600ms on, 200ms off
- Letter gap: 600ms
- Repeat continuously

### Exercise 2: Multi-Speed Blink
Blink three LEDs at different rates simultaneously:
- LED 1: 1 Hz (1 second on, 1 second off)
- LED 2: 2 Hz
- LED 3: 0.5 Hz

### Exercise 3: Sensor Averaging with Timing
Read a sensor every 100ms, keep a moving average of last 10 readings, print average every 1 second.

### Exercise 4: Watchdog Timer
Create a watchdog timer that resets a flag if not "fed" within 5 seconds:

```python
class Watchdog:
    def __init__(self, timeout_ms):
        # Your code here
        pass

    def feed(self):
        """Reset watchdog timer"""
        pass

    def expired(self):
        """Check if watchdog has expired"""
        pass

# Usage
watchdog = Watchdog(5000)  # 5 second timeout

while True:
    # Do work...
    if some_condition:
        watchdog.feed()  # Reset timer

    if watchdog.expired():
        print("Watchdog expired! System error!")
        # Take emergency action
```

## Common Issues

### Issue: Using seconds instead of milliseconds
**Problem:**
```python
previous_ms = 0
INTERVAL = 1  # Intended 1 second, but this is 1ms!

if time.ticks_diff(time.ticks_ms(), previous_ms) >= INTERVAL:
    # Triggers every millisecond!
```

**Solution:**
```python
INTERVAL = 1000  # 1 second = 1000 milliseconds
```

### Issue: Forgetting ticks_diff()
**Problem:**
```python
if time.ticks_ms() - previous_ms >= 1000:
    # Can fail on wraparound!
```

**Solution:**
```python
if time.ticks_diff(time.ticks_ms(), previous_ms) >= 1000:
    # Always correct
```

### Issue: Tight loop consuming CPU
**Problem:**
```python
while True:
    # Check timers constantly
    if timer.elapsed():
        do_something()
    # No sleep - wastes power!
```

**Solution:**
```python
while True:
    if timer.elapsed():
        do_something()
    time.sleep(0.001)  # Small sleep saves power
```

## Summary

| Feature | Arduino C++ | Python |
|---------|-------------|---------|
| **Blocking delay** | `delay(1000)` (ms) | `time.sleep(1)` (seconds!) |
| **Microsecond delay** | `delayMicroseconds(100)` | `time.sleep_us(100)` |
| **Current time (ms)** | `millis()` | `time.ticks_ms()` |
| **Current time (us)** | `micros()` | `time.ticks_us()` |
| **Time difference** | `current - previous` | `time.ticks_diff(current, previous)` |
| **Wraparound safe** | No (but usually okay) | Yes (with ticks_diff) |

**Critical Differences:**
- Arduino `delay()` uses **milliseconds**
- Python `time.sleep()` uses **seconds**
- Always use `time.ticks_diff()` for safe comparisons

**Non-Blocking Pattern:**
1. Store previous time
2. Get current time
3. Calculate difference with `ticks_diff()`
4. If difference >= interval, do task and update previous time

**Best Practices:**
- Use `ticks_diff()` for all tick comparisons
- Add small `sleep()` in loops to save power
- Create Timer class for reusable timing logic
- Remember: seconds vs milliseconds!

In the next lesson, we'll explore interrupts - how Arduino's `attachInterrupt()` compares to MicroPython's `Pin.irq()`.

---

> **Previous Lesson:** [Serial Communication](/learn/arduino_to_python/11_serial_communication.html) |
> **Next Lesson:** [Interrupts](/learn/arduino_to_python/13_interrupts.html)
>
> **Course Home:** [Arduino to Python](/learn/arduino_to_python/)
