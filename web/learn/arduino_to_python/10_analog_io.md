---
layout: lesson
title: Analog I/O - analogRead() and analogWrite() in MicroPython
author: Kevin McAleer
type: page
cover: /learn/arduino_to_python/assets/analog_io.jpg
date: 2025-01-20
previous: 09_pin_control.html
next: 11_serial_communication.html
description: Working with ADC for analog input and PWM for analog output
percent: 33
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


![Analog I/O cover image](assets/analog_io.jpg){:class="cover"}

## Introduction

In Arduino, `analogRead()` and `analogWrite()` are your go-to functions for working with analog sensors and controlling things like LED brightness or motor speed. They're simple function calls that hide the complexity of ADC (Analog-to-Digital Converter) and PWM (Pulse Width Modulation) hardware.

MicroPython takes a different approach: you work with `ADC` and `PWM` objects directly. This is more verbose initially, but gives you finer control over resolution, frequency, and timing.

In this lesson, you'll learn how to read analog sensors with the ADC class, control analog outputs with PWM, and understand the key differences in resolution and capabilities between Arduino and MicroPython.

## Analog Input: analogRead() vs ADC Class

### Arduino C++ (Function-Based)
```cpp
void setup() {
  // No setup needed for analog input!
  pinMode(A0, INPUT);  // Optional, analog pins default to INPUT
}

void loop() {
  // Read analog value (0-1023 on most Arduino boards)
  int sensorValue = analogRead(A0);

  // Convert to voltage (assuming 5V reference)
  float voltage = sensorValue * (5.0 / 1023.0);

  Serial.print("Value: ");
  Serial.print(sensorValue);
  Serial.print(" Voltage: ");
  Serial.println(voltage);

  delay(100);
}
```

**Arduino `analogRead()` characteristics:**
- Returns 10-bit value (0-1023)
- Simple function call
- Reference voltage configurable (5V, 3.3V, internal)

### MicroPython (Object-Oriented)
```python
from machine import ADC, Pin
import time

# Create ADC object for pin 26
sensor = ADC(Pin(26))

while True:
    # Read analog value (0-65535 on RP2040, 16-bit)
    sensor_value = sensor.read_u16()

    # Convert to voltage (assuming 3.3V reference)
    voltage = sensor_value * (3.3 / 65535)

    print(f"Value: {sensor_value}, Voltage: {voltage:.2f}V")

    time.sleep(0.1)
```

**MicroPython ADC characteristics:**
- Create ADC object first
- 16-bit resolution (0-65535) on most modern boards
- Higher precision than Arduino's 10-bit
- Object methods for different read formats

## Side-by-Side: Reading a Potentiometer

### Arduino C++
```cpp
const int POT_PIN = A0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  int potValue = analogRead(POT_PIN);

  // Map to 0-100 percentage
  int percentage = map(potValue, 0, 1023, 0, 100);

  Serial.print("Pot: ");
  Serial.print(percentage);
  Serial.println("%");

  delay(50);
}
```

### MicroPython
```python
from machine import ADC, Pin
import time

# Create ADC object
pot = ADC(Pin(26))

while True:
    pot_value = pot.read_u16()

    # Map to 0-100 percentage
    percentage = int(pot_value / 65535 * 100)

    print(f"Pot: {percentage}%")

    time.sleep(0.05)
```

**Key differences:**
- Arduino: 10-bit (0-1023), MicroPython: 16-bit (0-65535)
- Arduino: `map()` function, Python: manual calculation
- Arduino: `analogRead()` function, Python: `ADC.read_u16()` method

## ADC Reading Methods in MicroPython

MicroPython provides multiple read methods depending on your needs:

```python
from machine import ADC, Pin

adc = ADC(Pin(26))

# 16-bit read (0-65535) - most common
value_16bit = adc.read_u16()

# Note: Some boards also support:
# value_raw = adc.read()  # Platform-specific raw value
```

**Best practice:** Use `read_u16()` for consistency across platforms.

## Real-World Example: Light Sensor with Threshold

### Arduino C++
```cpp
const int LIGHT_SENSOR = A0;
const int LED = 13;
const int THRESHOLD = 512;  // Middle of 0-1023 range

void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  int lightLevel = analogRead(LIGHT_SENSOR);

  // Turn on LED if dark
  if (lightLevel < THRESHOLD) {
    digitalWrite(LED, HIGH);
    Serial.println("Dark - LED ON");
  } else {
    digitalWrite(LED, LOW);
    Serial.println("Bright - LED OFF");
  }

  delay(100);
}
```

### MicroPython
```python
from machine import ADC, Pin
import time

# Setup
light_sensor = ADC(Pin(26))
led = Pin(25, Pin.OUT)
THRESHOLD = 32768  # Middle of 0-65535 range

while True:
    light_level = light_sensor.read_u16()

    # Turn on LED if dark
    if light_level < THRESHOLD:
        led.on()
        print("Dark - LED ON")
    else:
        led.off()
        print("Bright - LED OFF")

    time.sleep(0.1)
```

## Analog Output: analogWrite() vs PWM Class

### Arduino C++ (Simple PWM)
```cpp
const int LED_PIN = 9;  // Must be PWM-capable pin

void setup() {
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // Fade in
  for (int brightness = 0; brightness <= 255; brightness++) {
    analogWrite(LED_PIN, brightness);  // 0-255
    delay(10);
  }

  // Fade out
  for (int brightness = 255; brightness >= 0; brightness--) {
    analogWrite(LED_PIN, brightness);
    delay(10);
  }
}
```

**Arduino `analogWrite()` characteristics:**
- Takes value 0-255 (8-bit)
- Default frequency ~490 Hz (pins 5,6 on Uno ~980 Hz)
- Can't change frequency easily
- Not true analog (it's PWM)

### MicroPython (Explicit PWM)
```python
from machine import Pin, PWM
import time

# Create PWM object
led = PWM(Pin(25))

# Set frequency (Hz) - required step!
led.freq(1000)  # 1 kHz

while True:
    # Fade in (0-65535 for 16-bit)
    for brightness in range(0, 65536, 256):
        led.duty_u16(brightness)
        time.sleep(0.01)

    # Fade out
    for brightness in range(65535, -1, -256):
        led.duty_u16(brightness)
        time.sleep(0.01)
```

**MicroPython PWM characteristics:**
- Create PWM object explicitly
- Must set frequency with `.freq()`
- 16-bit duty cycle (0-65535) with `duty_u16()`
- Full control over frequency
- Also supports percentage with `duty_ns()` for nanosecond timing

## PWM Methods Explained

```python
from machine import Pin, PWM

pwm = PWM(Pin(25))

# Set frequency (required before using duty)
pwm.freq(1000)  # 1000 Hz (1 kHz)

# Set duty cycle - different methods:

# 1. duty_u16() - 16-bit value (0-65535)
pwm.duty_u16(32768)  # 50% duty cycle

# 2. duty_ns() - nanoseconds (advanced timing control)
pwm.duty_ns(500000)  # 500 microseconds on time

# Get current frequency
current_freq = pwm.freq()

# Deinitialize PWM (return pin to normal use)
pwm.deinit()
```

## Side-by-Side: Motor Speed Control

### Arduino C++
```cpp
const int MOTOR_PIN = 9;
const int POT_PIN = A0;

void setup() {
  pinMode(MOTOR_PIN, OUTPUT);
}

void loop() {
  // Read potentiometer (0-1023)
  int potValue = analogRead(POT_PIN);

  // Map to motor speed (0-255)
  int motorSpeed = map(potValue, 0, 1023, 0, 255);

  // Set motor speed
  analogWrite(MOTOR_PIN, motorSpeed);

  delay(10);
}
```

### MicroPython
```python
from machine import Pin, PWM, ADC
import time

# Setup
motor = PWM(Pin(15))
motor.freq(1000)  # 1 kHz for smooth motor control
pot = ADC(Pin(26))

while True:
    # Read potentiometer (0-65535)
    pot_value = pot.read_u16()

    # Use pot value directly for motor speed (both 16-bit)
    motor.duty_u16(pot_value)

    time.sleep(0.01)
```

**Advantage in MicroPython:** Pot and PWM both use 16-bit values, no mapping needed!

## Controlling PWM Frequency

This is where MicroPython shines - full frequency control!

### Arduino C++ (Limited Control)
```cpp
// Frequency is mostly fixed (~490 Hz or ~980 Hz)
// Changing it requires Timer register manipulation
analogWrite(9, 128);  // Can't easily change frequency
```

### MicroPython (Full Control)
```python
from machine import Pin, PWM

led = PWM(Pin(25))

# Try different frequencies
led.freq(100)       # 100 Hz - visible flicker
led.duty_u16(32768)

time.sleep(2)

led.freq(1000)      # 1 kHz - smooth
time.sleep(2)

led.freq(10000)     # 10 kHz - ultrasonic, very smooth
time.sleep(2)

# For servos: use 50 Hz
servo = PWM(Pin(16))
servo.freq(50)      # Standard servo frequency
```

**Common PWM frequencies:**
- **LEDs:** 1-10 kHz (smooth, no flicker)
- **Motors:** 1-20 kHz (smooth operation)
- **Servos:** 50 Hz (standard)
- **Buzzers:** 2-4 kHz (audible tones)

## Real-World Example: RGB LED Control

### Arduino C++
```cpp
const int RED_PIN = 9;
const int GREEN_PIN = 10;
const int BLUE_PIN = 11;

void setup() {
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
}

void setColor(int red, int green, int blue) {
  analogWrite(RED_PIN, red);
  analogWrite(GREEN_PIN, green);
  analogWrite(BLUE_PIN, blue);
}

void loop() {
  setColor(255, 0, 0);    // Red
  delay(1000);
  setColor(0, 255, 0);    // Green
  delay(1000);
  setColor(0, 0, 255);    // Blue
  delay(1000);
  setColor(255, 255, 0);  // Yellow
  delay(1000);
}
```

### MicroPython
```python
from machine import Pin, PWM
import time

class RGBLED:
    def __init__(self, red_pin, green_pin, blue_pin):
        self.red = PWM(Pin(red_pin))
        self.green = PWM(Pin(green_pin))
        self.blue = PWM(Pin(blue_pin))

        # Set frequency for all channels
        for channel in [self.red, self.green, self.blue]:
            channel.freq(1000)

    def set_color(self, red, green, blue):
        """Set RGB values (0-255)"""
        # Convert 8-bit (0-255) to 16-bit (0-65535)
        self.red.duty_u16(red * 257)
        self.green.duty_u16(green * 257)
        self.blue.duty_u16(blue * 257)

    def off(self):
        self.set_color(0, 0, 0)

# Usage
led = RGBLED(9, 10, 11)

led.set_color(255, 0, 0)    # Red
time.sleep(1)
led.set_color(0, 255, 0)    # Green
time.sleep(1)
led.set_color(0, 0, 255)    # Blue
time.sleep(1)
led.set_color(255, 255, 0)  # Yellow
time.sleep(1)
led.off()
```

## Servo Control: Special Case of PWM

Servos use PWM with specific timing requirements.

### Arduino C++ (Servo Library)
```cpp
#include <Servo.h>

Servo myServo;

void setup() {
  myServo.attach(9);
}

void loop() {
  myServo.write(0);    // 0 degrees
  delay(1000);
  myServo.write(90);   // 90 degrees
  delay(1000);
  myServo.write(180);  // 180 degrees
  delay(1000);
}
```

### MicroPython (Manual PWM Control)
```python
from machine import Pin, PWM
import time

class Servo:
    def __init__(self, pin):
        self.pwm = PWM(Pin(pin))
        self.pwm.freq(50)  # 50 Hz for servos

    def write(self, angle):
        """Set servo angle (0-180 degrees)"""
        # Convert angle to duty cycle
        # 0° = 1ms pulse (3.28% of 20ms)
        # 90° = 1.5ms pulse (7.5% of 20ms)
        # 180° = 2ms pulse (10% of 20ms)

        # Map angle to duty cycle (16-bit)
        min_duty = 1638   # ~2.5% (1ms / 20ms * 65535)
        max_duty = 8192   # ~12.5% (2.5ms / 20ms * 65535)
        duty = int(min_duty + (angle / 180) * (max_duty - min_duty))

        self.pwm.duty_u16(duty)

    def deinit(self):
        self.pwm.deinit()

# Usage
servo = Servo(16)

servo.write(0)      # 0 degrees
time.sleep(1)
servo.write(90)     # 90 degrees
time.sleep(1)
servo.write(180)    # 180 degrees
time.sleep(1)
```

## Try It Yourself

### Exercise 1: Analog Joystick
Read a 2-axis analog joystick (two potentiometers) and print X, Y coordinates:

```python
from machine import ADC, Pin
import time

# Create ADC objects for X and Y axes
x_axis = ADC(Pin(26))
y_axis = ADC(Pin(27))

# Read and print joystick position
# Map to -100 to +100 range (center at 0)
```

### Exercise 2: Breathing LED
Create a "breathing" effect where an LED smoothly fades in and out using a sine wave:

```python
from machine import Pin, PWM
import time
import math

led = PWM(Pin(25))
led.freq(1000)

# Use sine wave for smooth breathing effect
# Hint: math.sin() returns -1 to 1, convert to 0-65535
```

### Exercise 3: Temperature-Controlled Fan
Read a temperature sensor (analog), control a fan speed with PWM:

```python
# If temp < 25°C: fan off
# If temp 25-35°C: fan speed proportional to temp
# If temp > 35°C: fan at full speed
```

### Exercise 4: RGB Color Mixer
Read 3 potentiometers (R, G, B) and set RGB LED accordingly:

```python
# Pot 1 controls red
# Pot 2 controls green
# Pot 3 controls blue
# Display current color values
```

## Common Issues

### Issue: PWM on Non-PWM Pin
**Problem:** Not all pins support PWM on all boards.

**Solution:** Check your board's pinout diagram for PWM-capable pins.

### Issue: Servo Jitter
**Problem:**
```python
servo = PWM(Pin(16))
servo.freq(50)
servo.duty_u16(4915)  # Servo jitters
```

**Solution:** Use more precise duty cycle values and ensure stable power supply. Servos draw significant current.

### Issue: ADC Reference Voltage
**Problem:** Readings seem off by a constant factor.

**Solution:** Check your board's ADC reference voltage (3.3V vs 5V) and adjust calculations:

```python
# For 3.3V systems (most modern boards)
voltage = adc_value * (3.3 / 65535)

# For 5V systems (rare in MicroPython)
voltage = adc_value * (5.0 / 65535)
```

## Summary

| Feature | Arduino C++ | MicroPython |
|---------|-------------|-------------|
| **Analog Input** | `analogRead(A0)` | `ADC(Pin(26)).read_u16()` |
| **Input Resolution** | 10-bit (0-1023) | 16-bit (0-65535) |
| **Analog Output** | `analogWrite(pin, 0-255)` | `PWM(Pin).duty_u16(0-65535)` |
| **Output Resolution** | 8-bit (0-255) | 16-bit (0-65535) |
| **PWM Frequency** | Fixed (~490 Hz) | Configurable with `.freq()` |
| **Setup** | Function-based | Object-oriented |
| **Servo Control** | Servo library | Manual PWM timing |

**Python Advantages:**
- Higher resolution (16-bit vs 8/10-bit)
- Full frequency control
- Object-oriented approach (more organized)
- Consistent API across boards

**Arduino Advantages:**
- Simpler for beginners (`analogWrite(pin, value)`)
- Servo library handles timing automatically
- More established ecosystem

**Best Practices:**
- Use `read_u16()` and `duty_u16()` for consistency
- Set PWM frequency before setting duty cycle
- Choose appropriate frequency for your application
- Remember MicroPython uses 16-bit resolution

In the next lesson, we'll explore serial communication - comparing Arduino's Serial class with MicroPython's UART.

---

> **Previous Lesson:** [Pin Control](/learn/arduino_to_python/09_pin_control.html) |
> **Next Lesson:** [Serial Communication](/learn/arduino_to_python/11_serial_communication.html)
>
> **Course Home:** [Arduino to Python](/learn/arduino_to_python/)
