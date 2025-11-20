---
layout: lesson
title: Arduino Libraries vs Python Modules
author: Kevin McAleer
type: page
cover: /learn/arduino_to_python/assets/libraries_modules.jpg
date: 2025-01-20
previous: 15_object_oriented.html
next: 17_file_systems.html
description: Understanding how to use and create libraries in both ecosystems
percent: 51
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


![Libraries vs Modules cover image](assets/libraries_modules.jpg){:class="cover"}

## Introduction

Code reuse is essential for efficient programming. In Arduino, you use libraries - collections of code that provide functionality for sensors, displays, motors, and more. Python has modules - a similar concept but with different conventions and capabilities.

In this lesson, you'll learn how Arduino's `#include` system compares to Python's `import`, how to install third-party code in both ecosystems, and how to create your own reusable modules.

## Including vs Importing

### Arduino C++ (#include)
```cpp
// Include Arduino built-in libraries
#include <Wire.h>         // I2C communication
#include <SPI.h>          // SPI communication
#include <Servo.h>        // Servo control

// Include third-party library
#include <Adafruit_NeoPixel.h>

// Include your own library (same folder)
#include "MySensor.h"

void setup() {
  // Use library functions
  Wire.begin();
  Servo myServo;
  myServo.attach(9);
}
```

**Arduino include types:**
- `<Library.h>` - System/installed libraries
- `"MyFile.h"` - Local files in your sketch folder

### Python (import)
```python
# Import built-in modules
import time              # Time functions
import gc                # Garbage collector
from machine import Pin  # Import specific class

# Import third-party module
import neopixel

# Import your own module (same folder)
import my_sensor

# Or import specific items
from my_sensor import MySensor, read_temperature

# Usage
sensor = MySensor()
temp = read_temperature()
```

**Python import variations:**
- `import module` - Import entire module
- `from module import item` - Import specific item
- `import module as alias` - Rename for convenience
- `from module import *` - Import everything (not recommended)

## Standard Library Comparison

### Arduino Built-In Libraries
```cpp
#include <Wire.h>       // I2C
#include <SPI.h>        // SPI
#include <EEPROM.h>     // Non-volatile storage
#include <Servo.h>      // Servo motors
#include <SoftwareSerial.h>  // Software serial

// Arduino ecosystem libraries (install via Library Manager)
#include <Adafruit_GFX.h>
#include <WiFi.h>
#include <SD.h>
```

### MicroPython Built-In Modules
```python
import time          # Time and delays
import gc            # Garbage collection
import os            # File system operations
import sys           # System-specific parameters
import json          # JSON encoding/decoding
import struct        # Pack/unpack binary data

# Hardware-specific modules
from machine import Pin, PWM, ADC, I2C, SPI, UART
import neopixel      # WS2812B LEDs (built-in on some boards)
import network       # WiFi/network (ESP32, Pico W)
```

## Installing Third-Party Code

### Arduino (Library Manager)
```
1. Open Arduino IDE
2. Go to Sketch → Include Library → Manage Libraries
3. Search for library (e.g., "Adafruit NeoPixel")
4. Click Install
5. #include <Adafruit_NeoPixel.h> in your sketch
```

**Manual installation:**
```
1. Download .zip file
2. Sketch → Include Library → Add .ZIP Library
3. Select downloaded file
```

### MicroPython (Multiple Methods)

**Method 1: Copy files directly**
```bash
# Upload .py file to board using Thonny, ampy, or rshell
ampy --port /dev/ttyUSB0 put library.py
```

**Method 2: mpremote (official tool)**
```bash
# Install mpremote
pip install mpremote

# Copy file to board
mpremote cp library.py :

# Copy entire folder
mpremote cp -r mymodule/ :
```

**Method 3: mip (MicroPython package manager - new!)**
```python
# On the board (requires network connection)
import mip

# Install from micropython-lib
mip.install("package-name")

# Install from GitHub
mip.install("github:user/repo")
```

**Method 4: Manual REPL paste**
```python
# In Thonny or REPL, paste small modules directly
# File is created on board's filesystem
```

## Creating Your Own Library/Module

### Arduino C++ Library

**File structure:**
```
MyLibrary/
├── MyLibrary.h      # Header file
├── MyLibrary.cpp    # Implementation
├── keywords.txt     # Syntax highlighting
├── library.properties  # Metadata
└── examples/        # Example sketches
    └── BasicUsage/
        └── BasicUsage.ino
```

**MyLibrary.h:**
```cpp
#ifndef MyLibrary_h
#define MyLibrary_h

#include "Arduino.h"

class DistanceSensor {
  public:
    DistanceSensor(int trigPin, int echoPin);
    void begin();
    float readDistance();

  private:
    int _trigPin;
    int _echoPin;
};

#endif
```

**MyLibrary.cpp:**
```cpp
#include "MyLibrary.h"

DistanceSensor::DistanceSensor(int trigPin, int echoPin) {
  _trigPin = trigPin;
  _echoPin = echoPin;
}

void DistanceSensor::begin() {
  pinMode(_trigPin, OUTPUT);
  pinMode(_echoPin, INPUT);
}

float DistanceSensor::readDistance() {
  digitalWrite(_trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_trigPin, LOW);

  long duration = pulseIn(_echoPin, HIGH);
  float distance = duration * 0.034 / 2;

  return distance;
}
```

**Usage:**
```cpp
#include <MyLibrary.h>

DistanceSensor sensor(9, 10);

void setup() {
  Serial.begin(9600);
  sensor.begin();
}

void loop() {
  float distance = sensor.readDistance();
  Serial.println(distance);
  delay(100);
}
```

### Python Module (Much Simpler!)

**File: distance_sensor.py**
```python
from machine import Pin
import time

class DistanceSensor:
    def __init__(self, trig_pin, echo_pin):
        self.trig = Pin(trig_pin, Pin.OUT)
        self.echo = Pin(echo_pin, Pin.IN)

    def read_distance(self):
        """Read distance in cm"""
        # Send trigger pulse
        self.trig.value(0)
        time.sleep_us(2)
        self.trig.value(1)
        time.sleep_us(10)
        self.trig.value(0)

        # Measure echo pulse
        while self.echo.value() == 0:
            pulse_start = time.ticks_us()

        while self.echo.value() == 1:
            pulse_end = time.ticks_us()

        # Calculate distance
        duration = time.ticks_diff(pulse_end, pulse_start)
        distance = duration * 0.034 / 2

        return distance
```

**Usage:**
```python
from distance_sensor import DistanceSensor
import time

sensor = DistanceSensor(trig_pin=9, echo_pin=10)

while True:
    distance = sensor.read_distance()
    print(f"Distance: {distance:.1f} cm")
    time.sleep(0.1)
```

**That's it!** No header files, no keywords.txt, no complex structure. Just a `.py` file.

## Import Variations

### Python Import Flexibility
```python
# Import entire module
import distance_sensor
sensor = distance_sensor.DistanceSensor(9, 10)

# Import specific class
from distance_sensor import DistanceSensor
sensor = DistanceSensor(9, 10)

# Import with alias
import distance_sensor as ds
sensor = ds.DistanceSensor(9, 10)

# Import multiple items
from distance_sensor import DistanceSensor, read_distance

# Import everything (not recommended)
from distance_sensor import *
```

### Creating Package with Multiple Modules

**File structure:**
```
robotics/
├── __init__.py      # Makes it a package
├── motors.py
├── sensors.py
└── navigation.py
```

**robotics/__init__.py:**
```python
# Import key classes for easy access
from .motors import Motor, MotorController
from .sensors import DistanceSensor, LineSensor
from .navigation import Navigator

__version__ = "1.0.0"
```

**Usage:**
```python
# Can import from package directly
from robotics import Motor, DistanceSensor, Navigator

# Or import modules
from robotics import motors, sensors

motor = motors.Motor(15)
sensor = sensors.DistanceSensor(9, 10)
```

## Real-World Example: Reusable Robot Library

### Arduino C++ Library

**RobotBase.h:**
```cpp
#ifndef RobotBase_h
#define RobotBase_h

#include "Arduino.h"

class Robot {
  public:
    Robot(int leftMotor, int rightMotor);
    void forward(int speed);
    void backward(int speed);
    void turnLeft(int speed);
    void turnRight(int speed);
    void stop();

  private:
    int _leftMotor;
    int _rightMotor;
};

#endif
```

**Usage:**
```cpp
#include <RobotBase.h>

Robot myRobot(9, 10);

void setup() {
  // Nothing needed
}

void loop() {
  myRobot.forward(150);
  delay(1000);
  myRobot.turnRight(100);
  delay(500);
}
```

### Python Module

**robot_base.py:**
```python
from machine import Pin, PWM

class Robot:
    def __init__(self, left_motor_pin, right_motor_pin):
        self.left_motor = PWM(Pin(left_motor_pin))
        self.right_motor = PWM(Pin(right_motor_pin))

        # Set PWM frequency
        self.left_motor.freq(1000)
        self.right_motor.freq(1000)

    def forward(self, speed):
        """Move forward at specified speed (0-255)"""
        duty = int(speed / 255 * 65535)
        self.left_motor.duty_u16(duty)
        self.right_motor.duty_u16(duty)

    def backward(self, speed):
        """Move backward at specified speed (0-255)"""
        # Assuming reversible motors or H-bridge
        duty = int(speed / 255 * 65535)
        self.left_motor.duty_u16(65535 - duty)
        self.right_motor.duty_u16(65535 - duty)

    def turn_left(self, speed):
        """Turn left"""
        duty = int(speed / 255 * 65535)
        self.left_motor.duty_u16(0)
        self.right_motor.duty_u16(duty)

    def turn_right(self, speed):
        """Turn right"""
        duty = int(speed / 255 * 65535)
        self.left_motor.duty_u16(duty)
        self.right_motor.duty_u16(0)

    def stop(self):
        """Stop both motors"""
        self.left_motor.duty_u16(0)
        self.right_motor.duty_u16(0)
```

**Usage:**
```python
from robot_base import Robot
import time

robot = Robot(left_motor_pin=15, right_motor_pin=16)

while True:
    robot.forward(150)
    time.sleep(1)
    robot.turn_right(100)
    time.sleep(0.5)
    robot.stop()
    time.sleep(0.5)
```

## Module Search Path

### Where Python Looks for Modules

```python
import sys

# Print module search paths
print(sys.path)
# Output: ['', '/lib', ...]
```

**Python searches in order:**
1. Current directory
2. `/lib` folder on the board
3. Built-in frozen modules

**Organizing modules:**
```
/  (root)
├── main.py
├── lib/
│   ├── robot_base.py
│   ├── sensors.py
│   └── navigation.py
└── config.json
```

## Try It Yourself

### Exercise 1: Create LED Controller Module
Create a reusable LED controller module:

```python
# led_controller.py
from machine import Pin
import time

class LEDController:
    def __init__(self, pin):
        # Your code here
        pass

    def on(self):
        pass

    def off(self):
        pass

    def blink(self, times, delay=0.5):
        pass

    def fade(self, duration=2):
        # Bonus: Use PWM for fading
        pass

# Test it
from led_controller import LEDController
led = LEDController(25)
led.blink(5)
```

### Exercise 2: Sensor Calibration Module
Create a module for calibrating analog sensors:

```python
# sensor_cal.py
# Should provide:
# - calibrate(sensor, num_samples=100)
# - map_value(value, in_min, in_max, out_min, out_max)
# - smooth_reading(sensor, window_size=10)
```

### Exercise 3: Configuration Manager
Create a module that loads/saves settings to JSON:

```python
# config.py
import json

def load_config(filename='config.json'):
    """Load configuration from JSON file"""
    pass

def save_config(config, filename='config.json'):
    """Save configuration to JSON file"""
    pass

# Usage:
# config = load_config()
# config['motor_speed'] = 150
# save_config(config)
```

### Exercise 4: Robot Behaviors Library
Create a behaviors module with common robot actions:

```python
# behaviors.py
# - obstacle_avoid(robot, sensor)
# - wall_follow(robot, sensor, side='left')
# - line_follow(robot, sensors)
# - wander(robot, duration)
```

## Common Issues

### Issue: Module Not Found
**Problem:**
```python
import my_module  # ModuleNotFoundError!
```

**Solution:**
```python
# 1. Check file is uploaded to board
# 2. Check filename (case-sensitive!)
# 3. Check sys.path
import sys
print(sys.path)

# 4. Put module in /lib folder for global access
```

### Issue: Circular Imports
**Problem:**
```python
# module_a.py
from module_b import something

# module_b.py
from module_a import something_else
# ImportError: circular import
```

**Solution:** Restructure code to avoid circular dependencies or use late imports inside functions.

### Issue: Importing from Parent Directory
**Problem:** Can't import from `../other_module`

**Solution:**
```python
# Add parent directory to path
import sys
sys.path.append('..')

import other_module
```

## Summary

| Feature | Arduino C++ | Python |
|---------|-------------|---------|
| **Include/Import** | `#include <Library.h>` | `import module` |
| **Local files** | `#include "MyFile.h"` | `import myfile` |
| **Specific items** | Not applicable | `from module import Class` |
| **File structure** | .h + .cpp | Single .py file |
| **Installation** | Library Manager / ZIP | Copy files / mip |
| **Package manager** | Library Manager | mip (network required) |
| **Creating library** | Complex (header, impl, metadata) | Simple (one .py file) |
| **Header files** | Required | Not needed |
| **Namespaces** | Manual | Automatic (module name) |

**Python Advantages:**
- Simpler module creation (no headers)
- Flexible import options
- No compilation needed
- Easy to share (just .py files)

**Arduino Advantages:**
- Established Library Manager
- Large ecosystem of libraries
- Syntax highlighting via keywords.txt
- Built-in examples system

**Best Practices:**
- Keep modules focused (one purpose)
- Use clear, descriptive names
- Include docstrings
- Organize related modules in packages
- Put reusable code in `/lib` folder

In the next lesson, we'll explore file systems - a capability Python has that Arduino typically doesn't, opening up new possibilities for data logging and configuration.

---

> **Previous Lesson:** [Memory Management](/learn/arduino_to_python/14_memory_management.html) |
> **Next Lesson:** [File Systems](/learn/arduino_to_python/17_file_systems.html)
>
> **Course Home:** [Arduino to Python](/learn/arduino_to_python/)
