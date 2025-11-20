---
layout: lesson
title: Variables and Data Types
author: Kevin McAleer
type: page
cover: /learn/arduino_to_python/assets/cover.jpg
date: 2025-01-20
previous: 03_setup_environment.html
next: 05_control_flow.html
description: Learn how Python's dynamic typing differs from C++'s static typing, with
  side-by-side comparisons of variable declaration and type handling.
percent: 15
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


![Code comparison showing C++ and Python variable declarations](assets/variables.jpg){:class="cover"}

## The Big Difference: Static vs Dynamic Typing

In Arduino C++, you must declare a variable's type before using it. In Python, you just assign a value and Python figures out the type automatically. This is the single biggest conceptual shift you'll encounter.

### Variable Declaration - Side by Side

**Arduino C++:**
```cpp
// Must specify type before using
int motorSpeed = 255;
float temperature = 23.5;
bool isMoving = true;
char letter = 'A';
String message = "Hello";

// Type is fixed - this causes an error:
int count = 10;
count = "ten";  // ERROR! Can't assign string to int
```

**Python:**
```python
# Just assign - Python figures out the type
motor_speed = 255        # Python knows this is an int
temperature = 23.5       # Python knows this is a float
is_moving = True         # Python knows this is a bool
letter = 'A'             # Python knows this is a string (not char)
message = "Hello"        # Also a string

# Type can change - this works fine:
count = 10
count = "ten"  # Totally fine in Python!
```

> ## C++ Developers: Important!
>
> Python has no separate `char` type - single characters are just strings of length 1. And yes, variables can change type at runtime. This flexibility is powerful but requires discipline.

---

## Data Type Equivalents

Here's how Arduino C++ types map to Python:

| C++ Type | Python Type | Example C++ | Example Python |
|----------|-------------|-------------|----------------|
| `int` | `int` | `int x = 42;` | `x = 42` |
| `float` | `float` | `float pi = 3.14;` | `pi = 3.14` |
| `double` | `float` | `double big = 3.14159;` | `big = 3.14159` |
| `bool` | `bool` | `bool on = true;` | `on = True` |
| `char` | `str` | `char c = 'A';` | `c = 'A'` |
| `String` | `str` | `String s = "Hi";` | `s = "Hi"` |
| `byte` | `int` | `byte b = 255;` | `b = 255` |
| `unsigned int` | `int` | `unsigned int n = 1000;` | `n = 1000` |

**Key points:**
- Python's `int` can be any size (no overflow at 32,767!)
- Python has only one string type (`str`), not separate `char` and `String`
- `True` and `False` are capitalized in Python (not lowercase)
- Python has no unsigned types - all ints can be positive or negative

---

## Checking Types at Runtime

In C++, types are checked at compile time. In Python, you can check types while the program runs:

**Arduino C++ (compile-time checking):**
```cpp
int speed = 100;
// Compiler knows speed is int
// Type errors caught before upload
```

**Python (runtime type checking):**
```python
speed = 100
print(type(speed))        # <class 'int'>

speed = "fast"
print(type(speed))        # <class 'str'>

# Check type in code:
if isinstance(speed, int):
    print("Speed is a number")
else:
    print("Speed is not a number")
```

**Why this matters:** In C++, type errors stop you from compiling. In Python, type errors happen when the code runs. This means testing is crucial!

---

## Real-World Robot Example: Motor Speed Control

Let's see how types work in a practical scenario - setting motor speed.

**Arduino C++ version:**
```cpp
// Motor speed controller
const int MOTOR_PIN = 9;
int motorSpeed = 0;  // 0-255

void setup() {
  pinMode(MOTOR_PIN, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // Read speed from serial (expecting 0-255)
  if (Serial.available()) {
    motorSpeed = Serial.parseInt();  // Converts string to int

    // Constrain to valid PWM range
    motorSpeed = constrain(motorSpeed, 0, 255);

    // Set motor speed
    analogWrite(MOTOR_PIN, motorSpeed);

    Serial.print("Motor speed: ");
    Serial.println(motorSpeed);
  }
}
```

**MicroPython version:**
```python
from machine import Pin, PWM
import sys

# Motor speed controller
motor = PWM(Pin(9))
motor.freq(1000)  # 1kHz PWM frequency
motor_speed = 0   # 0-65535 (MicroPython uses 16-bit PWM)

def set_motor_speed(speed):
    """Set motor speed with validation"""
    # Python's dynamic typing means we should validate
    if not isinstance(speed, int):
        print(f"Error: Expected int, got {type(speed).__name__}")
        return

    # Constrain to valid range
    speed = max(0, min(65535, speed))

    motor.duty_u16(speed)
    print(f"Motor speed: {speed}")

# Main loop
while True:
    # Read from stdin (simulating serial input)
    try:
        user_input = input("Enter speed (0-65535): ")
        speed_value = int(user_input)  # Convert string to int
        set_motor_speed(speed_value)
    except ValueError:
        print("Please enter a valid number")
    except KeyboardInterrupt:
        motor.duty_u16(0)  # Stop motor
        break
```

**Key differences explained:**

1. **PWM range**: Arduino uses 8-bit (0-255), MicroPython uses 16-bit (0-65535)
2. **Type validation**: Python version explicitly checks with `isinstance()`
3. **Error handling**: Python uses `try/except` to catch conversion errors
4. **Type conversion**: Both need `parseInt()` / `int()` to convert string input
5. **Function definition**: Python uses `def` instead of C++'s return type declaration

---

## Naming Conventions

You'll notice different variable naming styles:

**Arduino C++ convention:**
```cpp
int motorSpeed;      // camelCase
const int LED_PIN = 13;  // UPPER_CASE for constants
```

**Python convention:**
```python
motor_speed = 0      # snake_case
LED_PIN = 13         # UPPER_CASE for constants (same!)
```

Python style guide (PEP 8) recommends `snake_case` for variables and functions, `UPPER_CASE` for constants. While Python doesn't enforce this, following the convention makes your code more readable to other Python developers.

> ## Pro Tip
>
> Use descriptive names in Python - you're not saving memory like in embedded C++. `motor_speed` is better than `ms` or `m`.
{:.bg-blue}

---

## Constants and Immutability

**Arduino C++ constants:**
```cpp
const int MAX_SPEED = 255;
#define WHEEL_DIAMETER 65  // Also common in Arduino

// Attempting to change causes compile error:
MAX_SPEED = 100;  // ERROR!
```

**Python constants:**
```python
MAX_SPEED = 255
WHEEL_DIAMETER = 65

# Python won't stop you from changing these:
MAX_SPEED = 100  # This works, but breaks convention!

# For true immutability, use a tuple:
SETTINGS = (255, 65)  # Can't modify tuple contents
```

**Important:** Python has no true constants - the UPPER_CASE naming is just a convention meaning "don't change this." Experienced developers respect this convention, but Python won't stop you from modifying it.

---

## Numeric Type Conversions

**Arduino C++ (explicit casting):**
```cpp
int wholePart = 10;
float decimalPart = 3.14;

// Explicit conversion
float result = (float)wholePart + decimalPart;  // 13.14

// Truncation when converting float to int
int truncated = (int)decimalPart;  // 3 (loses .14)
```

**Python (automatic and explicit):**
```python
whole_part = 10
decimal_part = 3.14

# Automatic conversion (int promotes to float)
result = whole_part + decimal_part  # 13.14 (automatically float)

# Explicit conversion
truncated = int(decimal_part)  # 3 (loses .14)
rounded = round(decimal_part)   # 3 (rounds to nearest)
```

**Python advantage:** No need to cast when mixing int and float - Python promotes automatically. But be aware of the result type!

---

## Try It Yourself

**Exercise 1: Type Exploration**
```python
# Create variables of different types
sensor_reading = 1023
voltage = 3.3
is_active = True

# Print their types
print(f"sensor_reading is {type(sensor_reading)}")
print(f"voltage is {type(voltage)}")
print(f"is_active is {type(is_active)}")

# Now change sensor_reading to a string
sensor_reading = "Error"
print(f"Now sensor_reading is {type(sensor_reading)}")
```

**Exercise 2: Type Conversion**
```python
# Simulate reading a sensor value as a string
sensor_string = "512"

# Convert to integer
sensor_int = int(sensor_string)

# Map from 0-1023 to 0-255 (10-bit to 8-bit)
mapped_value = int((sensor_int / 1023) * 255)

print(f"Original: {sensor_string} ({type(sensor_string).__name__})")
print(f"Converted: {sensor_int} ({type(sensor_int).__name__})")
print(f"Mapped: {mapped_value}")
```

**Exercise 3: Build a Type-Safe Function**
```python
def set_servo_angle(angle):
    """Set servo angle with type and range checking"""
    # Add type checking here
    if not isinstance(angle, (int, float)):
        print(f"Error: angle must be number, got {type(angle).__name__}")
        return False

    # Add range checking (servos: 0-180 degrees)
    if angle < 0 or angle > 180:
        print(f"Error: angle {angle} out of range (0-180)")
        return False

    # If we get here, it's valid
    print(f"Setting servo to {angle} degrees")
    return True

# Test it
set_servo_angle(90)       # Should work
set_servo_angle("90")     # Should fail (string)
set_servo_angle(200)      # Should fail (out of range)
```

---

## Common Issues

**Problem: Integer Division Confusion**
```python
# In Python 3, division always returns float
result = 10 / 4
print(result)  # 2.5 (not 2!)

# For integer division like C++, use //
result = 10 // 4
print(result)  # 2 (truncated integer division)
```
**Solution:** Remember that `/` is float division, `//` is integer division.

**Why:** Python 3 changed this to be more intuitive. Python 2 worked like C++, but that caused bugs.

---

**Problem: Type Errors at Runtime**
```python
speed = "100"  # Accidentally a string
motor.duty_u16(speed)  # TypeError: duty_u16() expects int
```
**Solution:** Add type checking in critical functions, especially when receiving external data (serial input, sensor readings).

**Why:** Unlike C++, Python won't catch type mismatches until the code runs.

---

**Problem: Boolean Capitalization**
```cpp
// This works in C++
bool flag = true;

# This FAILS in Python
flag = true  # NameError: name 'true' is not defined

# Must capitalize
flag = True  # Correct!
```
**Solution:** Remember `True` and `False` are capitalized in Python.

**Why:** Python treats them as special constants, not keywords like C++.

---

## Summary: What You've Learned

| Concept | Arduino C++ | Python |
|---------|-------------|--------|
| **Declaration** | `int x = 5;` | `x = 5` |
| **Type required** | Yes, before use | No, inferred |
| **Type can change** | No (compile error) | Yes (at runtime) |
| **Check type** | Compile time | `type()` or `isinstance()` |
| **True/False** | `true`, `false` | `True`, `False` |
| **Division** | `10 / 4 = 2` | `10 / 4 = 2.5` |
| **Integer division** | `10 / 4 = 2` | `10 // 4 = 2` |
| **Naming style** | camelCase | snake_case |

**Key takeaway:** Python's dynamic typing is flexible and fast for prototyping, but requires discipline. Add type checks in critical code paths, especially when interfacing with hardware.

---

> **Next Lesson**: [Control Flow](/learn/arduino_to_python/05_control_flow.html) - if/else, loops, and the key differences in Python syntax
>
> **Previous Lesson**: [Setup Environment](/learn/arduino_to_python/03_setup_environment.html)

---
