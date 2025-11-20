---
layout: lesson
title: From setup() and loop() to Python Functions
author: Kevin McAleer
type: page
cover: /learn/arduino_to_python/assets/functions.jpg
date: 2025-01-20
previous: 05_control_flow.html
next: 07_strings.html
description: Understanding function definitions, parameters, and the Python program
  structure
percent: 21
duration: 12
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


![Functions cover image](assets/functions.jpg){:class="cover"}

## Introduction

Every Arduino sketch you've written has two mandatory functions: `setup()` and `loop()`. This structure is so ingrained that you probably don't even think about it anymore.

Python has no such structure. There's no `setup()`, no `loop()`, and no special entry point. Yet Python code runs just fine. How?

In this lesson, you'll learn how Python functions work, how to structure your programs without `setup()` and `loop()`, and discover Python's powerful function features that don't exist in Arduino C++ - like default parameters and keyword arguments.

## Arduino's Mandatory Structure

In Arduino, every program looks like this:

```cpp
void setup() {
  // Runs once when board powers on or resets
  pinMode(13, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // Runs forever, called repeatedly
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  delay(1000);
}
```

**The Arduino framework provides:**
- Automatic initialization (calling `setup()` once)
- Infinite loop (calling `loop()` repeatedly)
- Hidden `main()` function you never see

This works great for simple embedded programs. But Python takes a different, more flexible approach.

## Python's Flexible Structure

In Python, there's no mandatory structure. Your program starts at the top and runs line by line:

```python
from machine import Pin
import time

# This runs immediately when the program starts
led = Pin(25, Pin.OUT)

# This also runs immediately
print("Program starting...")

# If you want an infinite loop, you create it explicitly
while True:
    led.on()
    time.sleep(1)
    led.off()
    time.sleep(1)
```

**Key differences:**
- No `setup()` - initialization code just... exists at the top level
- No automatic `loop()` - you create loops explicitly with `while True:`
- More flexibility - you can structure your program however you want

## Function Definition: Type Declarations vs def

### Arduino C++ Function
```cpp
// Return type required
int calculateSpeed(int distance, int time) {
  return distance / time;
}

// void for no return
void printStatus(String message) {
  Serial.println(message);
}
```

**Required elements:**
- Return type (`int`, `void`, `float`, etc.)
- Function name
- Parameter types
- Curly braces for body

### Python Function
```python
# No return type specified
def calculate_speed(distance, time):
    return distance / time

# No need for "void"
def print_status(message):
    print(message)
```

**Required elements:**
- `def` keyword
- Function name (snake_case by convention)
- Colon
- Indented body
- **No type declarations needed** (dynamic typing)

## Side-by-Side Comparison: Motor Control Function

### Arduino C++
```cpp
// Must declare return type and parameter types
void setMotorSpeed(int pin, int speed) {
  if (speed < 0) {
    speed = 0;
  }
  if (speed > 255) {
    speed = 255;
  }
  analogWrite(pin, speed);
}

// Usage
void loop() {
  setMotorSpeed(9, 180);
  delay(1000);
}
```

### MicroPython
```python
def set_motor_speed(pin, speed):
    # Constrain speed to 0-255
    if speed < 0:
        speed = 0
    if speed > 255:
        speed = 255

    # Assuming PWM is set up
    pin.duty_u16(speed * 257)  # Convert to 16-bit

# Usage
while True:
    set_motor_speed(motor_pin, 180)
    time.sleep(1)
```

## Return Values: Explicit vs Implicit

### Arduino C++
```cpp
// Must declare return type
int add(int a, int b) {
  return a + b;  // Must return int
}

// void means no return
void blink() {
  digitalWrite(13, HIGH);
  // No return statement needed
}
```

### Python
```python
# Return type inferred from what you return
def add(a, b):
    return a + b  # Can return any type

# Function without return implicitly returns None
def blink():
    led.on()
    # Returns None automatically

# Can return different types
def get_status(sensor_value):
    if sensor_value > 100:
        return "High"  # String
    else:
        return sensor_value  # Number
```

**Python's flexibility:** Functions can return any type, or even different types based on conditions. No compile-time checking - more freedom, more responsibility.

## Default Parameters: A Python Superpower

This is where Python really shines. Default parameters don't exist in standard Arduino C++.

### Arduino C++ (No Default Parameters)
```cpp
// Must create multiple functions for optional parameters
void moveRobot(int speed) {
  moveRobot(speed, 1000);  // Call overload with default duration
}

void moveRobot(int speed, int duration) {
  analogWrite(MOTOR_PIN, speed);
  delay(duration);
}
```

### Python with Default Parameters
```python
def move_robot(speed, duration=1000):
    """Move robot with optional duration (default 1 second)"""
    motor_pin.duty_u16(speed * 257)
    time.sleep(duration / 1000)

# Can call with one or two arguments
move_robot(180)           # Uses default duration (1000ms)
move_robot(180, 2000)     # Explicit duration (2000ms)
move_robot(speed=180)     # Keyword argument with default duration
move_robot(speed=180, duration=500)  # Both as keyword arguments
```

**Benefits:**
- One function instead of multiple overloads
- Clear intent - you see the default right in the definition
- Makes code more maintainable

## Keyword Arguments: Call by Name

Python lets you specify arguments by name, not just position. This is incredibly useful for functions with many parameters.

### Arduino C++ (Position Only)
```cpp
// Must remember parameter order
void configureRobot(int speed, int direction, bool lights, int volume) {
  // Implementation
}

// Easy to mix up parameters!
configureRobot(100, 90, true, 50);  // Which is which?
```

### Python with Keyword Arguments
```python
def configure_robot(speed, direction, lights=True, volume=50):
    """Configure robot with named parameters"""
    # Implementation
    pass

# Can call with positional arguments
configure_robot(100, 90, True, 50)

# Or use keyword arguments (much clearer!)
configure_robot(speed=100, direction=90, lights=True, volume=50)

# Can mix positional and keyword
configure_robot(100, 90, lights=False)

# Can provide arguments in any order (when using keywords)
configure_robot(direction=90, speed=100, volume=30, lights=False)
```

**Benefits:**
- Self-documenting code
- Can't mix up parameter order
- Optional parameters are obvious
- Makes refactoring easier

## Structuring a Python Program Without setup()/loop()

Here's how to structure a MicroPython program that mimics Arduino's setup/loop pattern:

### Arduino C++
```cpp
// Global variables
int ledPin = 13;
int sensorPin = A0;
int sensorValue = 0;

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
  Serial.println("Robot starting...");
}

void loop() {
  sensorValue = analogRead(sensorPin);
  digitalWrite(ledPin, sensorValue > 512 ? HIGH : LOW);
  delay(100);
}
```

### Python Equivalent (Explicit Structure)
```python
from machine import Pin, ADC
import time

# Global variables (module level)
led_pin = Pin(25, Pin.OUT)
sensor_pin = ADC(26)

def setup():
    """Initialization - replaces Arduino setup()"""
    print("Robot starting...")
    # Additional setup if needed

def loop():
    """Main loop body - replaces Arduino loop()"""
    sensor_value = sensor_pin.read_u16()
    if sensor_value > 32768:  # Middle of 16-bit range
        led_pin.on()
    else:
        led_pin.off()
    time.sleep(0.1)

def main():
    """Main program entry point"""
    setup()
    while True:
        loop()

# Only runs when this file is executed (not when imported)
if __name__ == '__main__':
    main()
```

### Python Idiomatic Style (More Pythonic)
```python
from machine import Pin, ADC
import time

# Initialize hardware at module level
led = Pin(25, Pin.OUT)
sensor = ADC(26)

print("Robot starting...")

# Main loop - no wrapper functions needed
while True:
    sensor_value = sensor.read_u16()
    led.value(1 if sensor_value > 32768 else 0)
    time.sleep(0.1)
```

**Which style to use?**
- **Explicit setup/loop:** Good for Arduino developers transitioning
- **Idiomatic Python:** More concise, better for pure Python development
- **Choose based on:** Your comfort level and team familiarity

## Multiple Return Values: Tuples

Python functions can return multiple values easily. In C++, you'd need pointers or structs.

### Arduino C++ (Returning Multiple Values)
```cpp
// Need to pass pointers or use struct
void readSensors(int *left, int *right) {
  *left = analogRead(A0);
  *right = analogRead(A1);
}

// Usage
int leftSensor, rightSensor;
readSensors(&leftSensor, &rightSensor);
```

### Python (Tuples)
```python
def read_sensors():
    """Returns tuple of (left, right) sensor values"""
    left = sensor_left.read_u16()
    right = sensor_right.read_u16()
    return left, right  # Returns tuple automatically

# Unpack into separate variables
left_value, right_value = read_sensors()

# Or keep as tuple
sensor_values = read_sensors()
print(sensor_values[0], sensor_values[1])
```

**Real-world robot example:**
```python
def get_robot_state():
    """Returns (speed, direction, battery_level)"""
    speed = measure_speed()
    direction = compass.heading()
    battery = adc.read_u16() * 3.3 / 65535
    return speed, direction, battery

# Easy to use
speed, direction, battery = get_robot_state()
print(f"Speed: {speed}, Direction: {direction}°, Battery: {battery}V")
```

## Docstrings: Self-Documenting Functions

Python has built-in function documentation that's way better than comments.

### Arduino C++ (Comments)
```cpp
// Moves the robot forward at specified speed
// Parameters:
//   speed - PWM value 0-255
//   duration - movement time in milliseconds
// Returns: nothing
void moveForward(int speed, int duration) {
  // Implementation
}
```

### Python (Docstrings)
```python
def move_forward(speed, duration=1000):
    """
    Move the robot forward at specified speed.

    Args:
        speed (int): PWM value 0-255
        duration (int): Movement time in milliseconds (default 1000)

    Returns:
        None

    Example:
        move_forward(180, 2000)  # Move at speed 180 for 2 seconds
    """
    # Implementation
    pass

# View documentation interactively in REPL
>>> help(move_forward)
# Shows the docstring!
```

**Benefits:**
- Documentation is part of the code
- Accessible via `help()` in REPL
- IDEs can show it as you type
- Good practice for professional code

## Variable Scope: Similar but Different

Scope works similarly in both languages, with subtle differences.

### Arduino C++
```cpp
int globalVar = 100;  // Global

void setup() {
  int localVar = 50;  // Local to setup()
  globalVar = 200;    // Can modify global
}

void loop() {
  // localVar not accessible here
  Serial.println(globalVar);  // Can read global
}
```

### Python
```python
global_var = 100  # Module-level (like global)

def function():
    local_var = 50  # Local to function
    print(global_var)  # Can read module-level

    # To modify module-level, must declare global
    global global_var
    global_var = 200

# Main code
function()
print(global_var)  # Shows 200
```

**Key difference:** In Python, you must use the `global` keyword to modify module-level variables from within functions. Reading them works without `global`.

## Lambda Functions: Anonymous Quick Functions

Python has lambda functions for simple, one-line operations. No Arduino equivalent.

```python
# Regular function
def add(a, b):
    return a + b

# Lambda equivalent (anonymous function)
add = lambda a, b: a + b

# Useful for quick operations
sensor_values = [100, 200, 150, 300, 50]
filtered = list(filter(lambda x: x > 100, sensor_values))
# Result: [200, 150, 300]

# Sort sensors by distance
sensors = [(1, 45), (2, 30), (3, 50)]  # (id, distance)
sorted_sensors = sorted(sensors, key=lambda s: s[1])
# Sorted by distance: [(2, 30), (1, 45), (3, 50)]
```

Use lambdas for simple operations. For anything complex, use regular functions.

## Try It Yourself

### Exercise 1: Convert Arduino Functions to Python
Convert this Arduino code to Python with default parameters:

```cpp
void setLED(int pin, int brightness, int duration) {
  analogWrite(pin, brightness);
  delay(duration);
}
```

Make `brightness` default to 255 and `duration` default to 1000.

### Exercise 2: Multiple Return Values
Write a Python function `analyze_sensor(value)` that returns three values:
1. Whether value is in "safe" range (100-400)
2. The value normalized to 0-1 range (assuming 0-1023)
3. A status string ("low", "ok", or "high")

Test it with different sensor values.

### Exercise 3: Refactor Arduino Code
Take this Arduino sketch and convert it to Python:

```cpp
int motorPin = 9;
int sensorPin = A0;

void setup() {
  pinMode(motorPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  int sensorValue = analogRead(sensorPin);
  int motorSpeed = map(sensorValue, 0, 1023, 0, 255);
  analogWrite(motorPin, motorSpeed);
  delay(50);
}
```

Create it with:
1. Explicit `setup()` and `loop()` functions
2. Idiomatic Python style (no wrapper functions)

Which style do you prefer?

### Exercise 4: Keyword Arguments
Write a function `configure_servo(pin, angle, speed, smooth)` where:
- `pin` is required
- `angle` defaults to 90
- `speed` defaults to 100
- `smooth` defaults to True

Test calling it with various combinations of positional and keyword arguments.

## Common Issues

### Issue: "NameError: name 'move_forward' is not defined"
**Problem:** Calling a function before it's defined.

```python
# Wrong - function not defined yet
move_forward(100)

def move_forward(speed):
    pass
```

**Solution:** Define functions before calling them, or put calls inside `if __name__ == '__main__'`:

```python
def move_forward(speed):
    pass

# Now it's defined
move_forward(100)
```

### Issue: "TypeError: function() got an unexpected keyword argument"
**Problem:** Trying to use keyword argument with a function that doesn't have that parameter.

**Solution:** Check function definition and parameter names match exactly.

### Issue: Global Variable Not Updating
**Problem:**
```python
counter = 0

def increment():
    counter = counter + 1  # Error!

increment()
```

**Solution:** Use `global` keyword:
```python
counter = 0

def increment():
    global counter
    counter = counter + 1

increment()
print(counter)  # 1
```

## Summary

| Feature | Arduino C++ | Python |
|---------|-------------|---------|
| **Function definition** | `int func(int x)` | `def func(x):` |
| **Return type** | Must specify | Inferred |
| **Default parameters** | No (use overloading) | Yes `def func(x=10):` |
| **Keyword arguments** | No | Yes `func(x=10)` |
| **Multiple returns** | Pointers/structs | Tuples `return a, b` |
| **Documentation** | Comments | Docstrings |
| **setup()/loop()** | Mandatory | Create explicitly |
| **Anonymous functions** | No | Lambdas |

**Key Takeaways:**
- Python functions are more flexible (no type declarations)
- Default parameters reduce code duplication
- Keyword arguments make code self-documenting
- No mandatory `setup()`/`loop()` - create your own structure
- Tuples make returning multiple values easy
- Docstrings provide built-in documentation

**Python advantages:**
- Less boilerplate code
- More expressive function signatures
- Easier to refactor

**C++ advantages:**
- Type safety catches errors at compile time
- Can be more efficient (no runtime type checking)

In the next lesson, we'll explore string handling - where Python really shows its power compared to Arduino's String class.

---

> **Previous Lesson:** [Control Flow](/learn/arduino_to_python/05_control_flow.html) |
> **Next Lesson:** [Strings](/learn/arduino_to_python/07_strings.html)
>
> **Course Home:** [Arduino to Python](/learn/arduino_to_python/)
