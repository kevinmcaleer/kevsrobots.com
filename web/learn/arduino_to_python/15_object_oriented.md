---
layout: lesson
title: Object-Oriented Programming
author: Kevin McAleer
type: page
cover: /learn/arduino_to_python/assets/cover.jpg
date: 2025-01-20
previous: 14_memory_management.html
next: 16_libraries_vs_modules.html
description: Compare C++ and Python class syntax, learn Python's simpler approach
  to OOP, and build reusable robot components with side-by-side examples.
percent: 48
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


![Class diagram showing robot component hierarchy](assets/oop.jpg){:class="cover"}

## You Already Know OOP

If you've used Arduino libraries, you've used object-oriented programming:

```cpp
Servo myServo;        // Creating an object
myServo.attach(9);    // Calling a method
myServo.write(90);    // Calling another method
```

You've been doing OOP all along! Python just makes it more explicit and, in many ways, simpler.

---

## Creating a Class - Side by Side

Let's build a simple LED class that encapsulates pin control.

**Arduino C++ version:**
```cpp
// LED.h
#ifndef LED_H
#define LED_H

class LED {
  private:
    int pin;
    bool state;

  public:
    LED(int ledPin);           // Constructor
    void on();
    void off();
    void toggle();
    bool getState();
};

#endif
```

```cpp
// LED.cpp
#include "Arduino.h"
#include "LED.h"

LED::LED(int ledPin) {
  pin = ledPin;
  state = false;
  pinMode(pin, OUTPUT);
}

void LED::on() {
  digitalWrite(pin, HIGH);
  state = true;
}

void LED::off() {
  digitalWrite(pin, LOW);
  state = false;
}

void LED::toggle() {
  state = !state;
  digitalWrite(pin, state ? HIGH : LOW);
}

bool LED::getState() {
  return state;
}
```

```cpp
// main.ino
#include "LED.h"

LED statusLED(13);

void setup() {
  // LED already initialized by constructor
}

void loop() {
  statusLED.toggle();
  delay(1000);
}
```

**MicroPython version:**
```python
# led.py (separate file) or just in main code
from machine import Pin

class LED:
    """LED controller class"""

    def __init__(self, pin_number):
        """Constructor - initialize LED on specified pin"""
        self.pin = Pin(pin_number, Pin.OUT)
        self.state = False

    def on(self):
        """Turn LED on"""
        self.pin.on()
        self.state = True

    def off(self):
        """Turn LED off"""
        self.pin.off()
        self.state = False

    def toggle(self):
        """Toggle LED state"""
        self.state = not self.state
        self.pin.value(self.state)

    def get_state(self):
        """Return current state"""
        return self.state

# Usage
import time

status_led = LED(13)

while True:
    status_led.toggle()
    time.sleep(1)
```

---

## Key Differences Explained

### 1. No Header Files

**Arduino C++:**
- Requires separate `.h` (header) and `.cpp` (implementation) files
- Header guards (`#ifndef`, `#define`, `#endif`) to prevent double inclusion
- Must declare everything before implementing it

**Python:**
- Everything in one file (or can split into modules if you want)
- No header guards needed
- Define the class and use it - that's it!

---

### 2. Constructor Syntax

**Arduino C++:**
```cpp
LED::LED(int ledPin) {  // ClassName::ClassName(params)
  pin = ledPin;
}
```

**Python:**
```python
def __init__(self, pin_number):  # Always named __init__
    self.pin_number = pin_number
```

**Important:**
- Python constructors are **always** called `__init__` (double underscores)
- Every method must have `self` as the first parameter (like `this` pointer in C++)
- Use `self.variable` to access instance variables

---

### 3. The "self" Parameter

**Arduino C++:**
```cpp
class LED {
  private:
    int pin;  // Instance variable

  public:
    void on() {
      digitalWrite(pin, HIGH);  // Can access 'pin' directly
      // Or explicitly: digitalWrite(this->pin, HIGH);
    }
};
```

**Python:**
```python
class LED:
    def __init__(self, pin_number):
        self.pin_number = pin_number  # Instance variable

    def on(self):
        # MUST use self.pin_number, not just pin_number
        self.pin.on()
```

**Critical difference:** In C++, `this->` is optional. In Python, `self.` is **required** to access instance variables. Forgetting this is the #1 mistake Arduino developers make!

---

### 4. Access Modifiers

**Arduino C++:**
```cpp
class Motor {
  private:         // Only accessible within class
    int speed;

  protected:       // Accessible in subclasses
    int direction;

  public:          // Accessible everywhere
    void setSpeed(int s);
};
```

**Python:**
```python
class Motor:
    def __init__(self):
        self.speed = 0           # Public (by convention)
        self._direction = 0      # "Protected" (single underscore)
        self.__internal_state = 0  # "Private" (double underscore)
```

**Important:** Python doesn't enforce private/protected - it's based on **convention**:
- No underscore = public
- Single underscore `_name` = "internal, don't use outside class" (not enforced)
- Double underscore `__name` = name mangling (harder to access, but not truly private)

> ## C++ Developers: Important!
>
> Python's philosophy is "we're all consenting adults here." If someone really wants to access a "private" variable, they can. It's about signaling intent, not enforcement.

---

## Real-World Robot Example: Motor Controller Class

Let's build a complete motor controller with direction, speed, and safety features.

**Arduino C++ version:**
```cpp
// Motor.h
#ifndef MOTOR_H
#define MOTOR_H

class Motor {
  private:
    int enablePin;
    int in1Pin;
    int in2Pin;
    int currentSpeed;
    bool isRunning;

    void applySpeed();

  public:
    Motor(int enable, int in1, int in2);
    void forward(int speed);
    void reverse(int speed);
    void stop();
    void brake();
    int getSpeed();
    bool running();
};

#endif
```

```cpp
// Motor.cpp
#include "Arduino.h"
#include "Motor.h"

Motor::Motor(int enable, int in1, int in2) {
  enablePin = enable;
  in1Pin = in1;
  in2Pin = in2;
  currentSpeed = 0;
  isRunning = false;

  pinMode(enablePin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
}

void Motor::forward(int speed) {
  currentSpeed = constrain(speed, 0, 255);
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
  applySpeed();
  isRunning = true;
}

void Motor::reverse(int speed) {
  currentSpeed = constrain(speed, 0, 255);
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);
  applySpeed();
  isRunning = true;
}

void Motor::stop() {
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
  currentSpeed = 0;
  applySpeed();
  isRunning = false;
}

void Motor::brake() {
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, HIGH);
  currentSpeed = 0;
  applySpeed();
  isRunning = false;
}

void Motor::applySpeed() {
  analogWrite(enablePin, currentSpeed);
}

int Motor::getSpeed() {
  return currentSpeed;
}

bool Motor::running() {
  return isRunning;
}
```

```cpp
// main.ino
#include "Motor.h"

Motor leftMotor(9, 7, 8);
Motor rightMotor(10, 11, 12);

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Move forward
  leftMotor.forward(200);
  rightMotor.forward(200);
  delay(2000);

  // Stop
  leftMotor.stop();
  rightMotor.stop();
  delay(1000);

  // Turn right (left motor forward, right motor reverse)
  leftMotor.forward(150);
  rightMotor.reverse(150);
  delay(1000);

  leftMotor.stop();
  rightMotor.stop();
  delay(1000);
}
```

**MicroPython version:**
```python
from machine import Pin, PWM
import time

class Motor:
    """Motor controller with direction and speed control"""

    def __init__(self, enable_pin, in1_pin, in2_pin):
        """
        Initialize motor controller

        Args:
            enable_pin: PWM-capable pin for speed control
            in1_pin: Direction control pin 1
            in2_pin: Direction control pin 2
        """
        self.enable = PWM(Pin(enable_pin))
        self.in1 = Pin(in1_pin, Pin.OUT)
        self.in2 = Pin(in2_pin, Pin.OUT)
        self.enable.freq(1000)  # 1kHz PWM

        self.current_speed = 0
        self.is_running = False

    def forward(self, speed):
        """
        Run motor forward

        Args:
            speed: Motor speed (0-65535)
        """
        self.current_speed = max(0, min(65535, speed))  # Constrain
        self.in1.on()
        self.in2.off()
        self._apply_speed()
        self.is_running = True

    def reverse(self, speed):
        """
        Run motor in reverse

        Args:
            speed: Motor speed (0-65535)
        """
        self.current_speed = max(0, min(65535, speed))  # Constrain
        self.in1.off()
        self.in2.on()
        self._apply_speed()
        self.is_running = True

    def stop(self):
        """Coast to a stop"""
        self.in1.off()
        self.in2.off()
        self.current_speed = 0
        self._apply_speed()
        self.is_running = False

    def brake(self):
        """Active braking"""
        self.in1.on()
        self.in2.on()
        self.current_speed = 0
        self._apply_speed()
        self.is_running = False

    def _apply_speed(self):
        """Internal method to set PWM duty cycle"""
        self.enable.duty_u16(self.current_speed)

    def get_speed(self):
        """Return current speed setting"""
        return self.current_speed

    def running(self):
        """Return True if motor is running"""
        return self.is_running

# Usage
left_motor = Motor(enable_pin=9, in1_pin=7, in2_pin=8)
right_motor = Motor(enable_pin=10, in1_pin=11, in2_pin=12)

while True:
    # Move forward
    left_motor.forward(52428)   # ~80% speed
    right_motor.forward(52428)
    time.sleep(2)

    # Stop
    left_motor.stop()
    right_motor.stop()
    time.sleep(1)

    # Turn right
    left_motor.forward(39321)   # ~60% speed
    right_motor.reverse(39321)
    time.sleep(1)

    left_motor.stop()
    right_motor.stop()
    time.sleep(1)
```

---

## Inheritance - Building on Existing Classes

**Arduino C++ version:**
```cpp
// Base class
class Motor {
  protected:
    int enablePin;
    int speed;

  public:
    Motor(int pin) : enablePin(pin), speed(0) {
      pinMode(enablePin, OUTPUT);
    }

    virtual void setSpeed(int s) {
      speed = constrain(s, 0, 255);
      analogWrite(enablePin, speed);
    }
};

// Derived class
class ServoMotor : public Motor {
  private:
    Servo servo;

  public:
    ServoMotor(int pin) : Motor(pin) {
      servo.attach(pin);
    }

    void setSpeed(int s) override {
      // Convert speed to servo angle
      int angle = map(s, 0, 255, 0, 180);
      servo.write(angle);
    }
};
```

**Python version:**
```python
from machine import Pin, PWM, Servo
import time

class Motor:
    """Base motor class"""

    def __init__(self, pin):
        """Initialize with PWM pin"""
        self.enable = PWM(Pin(pin))
        self.enable.freq(1000)
        self.speed = 0

    def set_speed(self, speed):
        """Set motor speed (0-65535)"""
        self.speed = max(0, min(65535, speed))
        self.enable.duty_u16(self.speed)

class ServoMotor(Motor):
    """Servo motor - inherits from Motor"""

    def __init__(self, pin):
        """Initialize servo on specified pin"""
        # Don't call parent __init__ - servo is different
        self.servo = Servo(Pin(pin))
        self.speed = 0

    def set_speed(self, speed):
        """Override: Convert speed to servo angle"""
        # Map 0-65535 to 0-180 degrees
        angle = int((speed / 65535) * 180)
        self.servo.write(angle)
        self.speed = speed

# Usage
dc_motor = Motor(9)
servo = ServoMotor(10)

dc_motor.set_speed(32768)   # 50% speed
servo.set_speed(32768)      # 90 degrees (middle)
```

**Key difference:** Python uses `class ChildClass(ParentClass):` syntax instead of C++'s `class ChildClass : public ParentClass`.

---

## Docstrings vs Comments

**Arduino C++:**
```cpp
/**
 * Set motor speed with range validation
 * @param speed Motor speed (0-255)
 * @return true if successful
 */
bool setSpeed(int speed) {
  // Implementation
}
```

**Python:**
```python
def set_speed(self, speed):
    """
    Set motor speed with range validation

    Args:
        speed (int): Motor speed (0-65535)

    Returns:
        bool: True if successful
    """
    # Implementation
```

**Python advantage:** Docstrings are accessible at runtime and can be viewed in the REPL with `help(motor.set_speed)`.

---

## Class Variables vs Instance Variables

**Arduino C++:**
```cpp
class Robot {
  private:
    static int robotCount;  // Class variable (shared)
    int id;                 // Instance variable (unique)

  public:
    Robot() {
      id = robotCount++;
    }

    static int getRobotCount() {
      return robotCount;
    }
};

int Robot::robotCount = 0;  // Define static variable
```

**Python:**
```python
class Robot:
    """Robot with instance tracking"""

    robot_count = 0  # Class variable (shared by all instances)

    def __init__(self):
        """Constructor - each robot gets unique ID"""
        self.id = Robot.robot_count  # Instance variable
        Robot.robot_count += 1

    @classmethod
    def get_robot_count(cls):
        """Class method to access class variable"""
        return cls.robot_count

# Usage
robot1 = Robot()
robot2 = Robot()
robot3 = Robot()

print(f"Created {Robot.get_robot_count()} robots")  # Prints: Created 3 robots
print(f"Robot 2 ID: {robot2.id}")  # Prints: Robot 2 ID: 1
```

**Key difference:**
- Class variables in Python are defined **outside** `__init__`
- Access class variables with `ClassName.variable` or `self.__class__.variable`
- `@classmethod` decorator for methods that operate on class (like `static` in C++)

---

## Try It Yourself

**Exercise 1: Ultrasonic Sensor Class**
```python
from machine import Pin
import time

class UltrasonicSensor:
    """HC-SR04 ultrasonic distance sensor"""

    def __init__(self, trigger_pin, echo_pin):
        """Initialize sensor with trigger and echo pins"""
        # TODO: Create Pin objects for trigger (output) and echo (input)
        pass

    def get_distance(self):
        """
        Measure distance in centimeters

        Returns:
            float: Distance in cm, or None if out of range
        """
        # TODO: Implement ultrasonic pulse timing
        # 1. Send 10μs pulse on trigger pin
        # 2. Measure echo pulse duration
        # 3. Calculate distance: (duration * 0.0343) / 2
        pass

# Test your class
# sensor = UltrasonicSensor(trigger_pin=16, echo_pin=17)
# while True:
#     distance = sensor.get_distance()
#     print(f"Distance: {distance} cm")
#     time.sleep(0.5)
```

**Exercise 2: RGB LED Class**
```python
from machine import Pin, PWM

class RGBLED:
    """RGB LED with color mixing"""

    def __init__(self, red_pin, green_pin, blue_pin):
        """Initialize RGB LED with three PWM pins"""
        # TODO: Create three PWM objects
        pass

    def set_color(self, red, green, blue):
        """
        Set RGB color

        Args:
            red, green, blue: Color values (0-255)
        """
        # TODO: Convert 0-255 to 0-65535 and set duty cycles
        pass

    def red(self):
        """Set to red"""
        # TODO: Implement
        pass

    def green(self):
        """Set to green"""
        pass

    def blue(self):
        """Set to blue"""
        pass

    def off(self):
        """Turn off all LEDs"""
        pass

# Test your class
# rgb = RGBLED(red_pin=11, green_pin=12, blue_pin=13)
# rgb.red()
# time.sleep(1)
# rgb.set_color(128, 0, 128)  # Purple
```

**Exercise 3: Two-Wheeled Robot Class**
```python
from machine import Pin, PWM
import time

class TwoWheelRobot:
    """Differential drive robot"""

    def __init__(self, left_enable, left_in1, left_in2,
                 right_enable, right_in1, right_in2):
        """Initialize with motor control pins"""
        # TODO: Create two Motor objects (from earlier example)
        # Hint: You can use the Motor class we created earlier!
        pass

    def forward(self, speed):
        """Move forward at specified speed"""
        pass

    def reverse(self, speed):
        """Move backward at specified speed"""
        pass

    def turn_left(self, speed):
        """Turn left (right motor forward, left motor reverse)"""
        pass

    def turn_right(self, speed):
        """Turn right (left motor forward, right motor reverse)"""
        pass

    def stop(self):
        """Stop both motors"""
        pass

# Test your robot
# robot = TwoWheelRobot(
#     left_enable=9, left_in1=7, left_in2=8,
#     right_enable=10, right_in1=11, right_in2=12
# )
# robot.forward(32768)
# time.sleep(2)
# robot.turn_right(26214)
# time.sleep(1)
# robot.stop()
```

---

## Common Issues

**Problem: Forgetting "self"**
```python
class LED:
    def __init__(self, pin):
        self.pin = Pin(pin, Pin.OUT)  # Correct

    def on(self):
        pin.on()  # ERROR! NameError: name 'pin' is not defined
```
**Solution:** Always use `self.variable_name` to access instance variables.

**Why:** Python doesn't have implicit `this->` like C++. You must be explicit.

---

**Problem: Incorrect constructor name**
```python
class Motor:
    def Motor(self, pin):  # Wrong! This is just a regular method
        self.pin = pin

motor = Motor(9)  # Doesn't call Motor() method!
```
**Solution:** Constructor must be named `__init__` (with double underscores).

**Why:** Python has specific "magic methods" with double underscores for special purposes.

---

**Problem: Accessing private attributes**
```python
class Motor:
    def __init__(self):
        self.__speed = 0  # "Private" with double underscore

motor = Motor()
print(motor.__speed)  # AttributeError!
```
**Solution:** Use single underscore for "protected" or make it public.

**Why:** Double underscore triggers name mangling - Python renames it to `_Motor__speed`.

---

## Summary: OOP in Arduino C++ vs Python

| Feature | Arduino C++ | Python |
|---------|-------------|--------|
| **Class definition** | Separate `.h` and `.cpp` | Single file |
| **Constructor** | `ClassName(params)` | `def __init__(self, params):` |
| **Instance ref** | `this->` (optional) | `self.` (required) |
| **Private** | `private:` section | `__variable` (name mangling) |
| **Protected** | `protected:` section | `_variable` (convention) |
| **Public** | `public:` section | Default (no underscore) |
| **Inheritance** | `: public BaseClass` | `(BaseClass)` |
| **Method override** | `override` keyword | Just redefine method |
| **Static method** | `static` keyword | `@staticmethod` decorator |
| **Class method** | `static` + class logic | `@classmethod` decorator |
| **Documentation** | `/** */` comments | `"""docstring"""` |

**Key takeaways:**
- Python OOP is **simpler** - no header files, less boilerplate
- **Must use `self`** explicitly in all methods
- **Constructor is `__init__`** - don't forget double underscores
- **No enforced private/protected** - convention-based instead
- **Docstrings are better** than comments - accessible at runtime
- **Classes make hardware control cleaner** - encapsulate related functionality

---

> **Next Lesson**: [Libraries vs Modules](/learn/arduino_to_python/16_libraries_vs_modules.html) - How Python's import system compares to Arduino libraries
>
> **Previous Lesson**: [Memory Management](/learn/arduino_to_python/14_memory_management.html)

---
