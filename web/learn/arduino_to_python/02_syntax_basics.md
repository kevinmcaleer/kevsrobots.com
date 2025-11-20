---
layout: lesson
title: Syntax Basics - No More Semicolons!
author: Kevin McAleer
type: page
cover: /learn/arduino_to_python/assets/cover.jpg
date: 2025-11-20
previous: 01_why_micropython.html
next: 03_control_flow.html
description: Learn the fundamental syntax differences between Arduino C++ and MicroPython
  including variables, types, comments, and indentation
percent: 24
duration: 7
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


## The Big Syntax Changes

Moving from Arduino C++ to MicroPython means unlearning a few habits and learning new ones. The good news? Python syntax is generally simpler and more readable.

**The three biggest changes:**
1. No semicolons at the end of lines
2. Indentation defines code blocks (no curly braces)
3. No need to declare variable types

Let's dive into each difference with side-by-side examples.

## Semicolons: Gone!

In Arduino C++, every statement ends with a semicolon. Forget one, and your code won't compile. Python doesn't use semicolons at all.

**Arduino C++:**
```cpp
int ledPin = 13;
int brightness = 128;
digitalWrite(ledPin, HIGH);
```

**MicroPython:**
```python
led_pin = 13
brightness = 128
led.value(1)
```

Notice how much cleaner Python looks? Each line is a statement - no semicolons needed. Python uses line breaks to separate statements.

**Common mistake:** Adding semicolons in Python (they're technically allowed but unnecessary and considered bad style).

## Variables and Types

In C++, you must declare the type of every variable. In Python, the type is inferred automatically.

**Arduino C++:**
```cpp
// Must declare types explicitly
int counter = 0;
float temperature = 23.5;
bool isRunning = true;
String message = "Hello";
char letter = 'A';
```

**MicroPython:**
```python
# Types are inferred automatically
counter = 0              # int
temperature = 23.5       # float
is_running = True        # bool (note: capital T!)
message = "Hello"        # str
letter = 'A'             # str (single chars are also strings)
```

Key differences:
- No type declarations in Python
- Boolean values are `True` and `False` (capitalized!)
- Both single and double quotes work for strings
- Variables use `snake_case` in Python (convention), not `camelCase`

## Dynamic Typing

Python variables can change type during runtime. This is powerful but requires care:

**MicroPython:**
```python
value = 42           # int
print(value)         # 42

value = "Hello"      # Now it's a string!
print(value)         # Hello

value = 3.14         # Now it's a float!
print(value)         # 3.14
```

You can't do this in C++ - once an int, always an int. Python's flexibility is useful but can cause bugs if you're not careful.

## Comments

Comments work similarly, with one syntax change:

**Arduino C++:**
```cpp
// Single-line comment
int x = 10;  // End-of-line comment

/* Multi-line comment
   can span many lines
   like this */
```

**MicroPython:**
```python
# Single-line comment
x = 10  # End-of-line comment

"""Multi-line comment
can span many lines
like this"""

# Or use multiple single-line comments
# which is also common
# in Python code
```

Most Python programmers use `#` for everything, even multi-line comments. The triple-quote style (`"""text"""`) is technically a string literal, not a comment, but it works the same way.

## Indentation: This Changes Everything

In C++, curly braces `{}` define code blocks. Indentation is just for readability. In Python, **indentation IS the syntax**. No braces needed.

**Arduino C++:**
```cpp
void setup() {
    pinMode(13, OUTPUT);
    if (temperature > 25) {
        digitalWrite(13, HIGH);
        Serial.println("Hot!");
    }
}

// Bad indentation, but still works:
void loop(){
if(temperature>25){
digitalWrite(13,HIGH);
}}
```

**MicroPython:**
```python
# Indentation defines the block
from machine import Pin
led = Pin(13, Pin.OUT)

if temperature > 25:
    led.on()
    print("Hot!")

# Bad indentation = syntax error!
if temperature > 25:
led.on()  # ERROR: IndentationError
```

**Indentation rules:**
- Use 4 spaces per indentation level (Python standard)
- Never mix tabs and spaces (causes errors)
- Consistent indentation is required, not optional
- Colons `:` start indented blocks

## String Operations

Strings are much easier in Python:

**Arduino C++:**
```cpp
String name = "Arduino";
String greeting = "Hello, " + name + "!";
int length = greeting.length();
String upper = greeting;
upper.toUpperCase();
```

**MicroPython:**
```python
name = "Arduino"
greeting = f"Hello, {name}!"  # f-string (formatted string)
# or: greeting = "Hello, " + name + "!"
length = len(greeting)
upper = greeting.upper()
```

Python's f-strings (formatted strings) are incredibly powerful:

```python
temperature = 23.5
humidity = 67

# Arduino way
message = "Temp: " + str(temperature) + "C, Humidity: " + str(humidity) + "%"

# Python way
message = f"Temp: {temperature}C, Humidity: {humidity}%"
```

Much cleaner!

## Type Conversion

Converting between types is straightforward in both languages:

**Arduino C++:**
```cpp
int number = 42;
String text = String(number);
float decimal = float(number);

String numText = "123";
int parsed = numText.toInt();
```

**MicroPython:**
```python
number = 42
text = str(number)
decimal = float(number)

num_text = "123"
parsed = int(num_text)
```

Python's conversion functions are simpler: `int()`, `float()`, `str()`, `bool()`.

## Printing and Debugging

Serial output is much simpler in Python:

**Arduino C++:**
```cpp
Serial.begin(9600);
int value = 42;
Serial.print("Value: ");
Serial.println(value);
```

**MicroPython:**
```python
value = 42
print(f"Value: {value}")
# No setup needed - print() just works!
```

## Constants

Arduino uses `const` or `#define`. Python uses naming convention:

**Arduino C++:**
```cpp
const int LED_PIN = 13;
#define MAX_TEMP 30
```

**MicroPython:**
```python
# Use ALL_CAPS for constants (convention, not enforced)
LED_PIN = 13
MAX_TEMP = 30

# These can technically be changed (Python doesn't enforce const)
# but ALL_CAPS signals "don't change this"
```

## Side-by-Side Example

Let's convert a complete Arduino snippet:

**Arduino C++:**
```cpp
// Sensor reading
int sensorPin = A0;
int ledPin = 13;
int threshold = 500;

void setup() {
    Serial.begin(9600);
    pinMode(ledPin, OUTPUT);
}

void loop() {
    int sensorValue = analogRead(sensorPin);
    Serial.print("Sensor: ");
    Serial.println(sensorValue);

    if (sensorValue > threshold) {
        digitalWrite(ledPin, HIGH);
    } else {
        digitalWrite(ledPin, LOW);
    }
    delay(1000);
}
```

**MicroPython:**
```python
# Sensor reading
from machine import Pin, ADC
import time

sensor_pin = ADC(Pin(26))  # ADC0 on Pico
led_pin = Pin(25, Pin.OUT)
threshold = 32000  # 16-bit ADC value

while True:
    sensor_value = sensor_pin.read_u16()
    print(f"Sensor: {sensor_value}")

    if sensor_value > threshold:
        led_pin.on()
    else:
        led_pin.off()

    time.sleep(1)
```

Key changes:
- No `setup()` and `loop()` - just regular code
- No semicolons
- Indentation instead of braces
- `print()` instead of `Serial.print()`
- Variable names use `snake_case`

## Try It Yourself

**Exercise 1: Spot the Syntax Errors**

This MicroPython code has syntax errors from C++ habits. Find them:

```python
from machine import Pin;
import time;

led_pin = Pin(25, Pin.OUT);

while True
    led_pin.on();
    time.sleep(1);
    led_pin.off();
    time.sleep(1);
```

**Answer:**

Errors:
1. Semicolons after imports (remove them)
2. Semicolon after Pin assignment (remove it)
3. Missing colon after `while True`
4. Semicolons in the loop (remove all of them)

Corrected:
```python
from machine import Pin
import time

led_pin = Pin(25, Pin.OUT)

while True:
    led_pin.on()
    time.sleep(1)
    led_pin.off()
    time.sleep(1)
```

**Exercise 2: Convert Arduino to Python**

Convert this Arduino code to MicroPython:

```cpp
int buttonPin = 2;
int ledPin = 13;
bool buttonPressed = false;

void setup() {
    pinMode(buttonPin, INPUT);
    pinMode(ledPin, OUTPUT);
}

void loop() {
    buttonPressed = digitalRead(buttonPin);
    if (buttonPressed) {
        digitalWrite(ledPin, HIGH);
    }
}
```

**Answer:**

```python
from machine import Pin

button_pin = Pin(2, Pin.IN)
led_pin = Pin(13, Pin.OUT)

while True:
    button_pressed = button_pin.value()
    if button_pressed:
        led_pin.on()
```

**Exercise 3: Practice F-Strings**

Create a MicroPython program that reads a sensor value and prints:
- The raw value
- The value converted to voltage (assuming 3.3V reference, 16-bit ADC)
- A formatted message like "Sensor: 25000 (1.27V)"

Try using an f-string!

**Answer:**

```python
from machine import Pin, ADC
import time

sensor = ADC(Pin(26))

while True:
    value = sensor.read_u16()
    voltage = (value / 65535) * 3.3
    print(f"Sensor: {value} ({voltage:.2f}V)")
    time.sleep(1)
```

---
