---
layout: lesson
title: Control Flow - if, else, and Loops
author: Kevin McAleer
type: page
cover: /learn/arduino_to_python/assets/cover.jpg
date: 2025-01-20
previous: 04_variables_and_types.html
next: 06_functions.html
description: Master Python's indentation-based syntax and learn how if/elif/else and
  loops differ from Arduino C++. Critical syntax differences explained.
percent: 18
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


![Flowchart diagram showing conditional logic flow](assets/control_flow.jpg){:class="cover"}

## The Biggest Visual Difference

If you've only seen Python code in passing, you probably noticed the missing curly braces `{}`. Python uses **indentation** to define code blocks. This isn't just a style choice - it's a syntax requirement.

This takes some getting used to, but most developers find it cleaner after a few days.

---

## If Statements - Side by Side

**Arduino C++:**
```cpp
int sensorValue = analogRead(A0);

if (sensorValue > 512) {
  digitalWrite(LED_PIN, HIGH);
  Serial.println("Sensor HIGH");
}
```

**Python:**
```python
sensor_value = adc.read_u16()

if sensor_value > 32768:
    led.on()
    print("Sensor HIGH")
```

**Key differences:**
- No parentheses required around condition (but allowed)
- **Colon `:` after condition** (required!)
- **Indentation defines the block** (no curly braces)
- 4 spaces is Python standard (not tabs!)

> ## C++ Developers: Critical!
>
> Indentation is **not optional** in Python. Incorrect indentation causes `IndentationError`. Use 4 spaces (most IDEs auto-convert tabs to spaces).

---

## If-Else - The Basics

**Arduino C++:**
```cpp
if (buttonState == LOW) {
  digitalWrite(LED_PIN, HIGH);
} else {
  digitalWrite(LED_PIN, LOW);
}
```

**Python:**
```python
if button.value() == 0:
    led.on()
else:
    led.off()
```

**What changed:**
- Colon `:` after both `if` and `else`
- Indentation for both blocks
- Same logic, cleaner syntax

---

## If-Else-If Chains: elif vs else if

**Arduino C++:**
```cpp
int speed = getSpeed();

if (speed > 200) {
  Serial.println("Fast");
} else if (speed > 100) {
  Serial.println("Medium");
} else if (speed > 0) {
  Serial.println("Slow");
} else {
  Serial.println("Stopped");
}
```

**Python:**
```python
speed = get_speed()

if speed > 200:
    print("Fast")
elif speed > 100:
    print("Medium")
elif speed > 0:
    print("Slow")
else:
    print("Stopped")
```

**Key difference:** Python uses `elif` (not `else if`). It's shorter and more readable.

---

## Real-World Robot Example: Obstacle Detection

Let's implement a three-sensor obstacle avoidance system.

**Arduino C++:**
```cpp
// Ultrasonic sensors on left, center, right
const int DISTANCE_THRESHOLD = 20;  // cm

int leftDist = getDistance(LEFT_SENSOR);
int centerDist = getDistance(CENTER_SENSOR);
int rightDist = getDistance(RIGHT_SENSOR);

if (centerDist < DISTANCE_THRESHOLD) {
  // Obstacle ahead - decide which way to turn
  if (leftDist > rightDist) {
    turnLeft();
    Serial.println("Obstacle ahead, turning left");
  } else {
    turnRight();
    Serial.println("Obstacle ahead, turning right");
  }
} else if (leftDist < DISTANCE_THRESHOLD) {
  turnRight();
  Serial.println("Obstacle on left, turning right");
} else if (rightDist < DISTANCE_THRESHOLD) {
  turnLeft();
  Serial.println("Obstacle on right, turning left");
} else {
  moveForward();
  Serial.println("Path clear, moving forward");
}
```

**Python:**
```python
# Ultrasonic sensors on left, center, right
DISTANCE_THRESHOLD = 20  # cm

left_dist = get_distance(left_sensor)
center_dist = get_distance(center_sensor)
right_dist = get_distance(right_sensor)

if center_dist < DISTANCE_THRESHOLD:
    # Obstacle ahead - decide which way to turn
    if left_dist > right_dist:
        turn_left()
        print("Obstacle ahead, turning left")
    else:
        turn_right()
        print("Obstacle ahead, turning right")
elif left_dist < DISTANCE_THRESHOLD:
    turn_right()
    print("Obstacle on left, turning right")
elif right_dist < DISTANCE_THRESHOLD:
    turn_left()
    print("Obstacle on right, turning left")
else:
    move_forward()
    print("Path clear, moving forward")
```

**What stayed the same:**
- Logic structure is identical
- Nested if statements work the same way
- Comparison operators unchanged

**What changed:**
- `elif` instead of `else if`
- Colons after each condition
- Indentation instead of braces
- Snake_case naming convention

---

## Comparison Operators - Mostly the Same

Good news: Most comparison operators are identical!

| Operator | Meaning | C++ | Python |
|----------|---------|-----|--------|
| `==` | Equal to | Yes | Yes |
| `!=` | Not equal to | Yes | Yes |
| `<` | Less than | Yes | Yes |
| `>` | Greater than | Yes | Yes |
| `<=` | Less than or equal | Yes | Yes |
| `>=` | Greater than or equal | Yes | Yes |
| `&&` | Logical AND | Yes | **No - use `and`** |
| `||` | Logical OR | Yes | **No - use `or`** |
| `!` | Logical NOT | Yes | **No - use `not`** |

**Arduino C++:**
```cpp
if (speed > 100 && speed < 200) {
  Serial.println("Medium speed");
}

if (sensorA == HIGH || sensorB == HIGH) {
  digitalWrite(LED_PIN, HIGH);
}

if (!motorRunning) {
  startMotor();
}
```

**Python:**
```python
if speed > 100 and speed < 200:
    print("Medium speed")

if sensor_a.value() or sensor_b.value():
    led.on()

if not motor_running:
    start_motor()
```

**Python bonus - range checking:**
```python
# Python allows elegant range checks
if 100 < speed < 200:
    print("Medium speed")

# Equivalent to:
if speed > 100 and speed < 200:
    print("Medium speed")
```

> ## Pro Tip
>
> Python's chained comparisons (`100 < speed < 200`) are more readable than C++'s compound conditions. Use them!
{:.bg-blue}

---

## While Loops - Nearly Identical

**Arduino C++:**
```cpp
void loop() {
  // Arduino's loop() is essentially while(true)
  // Your code here
}

// Explicit while loop
int count = 0;
while (count < 10) {
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  delay(100);
  count++;
}
```

**Python:**
```python
# MicroPython's equivalent to Arduino's loop()
while True:
    # Your code here
    pass

# Explicit while loop
count = 0
while count < 10:
    led.on()
    time.sleep(0.1)
    led.off()
    time.sleep(0.1)
    count += 1
```

**Key differences:**
- `True` is capitalized in Python (not `true`)
- Colon after condition
- Use `pass` if you need an empty block (Python doesn't allow empty blocks)
- `count++` doesn't exist - use `count += 1`

---

## For Loops - Completely Different!

This is where Python really diverges from C++. Python for loops are **iterator-based**, not counter-based.

**Arduino C++ (counter-based):**
```cpp
// Classic C-style for loop
for (int i = 0; i < 5; i++) {
  digitalWrite(ledPins[i], HIGH);
  delay(200);
  digitalWrite(ledPins[i], LOW);
}

// Iterating array
int pins[] = {2, 3, 4, 5, 6};
for (int i = 0; i < 5; i++) {
  pinMode(pins[i], OUTPUT);
}
```

**Python (iterator-based):**
```python
# Python's for loop iterates over sequences
led_pins = [2, 3, 4, 5, 6]

for pin_num in led_pins:
    led = Pin(pin_num, Pin.OUT)
    led.on()
    time.sleep(0.2)
    led.off()

# Need a counter? Use range()
for i in range(5):  # 0, 1, 2, 3, 4
    print(f"Count: {i}")

# Range with start and stop
for i in range(10, 15):  # 10, 11, 12, 13, 14
    print(f"Count: {i}")

# Range with step
for i in range(0, 10, 2):  # 0, 2, 4, 6, 8
    print(f"Even number: {i}")
```

**Key differences:**
- No `int i = 0; i < 5; i++` syntax in Python
- `range(5)` generates 0, 1, 2, 3, 4 (not 1-5!)
- `for item in collection:` directly iterates items
- Much cleaner when working with lists

---

## Real-World Example: LED Chase Pattern

**Arduino C++:**
```cpp
const int NUM_LEDS = 5;
int ledPins[NUM_LEDS] = {2, 3, 4, 5, 6};

void setup() {
  for (int i = 0; i < NUM_LEDS; i++) {
    pinMode(ledPins[i], OUTPUT);
  }
}

void loop() {
  // Forward chase
  for (int i = 0; i < NUM_LEDS; i++) {
    digitalWrite(ledPins[i], HIGH);
    delay(100);
    digitalWrite(ledPins[i], LOW);
  }

  // Backward chase
  for (int i = NUM_LEDS - 1; i >= 0; i--) {
    digitalWrite(ledPins[i], HIGH);
    delay(100);
    digitalWrite(ledPins[i], LOW);
  }
}
```

**Python:**
```python
from machine import Pin
import time

# Create list of LED Pin objects
led_pins = [Pin(num, Pin.OUT) for num in [2, 3, 4, 5, 6]]

while True:
    # Forward chase
    for led in led_pins:
        led.on()
        time.sleep(0.1)
        led.off()

    # Backward chase
    for led in reversed(led_pins):
        led.on()
        time.sleep(0.1)
        led.off()
```

**Python advantages:**
- No need to track array length
- `reversed()` is cleaner than `i--` countdown
- List comprehension creates all Pin objects in one line
- Direct iteration over LEDs, not indices

---

## Break and Continue - Same Concept

**Arduino C++:**
```cpp
while (true) {
  int sensor = analogRead(A0);

  if (sensor < 100) {
    continue;  // Skip rest of loop
  }

  if (sensor > 900) {
    break;  // Exit loop
  }

  processData(sensor);
}
```

**Python:**
```python
while True:
    sensor = adc.read_u16()

    if sensor < 6554:  # ~10%
        continue  # Skip rest of loop

    if sensor > 58982:  # ~90%
        break  # Exit loop

    process_data(sensor)
```

**Same in both languages:** `break` exits loop, `continue` skips to next iteration.

---

## Switch/Case vs If/Elif

**Arduino C++ has switch/case:**
```cpp
switch (motorState) {
  case 0:
    motorStop();
    break;
  case 1:
    motorForward();
    break;
  case 2:
    motorReverse();
    break;
  default:
    motorStop();
}
```

**Python traditionally uses if/elif:**
```python
if motor_state == 0:
    motor_stop()
elif motor_state == 1:
    motor_forward()
elif motor_state == 2:
    motor_reverse()
else:
    motor_stop()
```

**Python 3.10+ has match/case:**
```python
match motor_state:
    case 0:
        motor_stop()
    case 1:
        motor_forward()
    case 2:
        motor_reverse()
    case _:  # default
        motor_stop()
```

> ## Note
>
> `match/case` requires Python 3.10+. MicroPython support varies by board. Use `if/elif` for maximum compatibility.

---

## No Do-While in Python

**Arduino C++:**
```cpp
int count = 0;
do {
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);
  count++;
} while (count < 5);
```

**Python equivalent:**
```python
count = 0
while True:
    led.on()
    time.sleep(0.5)
    led.off()
    time.sleep(0.5)
    count += 1

    if count >= 5:
        break
```

**Workaround:** Python has no `do-while`. Use `while True` with `break` condition at the end.

---

## Try It Yourself

**Exercise 1: Traffic Light Controller**
```python
from machine import Pin
import time

red = Pin(2, Pin.OUT)
yellow = Pin(3, Pin.OUT)
green = Pin(4, Pin.OUT)

# TODO: Implement traffic light sequence:
# 1. Green for 5 seconds
# 2. Yellow for 2 seconds
# 3. Red for 5 seconds
# 4. Repeat
# Use if/elif to track state or a simple loop

# Your code here
```

**Exercise 2: Button Debounce with State Machine**
```python
from machine import Pin
import time

button = Pin(14, Pin.IN, Pin.PULL_UP)
led = Pin(13, Pin.OUT)

# TODO: Implement toggle on button press
# - Wait for button press (value goes to 0)
# - Toggle LED
# - Wait for button release
# - Repeat
# Hint: Use while loops and if statements to track button state

# Your code here
```

**Exercise 3: Sensor Range Classifier**
```python
from machine import ADC, Pin

adc = ADC(Pin(26))

# TODO: Read analog sensor and classify:
# - 0-20% = "Very Low"
# - 20-40% = "Low"
# - 40-60% = "Medium"
# - 60-80% = "High"
# - 80-100% = "Very High"
# Use if/elif/else chain or try match/case if available

while True:
    value = adc.read_u16()
    # Your code here
    time.sleep(1)
```

**Exercise 4: LED Pattern Generator**
```python
from machine import Pin
import time

leds = [Pin(i, Pin.OUT) for i in range(2, 7)]

# TODO: Create different patterns:
# Pattern 1: All on, all off, repeat
# Pattern 2: Chase forward, chase backward
# Pattern 3: Alternate even/odd LEDs
# Pattern 4: Your own creative pattern!

# Use for loops and appropriate delays

# Your code here
```

---

## Common Issues

**Problem: IndentationError**
```python
if sensor_value > 100:
print("High")  # ERROR! Not indented
```
**Solution:** Indent with 4 spaces after colon.
```python
if sensor_value > 100:
    print("High")  # Correct!
```
**Why:** Python uses indentation to define blocks. This isn't optional.

---

**Problem: Using && instead of and**
```python
if speed > 100 && speed < 200:  # SyntaxError!
```
**Solution:** Use `and`, `or`, `not` (words, not symbols).
```python
if speed > 100 and speed < 200:  # Correct!
```
**Why:** Python uses English words for logical operators.

---

**Problem: Forgetting colons**
```python
if sensor_value > 100  # SyntaxError: expected ':'
    print("High")
```
**Solution:** Always add colon after if/elif/else/while/for.
```python
if sensor_value > 100:
    print("High")
```
**Why:** Colon indicates start of indented block in Python.

---

**Problem: Using true instead of True**
```python
while true:  # NameError: name 'true' is not defined
    pass
```
**Solution:** Capitalize boolean values.
```python
while True:  # Correct!
    pass
```
**Why:** Python uses `True` and `False` (capitalized), not `true`/`false`.

---

**Problem: Range off-by-one**
```cpp
// C++ - this runs 5 times: i = 1, 2, 3, 4, 5
for (int i = 1; i <= 5; i++) {
```
```python
# Python - this runs 4 times: i = 1, 2, 3, 4 (not 5!)
for i in range(1, 5):
```
**Solution:** Remember range(1, 5) means 1 to 4. Use range(1, 6) for 1 to 5.
```python
for i in range(1, 6):  # 1, 2, 3, 4, 5
```
**Why:** Python's range() is "up to but not including" the end value.

---

## Summary: Control Flow Comparison

| Feature | Arduino C++ | Python |
|---------|-------------|--------|
| **If statement** | `if (x > 5) { }` | `if x > 5:` |
| **Else if** | `else if` | `elif` |
| **Else** | `else { }` | `else:` |
| **Logical AND** | `&&` | `and` |
| **Logical OR** | `||` | `or` |
| **Logical NOT** | `!` | `not` |
| **While loop** | `while (x < 10) { }` | `while x < 10:` |
| **For loop** | `for (int i=0; i<5; i++)` | `for i in range(5):` |
| **For each** | C++11: `for (auto x : arr)` | `for x in list:` |
| **Switch/case** | `switch/case/break` | `if/elif` or `match/case` |
| **Do-while** | `do { } while(x);` | `while True:` + `break` |
| **Break** | `break;` | `break` |
| **Continue** | `continue;` | `continue` |
| **Code blocks** | `{ }` curly braces | Indentation |
| **True/False** | `true`, `false` | `True`, `False` |

**Key takeaways:**
- **Indentation is syntax** in Python - use 4 spaces consistently
- **Colons `:` required** after if/elif/else/while/for
- Use **`and`, `or`, `not`** instead of `&&`, `||`, `!`
- **`elif`** not `else if`
- **`range()`** for numeric loops, **`for x in list:`** for iteration
- **`True`/`False` capitalized**
- No `do-while`, no `switch/case` (pre-3.10), no `i++`

The logic is the same - only the syntax changed!

---

> **Next Lesson**: [Functions](/learn/arduino_to_python/06_functions.html) - From setup()/loop() to Python functions and beyond
>
> **Previous Lesson**: [Variables and Data Types](/learn/arduino_to_python/04_variables_and_types.html)

---
