---
layout: lesson
title: Interrupts and Event Handling
author: Kevin McAleer
type: page
cover: /learn/arduino_to_python/assets/interrupts.jpg
date: 2025-01-20
previous: 12_timing_and_delays.html
next: 14_memory_management.html
description: Comparing attachInterrupt() with Pin.irq() for responsive hardware control
percent: 42
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


![Interrupts cover image](assets/interrupts.jpg){:class="cover"}

## Introduction

Interrupts are the secret to creating responsive embedded systems. Instead of constantly checking if a button was pressed or a sensor triggered (polling), interrupts let the hardware notify your code immediately when an event occurs.

In Arduino, you use `attachInterrupt()` to set up interrupt handlers. MicroPython provides `Pin.irq()` - a more object-oriented approach with similar capabilities.

In this lesson, you'll learn how interrupts work in both systems, understand the strict rules for interrupt service routines (ISRs), and discover how to use interrupts for button handling, rotary encoders, and emergency stops.

## What Are Interrupts?

Think of interrupts like a doorbell:

**Without interrupts (polling):**
```
while (true) {
  if (door_has_visitor()) {  // Check every loop iteration
    answer_door()
  }
  do_other_work()
}
```

**With interrupts:**
```
on_doorbell_ring() {  // Called automatically when doorbell rings
  answer_door()
}

while (true) {
  do_other_work()  // No need to check door!
}
```

Interrupts are more efficient and have near-zero latency - the code runs immediately when the event occurs.

## Simple Button Interrupt

### Arduino C++ (attachInterrupt)
```cpp
const int BUTTON_PIN = 2;  // Must be interrupt-capable pin
const int LED_PIN = 13;

volatile bool ledState = false;

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  // Attach interrupt (pin, function, mode)
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);
}

void loop() {
  // Main code can do other things
  digitalWrite(LED_PIN, ledState);
  delay(10);
}

// Interrupt Service Routine (ISR)
void buttonISR() {
  ledState = !ledState;  // Toggle LED state
}
```

**Key elements:**
- `volatile` keyword for shared variables
- `digitalPinToInterrupt()` to convert pin to interrupt number
- ISR function: `void buttonISR()`
- Trigger modes: `RISING`, `FALLING`, `CHANGE`

### MicroPython (Pin.irq)
```python
from machine import Pin
import time

LED_PIN = 25
BUTTON_PIN = 15

led = Pin(LED_PIN, Pin.OUT)
button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_UP)

led_state = False

def button_isr(pin):
    """Interrupt Service Routine"""
    global led_state
    led_state = not led_state

# Attach interrupt
button.irq(trigger=Pin.IRQ_FALLING, handler=button_isr)

# Main loop
while True:
    led.value(led_state)
    time.sleep(0.01)
```

**Key elements:**
- ISR takes `pin` parameter
- `global` keyword for shared variables
- `.irq()` method on Pin object
- Trigger modes: `Pin.IRQ_FALLING`, `Pin.IRQ_RISING`, `Pin.IRQ_RISING | Pin.IRQ_FALLING`

## Interrupt Trigger Modes

### Arduino C++
```cpp
// Trigger on rising edge (LOW → HIGH)
attachInterrupt(intNum, ISR, RISING);

// Trigger on falling edge (HIGH → LOW)
attachInterrupt(intNum, ISR, FALLING);

// Trigger on any change
attachInterrupt(intNum, ISR, CHANGE);

// Trigger when LOW (continuous, careful!)
attachInterrupt(intNum, ISR, LOW);
```

### MicroPython
```python
# Trigger on rising edge (0 → 1)
pin.irq(trigger=Pin.IRQ_RISING, handler=isr)

# Trigger on falling edge (1 → 0)
pin.irq(trigger=Pin.IRQ_FALLING, handler=isr)

# Trigger on both edges
pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=isr)

# Note: No continuous LOW trigger in MicroPython
```

## ISR Rules: What You Can and Can't Do

Interrupt Service Routines have strict rules. Violating them causes crashes!

### Arduino C++ ISR Rules
```cpp
volatile int counter = 0;

// Good ISR - short and simple
void goodISR() {
  counter++;  // OK: Simple variable update
}

// Bad ISR - violates rules!
void badISR() {
  Serial.println("Button pressed!");  // NO! Serial in ISR
  delay(100);                          // NO! Delay in ISR
  float result = sqrt(counter);        // NO! Complex math
  analogWrite(9, 128);                 // NO! analogWrite uses interrupts
}
```

**ISR Rules:**
- ✅ Keep it short (microseconds, not milliseconds)
- ✅ Use `volatile` for shared variables
- ✅ Simple operations only
- ❌ No `delay()` or `delayMicroseconds()`
- ❌ No `Serial.print()`
- ❌ No `analogWrite()` (it uses timers)
- ❌ No complex calculations
- ❌ No function calls that might block

### MicroPython ISR Rules
```python
counter = 0

# Good ISR - short and simple
def good_isr(pin):
    global counter
    counter += 1  # OK: Simple variable update

# Bad ISR - violates rules!
def bad_isr(pin):
    print("Button pressed!")  # NO! print() in ISR
    time.sleep(0.1)            # NO! sleep() in ISR
    result = complex_calc()     # NO! Complex function
    sensor.read()              # NO! Hardware access might block
```

**Same rules apply:**
- ✅ Keep it extremely short
- ✅ Use `global` for shared variables
- ✅ Set flags only
- ❌ No `print()` or `time.sleep()`
- ❌ No hardware access (ADC, I2C, SPI)
- ❌ No complex operations
- ❌ No memory allocation

## The Flag Pattern: Best Practice

The safest ISR pattern: set a flag, handle it in main loop.

### Arduino C++
```cpp
volatile bool buttonPressed = false;

void setup() {
  pinMode(2, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(2), buttonISR, FALLING);
}

void loop() {
  // Check flag in main loop
  if (buttonPressed) {
    buttonPressed = false;  // Clear flag

    // Now we can do complex operations safely
    Serial.println("Button pressed!");
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);
  }

  // Other main loop code...
}

// ISR just sets flag - super fast
void buttonISR() {
  buttonPressed = true;
}
```

### MicroPython (Same Pattern)
```python
from machine import Pin
import time

button_pressed = False

def button_isr(pin):
    """ISR just sets flag"""
    global button_pressed
    button_pressed = True

# Setup
button = Pin(15, Pin.IN, Pin.PULL_UP)
led = Pin(25, Pin.OUT)
button.irq(trigger=Pin.IRQ_FALLING, handler=button_isr)

# Main loop handles flag
while True:
    if button_pressed:
        button_pressed = False  # Clear flag

        # Complex operations safe here
        print("Button pressed!")
        led.on()
        time.sleep(0.1)
        led.off()

    # Other main loop code...
    time.sleep(0.01)
```

## Debouncing Interrupts

Mechanical buttons bounce - one press can trigger multiple interrupts!

### Arduino C++ (Debounce with Timing)
```cpp
volatile unsigned long lastInterruptTime = 0;
const unsigned long DEBOUNCE_TIME = 200;  // ms

void buttonISR() {
  unsigned long interruptTime = millis();

  // Ignore if within debounce window
  if (interruptTime - lastInterruptTime > DEBOUNCE_TIME) {
    // Valid press
    buttonPressed = true;
  }

  lastInterruptTime = interruptTime;
}
```

### MicroPython (Same Debounce Logic)
```python
import time

last_interrupt_time = 0
DEBOUNCE_TIME = 200  # ms

def button_isr(pin):
    global last_interrupt_time, button_pressed

    interrupt_time = time.ticks_ms()

    # Ignore if within debounce window
    if time.ticks_diff(interrupt_time, last_interrupt_time) > DEBOUNCE_TIME:
        # Valid press
        button_pressed = True

    last_interrupt_time = interrupt_time
```

## Real-World Example: Rotary Encoder

Rotary encoders generate two signals (A and B) that must be read together.

### Arduino C++ (Encoder with Interrupts)
```cpp
const int ENCODER_A = 2;
const int ENCODER_B = 3;

volatile int encoderPos = 0;
volatile byte lastA = 1;
volatile byte lastB = 1;

void setup() {
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderISR, CHANGE);
}

void loop() {
  Serial.println(encoderPos);
  delay(100);
}

void encoderISR() {
  byte a = digitalRead(ENCODER_A);
  byte b = digitalRead(ENCODER_B);

  // Determine direction based on state changes
  if (a != lastA) {
    if (a == b) {
      encoderPos++;  // Clockwise
    } else {
      encoderPos--;  // Counter-clockwise
    }
  }

  lastA = a;
  lastB = b;
}
```

### MicroPython (Same Encoder Logic)
```python
from machine import Pin
import time

ENCODER_A = 14
ENCODER_B = 15

encoder_pos = 0
last_a = 1
last_b = 1

def encoder_isr(pin):
    global encoder_pos, last_a, last_b

    a = pin_a.value()
    b = pin_b.value()

    # Determine direction
    if a != last_a:
        if a == b:
            encoder_pos += 1  # Clockwise
        else:
            encoder_pos -= 1  # Counter-clockwise

    last_a = a
    last_b = b

# Setup
pin_a = Pin(ENCODER_A, Pin.IN, Pin.PULL_UP)
pin_b = Pin(ENCODER_B, Pin.IN, Pin.PULL_UP)

pin_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=encoder_isr)
pin_b.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=encoder_isr)

# Main loop
while True:
    print(encoder_pos)
    time.sleep(0.1)
```

## Emergency Stop Button

Interrupts are perfect for safety features that need instant response.

### Arduino C++
```cpp
const int ESTOP_PIN = 2;
volatile bool emergencyStop = false;

void setup() {
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), eStopISR, FALLING);
}

void loop() {
  if (emergencyStop) {
    // Stop all motors immediately
    stopAllMotors();

    // Wait for reset button
    while (emergencyStop) {
      // Blink LED to show emergency state
      blinkEmergencyLED();
    }
  }

  // Normal robot operation
  normalOperation();
}

void eStopISR() {
  emergencyStop = true;
}
```

### MicroPython
```python
from machine import Pin
import time

ESTOP_PIN = 14
emergency_stop = False

def estop_isr(pin):
    global emergency_stop
    emergency_stop = True

# Setup
estop_button = Pin(ESTOP_PIN, Pin.IN, Pin.PULL_UP)
estop_button.irq(trigger=Pin.IRQ_FALLING, handler=estop_isr)

while True:
    if emergency_stop:
        # Stop all motors immediately
        stop_all_motors()

        # Wait for reset
        while emergency_stop:
            # Blink LED to show emergency state
            blink_emergency_led()

    # Normal robot operation
    normal_operation()
```

## Disabling and Re-enabling Interrupts

### Arduino C++
```cpp
void loop() {
  // Disable all interrupts
  noInterrupts();

  // Critical section - no interrupts can occur
  criticalOperation();

  // Re-enable interrupts
  interrupts();

  // Or disable specific interrupt
  detachInterrupt(digitalPinToInterrupt(2));

  // Re-attach later
  attachInterrupt(digitalPinToInterrupt(2), buttonISR, FALLING);
}
```

### MicroPython
```python
# Disable specific interrupt
button.irq(handler=None)

# Re-enable later
button.irq(trigger=Pin.IRQ_FALLING, handler=button_isr)

# Note: No global interrupt disable in MicroPython
# (platform-dependent, use with caution)
```

## Try It Yourself

### Exercise 1: Button Counter
Count button presses with interrupt and display count:
- Use interrupt for button
- Implement proper debouncing
- Display count on serial monitor
- Reset count after 10 presses

### Exercise 2: Frequency Counter
Measure the frequency of an incoming signal using interrupts:
```python
# Count pulses in 1 second window
# Calculate frequency in Hz
# Display on screen
```

### Exercise 3: Multi-Button State Machine
Implement a state machine with three buttons:
- Button 1: Next state
- Button 2: Previous state
- Button 3: Reset to state 0
- Each state lights a different LED

### Exercise 4: Wheel Speed Sensor
Simulate a robot wheel with encoder (use button as encoder pulses):
- Count pulses per second
- Calculate RPM
- Display speed
- Detect if wheel stops (no pulses for 2 seconds)

## Common Issues

### Issue: Interrupt Not Firing
**Problem:** ISR never gets called.

**Solution:**
```python
# Check pin capabilities - not all pins support interrupts
# On Pico: Most GPIO pins support interrupts
# On ESP32: All GPIO pins support interrupts

# Check trigger mode
button.irq(trigger=Pin.IRQ_FALLING, handler=button_isr)  # Correct
# button.irq(handler=button_isr)  # Wrong! No trigger specified
```

### Issue: Crashes in ISR
**Problem:** Program hangs or crashes when interrupt fires.

**Solution:** Remove complex operations from ISR:
```python
# Bad - crashes
def bad_isr(pin):
    print(f"Value: {sensor.read()}")  # NO!
    time.sleep(0.1)                    # NO!

# Good - sets flag only
def good_isr(pin):
    global flag
    flag = True
```

### Issue: Variable Not Updating
**Problem:** Variable changed in ISR doesn't update in main loop.

**Arduino C++ Solution:**
```cpp
volatile bool flag = false;  // Must use volatile!
```

**Python Solution:**
```python
# Use global keyword
def isr(pin):
    global flag
    flag = True
```

### Issue: Bouncing Causes Multiple Triggers
**Solution:** Implement debouncing with timing check in ISR.

## Summary

| Feature | Arduino C++ | MicroPython |
|---------|-------------|-------------|
| **Attach interrupt** | `attachInterrupt(intNum, ISR, mode)` | `pin.irq(trigger=mode, handler=ISR)` |
| **Detach interrupt** | `detachInterrupt(intNum)` | `pin.irq(handler=None)` |
| **Rising edge** | `RISING` | `Pin.IRQ_RISING` |
| **Falling edge** | `FALLING` | `Pin.IRQ_FALLING` |
| **Both edges** | `CHANGE` | `Pin.IRQ_RISING \| Pin.IRQ_FALLING` |
| **ISR parameter** | None | `pin` (the pin that triggered) |
| **Shared variables** | `volatile` keyword | `global` keyword |
| **Global disable** | `noInterrupts()` / `interrupts()` | Platform-specific |

**ISR Golden Rules:**
1. ✅ Keep it short (< 10 microseconds ideal)
2. ✅ Set flags, handle in main loop
3. ✅ Use volatile/global for shared variables
4. ❌ No print/Serial
5. ❌ No delays
6. ❌ No hardware access
7. ❌ No complex logic

**When to Use Interrupts:**
- Button presses
- Rotary encoders
- Emergency stops
- External sensors with event pins
- Wheel encoders
- Frequency measurement

**When NOT to Use Interrupts:**
- Can be handled in main loop adequately
- Event frequency > 1 kHz (might overwhelm CPU)
- Complex processing needed
- Simple polling works fine

In the final lesson of this section, we'll explore memory management - how Python's automatic garbage collection compares to C++'s manual memory management.

---

> **Previous Lesson:** [Timing and Delays](/learn/arduino_to_python/12_timing_and_delays.html) |
> **Next Lesson:** [Memory Management](/learn/arduino_to_python/14_memory_management.html)
>
> **Course Home:** [Arduino to Python](/learn/arduino_to_python/)
