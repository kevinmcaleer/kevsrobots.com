---
layout: lesson
title: Pin Control - Goodbye pinMode, Hello Pin Class!
author: Kevin McAleer
type: page
cover: /learn/arduino_to_python/assets/cover.jpg
date: 2025-11-20
previous: 04_functions.html
next: 06_analog_pwm.html
description: Learn how to control GPIO pins in MicroPython using the Pin class instead
  of Arduino's pinMode, digitalWrite, and digitalRead
percent: 48
duration: 9
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


## Digital Pin Control: A New Approach

Arduino uses separate functions for pin setup and control: `pinMode()`, `digitalWrite()`, and `digitalRead()`. MicroPython uses an object-oriented approach with the `Pin` class.

Instead of treating pins as numbers and calling global functions, you create Pin objects and call methods on them. It's cleaner and more Pythonic.

## Setting Up Pins

**Arduino C++:**
```cpp
const int LED_PIN = 13;
const int BUTTON_PIN = 2;

void setup() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT);
}
```

**MicroPython:**
```python
from machine import Pin

LED_PIN = 25
BUTTON_PIN = 14

led = Pin(LED_PIN, Pin.OUT)
button = Pin(BUTTON_PIN, Pin.IN)
```

Key differences:
- **Import the Pin class** from the `machine` module
- **Create Pin objects** instead of calling `pinMode()`
- **Use `Pin.OUT` and `Pin.IN`** instead of `OUTPUT` and `INPUT`
- **Store the Pin object** in a variable (like `led` or `button`)

## Pin Numbers

Pin numbers vary by board. Here's a quick reference:

**Arduino Nano ESP32:**
- D2-D13 (digital pins)
- LED_BUILTIN = pin 13
- Uses standard Arduino numbering

**Raspberry Pi Pico / Pico W / Pico 2W:**
- GP0-GP28 (GPIO pins)
- Built-in LED = pin 25 (Pico) or "LED" (Pico W/2W)
- Uses GP numbering (e.g., GP15 = pin 15)

**Example for built-in LED:**
```python
# Pico / Pico 2W
led = Pin(25, Pin.OUT)

# Pico W (WiFi LED is different)
led = Pin("LED", Pin.OUT)

# Arduino Nano ESP32
led = Pin(13, Pin.OUT)
```

## Writing to Digital Pins

**Arduino C++:**
```cpp
void loop() {
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    delay(1000);
}
```

**MicroPython:**
```python
import time

while True:
    led.on()        # or led.value(1)
    time.sleep(1)
    led.off()       # or led.value(0)
    time.sleep(1)
```

MicroPython gives you two ways to control pins:
- **`.on()` and `.off()`** - Clear and readable
- **`.value(1)` and `.value(0)`** - More explicit

Both do the same thing. Most code uses `.on()` and `.off()` for clarity.

## Reading from Digital Pins

**Arduino C++:**
```cpp
void loop() {
    int buttonState = digitalRead(BUTTON_PIN);

    if (buttonState == HIGH) {
        digitalWrite(LED_PIN, HIGH);
    } else {
        digitalWrite(LED_PIN, LOW);
    }
}
```

**MicroPython:**
```python
while True:
    button_state = button.value()

    if button_state == 1:
        led.on()
    else:
        led.off()
```

**`.value()`** reads the pin state:
- Returns `1` for HIGH
- Returns `0` for LOW

You can also use `.value()` to write by passing an argument: `.value(1)` or `.value(0)`.

## Pull-up and Pull-down Resistors

Buttons need pull-up or pull-down resistors. MicroPython makes this easy:

**Arduino C++:**
```cpp
void setup() {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    // or INPUT_PULLDOWN (if supported)
}
```

**MicroPython:**
```python
# Pull-up resistor (default state HIGH)
button = Pin(14, Pin.IN, Pin.PULL_UP)

# Pull-down resistor (default state LOW)
button = Pin(14, Pin.IN, Pin.PULL_DOWN)

# No resistor (floating - not recommended)
button = Pin(14, Pin.IN)
```

**When to use which:**
- **`Pin.PULL_UP`** - Button connects pin to GND. Pressed = LOW (0), Released = HIGH (1)
- **`Pin.PULL_DOWN`** - Button connects pin to 3.3V. Pressed = HIGH (1), Released = LOW (0)

Most buttons use pull-up configuration.

## Toggle Method

Here's a handy method Arduino doesn't have:

**MicroPython:**
```python
led = Pin(25, Pin.OUT)

# Toggle the LED state (on→off or off→on)
led.toggle()
```

This is perfect for simple blinking:

```python
import time

led = Pin(25, Pin.OUT)

while True:
    led.toggle()
    time.sleep(0.5)  # Blink every 500ms
```

## Getting Pin State

You can read back the current state of an output pin:

**MicroPython:**
```python
led = Pin(25, Pin.OUT)
led.on()

print(led.value())  # Prints: 1

led.off()
print(led.value())  # Prints: 0
```

This is useful for maintaining state in complex programs.

## Complete Button-LED Example

**Arduino C++:**
```cpp
const int LED_PIN = 13;
const int BUTTON_PIN = 2;

void setup() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void loop() {
    int buttonState = digitalRead(BUTTON_PIN);

    // Active LOW (pressed = LOW)
    if (buttonState == LOW) {
        digitalWrite(LED_PIN, HIGH);
    } else {
        digitalWrite(LED_PIN, LOW);
    }
}
```

**MicroPython:**
```python
from machine import Pin

led = Pin(25, Pin.OUT)
button = Pin(14, Pin.IN, Pin.PULL_UP)

while True:
    button_state = button.value()

    # Active LOW (pressed = 0)
    if button_state == 0:
        led.on()
    else:
        led.off()
```

## Button Debouncing

Mechanical buttons bounce - they send multiple signals when pressed. Here's a simple debounce pattern:

**Arduino C++:**
```cpp
const int BUTTON_PIN = 2;
const int DEBOUNCE_DELAY = 50;

int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;

void loop() {
    int reading = digitalRead(BUTTON_PIN);

    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
        // Button state is stable
        if (reading == LOW) {
            // Button pressed
        }
    }

    lastButtonState = reading;
}
```

**MicroPython:**
```python
from machine import Pin
import time

button = Pin(14, Pin.IN, Pin.PULL_UP)
DEBOUNCE_DELAY = 0.05  # 50ms

last_state = 1
last_time = 0

while True:
    current_state = button.value()

    if current_state != last_state:
        last_time = time.ticks_ms()

    if (time.ticks_ms() - last_time) > DEBOUNCE_DELAY * 1000:
        # Button state is stable
        if current_state == 0:
            # Button pressed
            print("Button pressed!")

    last_state = current_state
    time.sleep(0.01)  # Small delay
```

**Simpler approach** - just add a delay:

```python
from machine import Pin
import time

button = Pin(14, Pin.IN, Pin.PULL_UP)
led = Pin(25, Pin.OUT)

last_state = 1

while True:
    current_state = button.value()

    # Detect button press (transition from HIGH to LOW)
    if last_state == 1 and current_state == 0:
        led.toggle()
        time.sleep(0.2)  # Simple debounce

    last_state = current_state
    time.sleep(0.01)
```

## Multiple Pin Control

Working with multiple pins is cleaner in Python:

**Arduino C++:**
```cpp
int leds[] = {10, 11, 12, 13};
int numLeds = 4;

void setup() {
    for (int i = 0; i < numLeds; i++) {
        pinMode(leds[i], OUTPUT);
    }
}

void loop() {
    // Turn all on
    for (int i = 0; i < numLeds; i++) {
        digitalWrite(leds[i], HIGH);
    }
    delay(1000);

    // Turn all off
    for (int i = 0; i < numLeds; i++) {
        digitalWrite(leds[i], LOW);
    }
    delay(1000);
}
```

**MicroPython:**
```python
from machine import Pin
import time

# Create list of Pin objects
leds = [Pin(pin_num, Pin.OUT) for pin_num in [10, 11, 12, 13]]

while True:
    # Turn all on
    for led in leds:
        led.on()
    time.sleep(1)

    # Turn all off
    for led in leds:
        led.off()
    time.sleep(1)
```

That list comprehension `[Pin(pin_num, Pin.OUT) for pin_num in [10, 11, 12, 13]]` creates all Pin objects in one line!

## Running Light Pattern

**MicroPython:**
```python
from machine import Pin
import time

leds = [Pin(pin_num, Pin.OUT) for pin_num in [10, 11, 12, 13]]

while True:
    # Light up LEDs in sequence
    for led in leds:
        led.on()
        time.sleep(0.2)
        led.off()

    # Light up in reverse
    for led in reversed(leds):
        led.on()
        time.sleep(0.2)
        led.off()
```

## Pin Interrupts (Advanced)

Interrupts let you respond to pin changes instantly without polling. This is advanced but powerful:

**MicroPython:**
```python
from machine import Pin
import time

led = Pin(25, Pin.OUT)
button = Pin(14, Pin.IN, Pin.PULL_UP)

# Interrupt handler function
def button_pressed(pin):
    led.toggle()
    print("Button interrupt!")

# Set up interrupt on falling edge (HIGH→LOW)
button.irq(trigger=Pin.IRQ_FALLING, handler=button_pressed)

# Main loop can do other things
while True:
    print("Doing other work...")
    time.sleep(2)
```

**Interrupt triggers:**
- `Pin.IRQ_FALLING` - HIGH to LOW transition
- `Pin.IRQ_RISING` - LOW to HIGH transition
- `Pin.IRQ_FALLING | Pin.IRQ_RISING` - Both transitions

**Important:** Interrupt handlers should be short and fast. Don't use delays or print statements in production interrupt handlers.

## Try It Yourself

**Exercise 1: Traffic Light**

Create a traffic light with 3 LEDs (red, yellow, green) on pins 10, 11, 12:
- Red on for 5 seconds
- Green on for 5 seconds
- Yellow on for 2 seconds
- Repeat

**Answer:**

```python
from machine import Pin
import time

red = Pin(10, Pin.OUT)
yellow = Pin(11, Pin.OUT)
green = Pin(12, Pin.OUT)

def all_off():
    red.off()
    yellow.off()
    green.off()

while True:
    all_off()
    red.on()
    time.sleep(5)

    all_off()
    green.on()
    time.sleep(5)

    all_off()
    yellow.on()
    time.sleep(2)
```

**Exercise 2: Button Counter**

Create a program that:
- Counts button presses
- Displays the count on 4 LEDs in binary (pins 10-13)
- Button on pin 14 with pull-up
- Max count = 15 (4 bits), then reset to 0

**Answer:**

```python
from machine import Pin
import time

button = Pin(14, Pin.IN, Pin.PULL_UP)
leds = [Pin(pin_num, Pin.OUT) for pin_num in [10, 11, 12, 13]]

count = 0
last_state = 1

def display_binary(number):
    """Display a number on 4 LEDs in binary"""
    for i in range(4):
        bit = (number >> i) & 1  # Extract bit i
        leds[i].value(bit)

while True:
    current_state = button.value()

    # Detect button press (HIGH to LOW)
    if last_state == 1 and current_state == 0:
        count = (count + 1) % 16  # Wrap at 16
        display_binary(count)
        print(f"Count: {count} (Binary: {count:04b})")
        time.sleep(0.2)  # Debounce

    last_state = current_state
    time.sleep(0.01)
```

**Exercise 3: Button-Controlled Patterns**

Create a program with:
- 4 LEDs on pins 10-13
- 1 button on pin 14 with pull-up
- Each button press cycles through patterns:
  1. All LEDs on
  2. All LEDs off
  3. Alternating pattern (10, 12 on)
  4. Other alternating pattern (11, 13 on)
  5. Running light
- Hold the pattern until next button press

**Answer:**

```python
from machine import Pin
import time

button = Pin(14, Pin.IN, Pin.PULL_UP)
leds = [Pin(pin_num, Pin.OUT) for pin_num in [10, 11, 12, 13]]

pattern = 0
last_state = 1

def set_pattern(pattern_num):
    if pattern_num == 0:  # All on
        for led in leds:
            led.on()
    elif pattern_num == 1:  # All off
        for led in leds:
            led.off()
    elif pattern_num == 2:  # Alternating 1
        leds[0].on()
        leds[1].off()
        leds[2].on()
        leds[3].off()
    elif pattern_num == 3:  # Alternating 2
        leds[0].off()
        leds[1].on()
        leds[2].off()
        leds[3].on()

while True:
    current_state = button.value()

    # Detect button press
    if last_state == 1 and current_state == 0:
        pattern = (pattern + 1) % 4
        set_pattern(pattern)
        print(f"Pattern: {pattern}")
        time.sleep(0.2)

    last_state = current_state
    time.sleep(0.01)
```
