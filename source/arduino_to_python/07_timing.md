---
title: Timing and Delays - Seconds vs Milliseconds!
description: Learn the critical differences between Arduino timing functions and MicroPython's time module - avoid the milliseconds trap!
layout: lesson
type: page
cover: /learn/arduino_to_python/assets/cover.jpg
---

## Timing: The Most Important Difference

Here's the **single biggest gotcha** when moving from Arduino to MicroPython:

**Arduino uses milliseconds. MicroPython uses seconds.**

This one difference causes more bugs than anything else. Let's make sure you never fall into this trap.

## Delays: Milliseconds vs Seconds

**Arduino C++:**
```cpp
void loop() {
    digitalWrite(LED_PIN, HIGH);
    delay(1000);  // Wait 1000 milliseconds = 1 second
    digitalWrite(LED_PIN, LOW);
    delay(500);   // Wait 500 milliseconds = 0.5 seconds
}
```

**MicroPython:**
```python
import time

while True:
    led.on()
    time.sleep(1)      # Wait 1 second (NOT 1 millisecond!)
    led.off()
    time.sleep(0.5)    # Wait 0.5 seconds
```

**Critical differences:**
- **`delay(1000)`** in Arduino = **`time.sleep(1)`** in MicroPython
- **`delay(500)`** in Arduino = **`time.sleep(0.5)`** in MicroPython
- Arduino counts in milliseconds (1000 = 1 second)
- Python counts in seconds (1 = 1 second)

**Common mistake:**
```python
# WRONG! This waits 1000 SECONDS (16+ minutes!)
time.sleep(1000)

# RIGHT! This waits 1 second
time.sleep(1)
```

## Conversion Reference

Quick conversion table:

| Arduino (ms) | MicroPython (s) | Duration |
|--------------|-----------------|----------|
| `delay(1)` | `time.sleep(0.001)` | 1 millisecond |
| `delay(10)` | `time.sleep(0.01)` | 10 milliseconds |
| `delay(100)` | `time.sleep(0.1)` | 100 milliseconds |
| `delay(1000)` | `time.sleep(1)` | 1 second |
| `delay(5000)` | `time.sleep(5)` | 5 seconds |
{:class="table table-single table-narrow"}

## Getting Current Time: millis() vs ticks_ms()

Arduino's `millis()` returns milliseconds since boot. MicroPython has multiple timing functions:

**Arduino C++:**
```cpp
unsigned long startTime = millis();
// Do something
unsigned long elapsed = millis() - startTime;
Serial.println(elapsed);  // Prints milliseconds
```

**MicroPython:**
```python
import time

start_time = time.ticks_ms()  # Milliseconds since boot
# Do something
elapsed = time.ticks_diff(time.ticks_ms(), start_time)
print(elapsed)  # Prints milliseconds
```

**Key functions:**
- **`time.ticks_ms()`** - Returns milliseconds since boot (like `millis()`)
- **`time.ticks_us()`** - Returns microseconds since boot (like `micros()`)
- **`time.ticks_diff(new, old)`** - Calculates difference, handles wrap-around

## Why ticks_diff()? The Wrap-Around Problem

Timer values eventually wrap around (overflow). Using subtraction can give wrong results:

**Arduino C++ (wrong way):**
```cpp
unsigned long start = millis();
delay(5000);
unsigned long elapsed = millis() - start;  // Can be wrong near wrap-around!
```

**MicroPython (correct way):**
```python
start = time.ticks_ms()
time.sleep(5)
elapsed = time.ticks_diff(time.ticks_ms(), start)  # Handles wrap-around correctly
```

**Always use `time.ticks_diff()`** for calculating time differences. It handles the wrap-around case automatically.

## Blinking Without Delay Pattern

A common Arduino pattern is blinking an LED without using `delay()`, so other code can run:

**Arduino C++:**
```cpp
unsigned long previousMillis = 0;
const long interval = 1000;

void loop() {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        // Toggle LED
        if (digitalRead(LED_PIN) == HIGH) {
            digitalWrite(LED_PIN, LOW);
        } else {
            digitalWrite(LED_PIN, HIGH);
        }
    }

    // Other code can run here
}
```

**MicroPython:**
```python
from machine import Pin
import time

led = Pin(25, Pin.OUT)

previous_time = time.ticks_ms()
interval = 1000  # milliseconds

while True:
    current_time = time.ticks_ms()

    if time.ticks_diff(current_time, previous_time) >= interval:
        previous_time = current_time
        led.toggle()

    # Other code can run here
    # Do other work...
```

This pattern lets you multitask without blocking.

## Multiple Timers Example

**Arduino C++:**
```cpp
unsigned long ledPreviousMillis = 0;
unsigned long printPreviousMillis = 0;
const long ledInterval = 500;
const long printInterval = 2000;

void loop() {
    unsigned long currentMillis = millis();

    // Blink LED every 500ms
    if (currentMillis - ledPreviousMillis >= ledInterval) {
        ledPreviousMillis = currentMillis;
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }

    // Print message every 2 seconds
    if (currentMillis - printPreviousMillis >= printInterval) {
        printPreviousMillis = currentMillis;
        Serial.println("Hello!");
    }
}
```

**MicroPython:**
```python
from machine import Pin
import time

led = Pin(25, Pin.OUT)

led_previous = time.ticks_ms()
print_previous = time.ticks_ms()
led_interval = 500      # milliseconds
print_interval = 2000   # milliseconds

while True:
    current = time.ticks_ms()

    # Blink LED every 500ms
    if time.ticks_diff(current, led_previous) >= led_interval:
        led_previous = current
        led.toggle()

    # Print message every 2 seconds
    if time.ticks_diff(current, print_previous) >= print_interval:
        print_previous = current
        print("Hello!")
```

## Microsecond Timing

For precise timing, use microseconds:

**Arduino C++:**
```cpp
unsigned long startMicros = micros();
// Fast operation
unsigned long elapsed = micros() - startMicros;
Serial.println(elapsed);  // Microseconds
```

**MicroPython:**
```python
import time

start = time.ticks_us()
# Fast operation
elapsed = time.ticks_diff(time.ticks_us(), start)
print(elapsed)  # Microseconds
```

## Sleep vs Sleep_ms vs Sleep_us

MicroPython has multiple sleep functions:

```python
import time

time.sleep(1)        # Sleep 1 second
time.sleep_ms(1000)  # Sleep 1000 milliseconds (1 second)
time.sleep_us(1000000)  # Sleep 1,000,000 microseconds (1 second)
```

**When to use which:**
- **`time.sleep()`** - Most common, use for delays > 1ms
- **`time.sleep_ms()`** - When you want to think in milliseconds
- **`time.sleep_us()`** - For very short, precise delays

**Example:**
```python
# These are all equivalent (1 second delay)
time.sleep(1)
time.sleep_ms(1000)
time.sleep_us(1000000)

# But this is clearer for different scales:
time.sleep(1)          # 1 second - clear
time.sleep_ms(100)     # 100 milliseconds - clear
time.sleep_us(50)      # 50 microseconds - clear
```

## Real-Time Clock Functions

MicroPython also has real-time clock functions (if your board has an RTC):

```python
import time

# Get current time as tuple (year, month, day, hour, min, sec, weekday, yearday)
current_time = time.localtime()
print(current_time)

# Format as readable string
print(f"Date: {current_time[0]}-{current_time[1]:02d}-{current_time[2]:02d}")
print(f"Time: {current_time[3]:02d}:{current_time[4]:02d}:{current_time[5]:02d}")

# Get Unix timestamp (seconds since Jan 1, 1970)
timestamp = time.time()
print(f"Timestamp: {timestamp}")
```

**Note:** On most MicroPython boards, `time.time()` starts from 0 at boot unless you sync with NTP (Network Time Protocol) over WiFi.

## Timeout Pattern

A common pattern is checking for a condition with a timeout:

**Arduino C++:**
```cpp
bool waitForButton(unsigned long timeout) {
    unsigned long startTime = millis();

    while (millis() - startTime < timeout) {
        if (digitalRead(BUTTON_PIN) == LOW) {
            return true;
        }
    }
    return false;  // Timeout
}

// Usage
if (waitForButton(5000)) {  // Wait up to 5 seconds
    Serial.println("Button pressed!");
} else {
    Serial.println("Timeout!");
}
```

**MicroPython:**
```python
from machine import Pin
import time

button = Pin(14, Pin.IN, Pin.PULL_UP)

def wait_for_button(timeout_ms):
    start = time.ticks_ms()

    while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
        if button.value() == 0:
            return True
        time.sleep_ms(10)  # Small delay to prevent busy-waiting

    return False  # Timeout

# Usage
if wait_for_button(5000):  # Wait up to 5 seconds
    print("Button pressed!")
else:
    print("Timeout!")
```

## Performance Timing

Measure how long code takes to run:

**Arduino C++:**
```cpp
unsigned long start = millis();

// Code to measure
for (int i = 0; i < 1000; i++) {
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(LED_PIN, LOW);
}

unsigned long duration = millis() - start;
Serial.print("Duration: ");
Serial.print(duration);
Serial.println("ms");
```

**MicroPython:**
```python
from machine import Pin
import time

led = Pin(25, Pin.OUT)

start = time.ticks_us()  # Use microseconds for precision

# Code to measure
for i in range(1000):
    led.on()
    led.off()

duration = time.ticks_diff(time.ticks_us(), start)
print(f"Duration: {duration}us ({duration/1000:.2f}ms)")
```

## Debounce with Timing

Button debouncing using timing:

**Arduino C++:**
```cpp
const int DEBOUNCE_DELAY = 50;
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;

void loop() {
    int reading = digitalRead(BUTTON_PIN);

    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
        if (reading == LOW) {
            // Button is pressed and stable
            Serial.println("Button pressed!");
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

DEBOUNCE_DELAY = 50  # milliseconds
last_state = 1
last_debounce_time = time.ticks_ms()

while True:
    reading = button.value()

    if reading != last_state:
        last_debounce_time = time.ticks_ms()

    if time.ticks_diff(time.ticks_ms(), last_debounce_time) > DEBOUNCE_DELAY:
        if reading == 0:
            # Button is pressed and stable
            print("Button pressed!")
            time.sleep(0.2)  # Prevent multiple triggers

    last_state = reading
    time.sleep_ms(10)
```

## Complete Example: Traffic Light with Timing

**MicroPython:**
```python
from machine import Pin
import time

# Setup
red = Pin(10, Pin.OUT)
yellow = Pin(11, Pin.OUT)
green = Pin(12, Pin.OUT)

# Timing intervals (milliseconds)
RED_TIME = 5000
GREEN_TIME = 5000
YELLOW_TIME = 2000

# State tracking
current_light = "RED"
state_start = time.ticks_ms()

def all_off():
    red.off()
    yellow.off()
    green.off()

while True:
    current = time.ticks_ms()
    elapsed = time.ticks_diff(current, state_start)

    if current_light == "RED" and elapsed >= RED_TIME:
        all_off()
        green.on()
        current_light = "GREEN"
        state_start = current
        print("GREEN")

    elif current_light == "GREEN" and elapsed >= GREEN_TIME:
        all_off()
        yellow.on()
        current_light = "YELLOW"
        state_start = current
        print("YELLOW")

    elif current_light == "YELLOW" and elapsed >= YELLOW_TIME:
        all_off()
        red.on()
        current_light = "RED"
        state_start = current
        print("RED")

    # Other code can run here
    time.sleep_ms(10)
```

This non-blocking design allows other code to run while the traffic light cycles.

## Try It Yourself

**Exercise 1: Dual-Speed Blinker**

Create a program with two LEDs:
- LED 1 blinks every 500ms
- LED 2 blinks every 1300ms
- Both should run simultaneously without blocking
- Print a message every 5 seconds showing uptime

**Answer:**

```python
from machine import Pin
import time

led1 = Pin(10, Pin.OUT)
led2 = Pin(11, Pin.OUT)

led1_previous = time.ticks_ms()
led2_previous = time.ticks_ms()
print_previous = time.ticks_ms()

LED1_INTERVAL = 500
LED2_INTERVAL = 1300
PRINT_INTERVAL = 5000

while True:
    current = time.ticks_ms()

    # LED 1 toggle
    if time.ticks_diff(current, led1_previous) >= LED1_INTERVAL:
        led1_previous = current
        led1.toggle()

    # LED 2 toggle
    if time.ticks_diff(current, led2_previous) >= LED2_INTERVAL:
        led2_previous = current
        led2.toggle()

    # Print uptime
    if time.ticks_diff(current, print_previous) >= PRINT_INTERVAL:
        print_previous = current
        uptime_sec = current / 1000
        print(f"Uptime: {uptime_sec:.1f} seconds")

    time.sleep_ms(10)
```

**Exercise 2: Reaction Time Game**

Create a reaction time game:
1. LED turns on after a random delay (1-5 seconds)
2. User presses button as fast as possible
3. Measure and print reaction time in milliseconds
4. Repeat

**Answer:**

```python
from machine import Pin
import time
import random

led = Pin(25, Pin.OUT)
button = Pin(14, Pin.IN, Pin.PULL_UP)

def play_round():
    print("Get ready...")
    led.off()

    # Random delay 1-5 seconds
    delay = random.uniform(1, 5)
    time.sleep(delay)

    # Turn on LED and start timer
    led.on()
    start = time.ticks_ms()

    # Wait for button press
    while button.value() == 1:
        time.sleep_ms(1)

    # Calculate reaction time
    reaction_time = time.ticks_diff(time.ticks_ms(), start)
    led.off()

    print(f"Reaction time: {reaction_time}ms")
    time.sleep(2)  # Pause before next round

while True:
    play_round()
```

**Exercise 3: Timeout Button Press**

Create a program that:
- Waits for button press with 10-second timeout
- LED blinks slowly while waiting
- If button pressed: LED stays on for 3 seconds, print "Success!"
- If timeout: LED blinks rapidly 5 times, print "Timeout!"
- Repeat

**Answer:**

```python
from machine import Pin
import time

led = Pin(25, Pin.OUT)
button = Pin(14, Pin.IN, Pin.PULL_UP)

def wait_for_button_with_blink(timeout_ms):
    start = time.ticks_ms()
    last_blink = start
    led_state = False

    while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
        # Check button
        if button.value() == 0:
            return True

        # Slow blink while waiting
        if time.ticks_diff(time.ticks_ms(), last_blink) > 500:
            last_blink = time.ticks_ms()
            led_state = not led_state
            led.value(led_state)

        time.sleep_ms(10)

    return False

while True:
    print("Press button within 10 seconds...")
    led.off()

    if wait_for_button_with_blink(10000):
        # Success
        led.on()
        print("Success!")
        time.sleep(3)
    else:
        # Timeout - rapid blink
        print("Timeout!")
        for i in range(5):
            led.on()
            time.sleep(0.1)
            led.off()
            time.sleep(0.1)

    time.sleep(1)  # Pause before next round
```
