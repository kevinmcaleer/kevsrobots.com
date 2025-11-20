---
layout: lesson
title: Analog Reading and PWM - ADC and PWM Classes
author: Kevin McAleer
type: page
cover: /learn/arduino_to_python/assets/cover.jpg
date: 2025-11-20
previous: 05_pin_control.html
next: 07_timing.html
description: Learn how to read analog sensors with the ADC class and control PWM outputs
  for LED brightness and motor speed control
percent: 56
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


## Analog Input and Output: A Class-Based Approach

Arduino uses `analogRead()` and `analogWrite()` functions. MicroPython uses object-oriented classes: `ADC` for reading analog values and `PWM` for pulse-width modulation output.

The biggest difference? **Resolution and duty cycle values**. Arduino uses 0-1023 for ADC and 0-255 for PWM. MicroPython boards use different resolutions, typically 12-bit or 16-bit ADC and 16-bit PWM.

## Reading Analog Values with ADC

**Arduino C++:**
```cpp
const int SENSOR_PIN = A0;

void setup() {
    Serial.begin(9600);
}

void loop() {
    int sensorValue = analogRead(SENSOR_PIN);  // 0-1023 (10-bit)
    Serial.println(sensorValue);
    delay(1000);
}
```

**MicroPython:**
```python
from machine import Pin, ADC
import time

sensor = ADC(Pin(26))  # Create ADC object

while True:
    # Read raw value
    value = sensor.read_u16()  # 0-65535 (16-bit)
    print(value)
    time.sleep(1)
```

**Key differences:**
- **Import ADC** from the `machine` module
- **Create ADC object** with a Pin
- **`.read_u16()`** reads a 16-bit value (0-65535)
- Higher resolution than Arduino's 10-bit (0-1023)

## ADC Pin Compatibility

Different boards have different ADC-capable pins:

**Arduino Nano ESP32:**
- A0-A7 (analog pins)
- Some digital pins also support ADC

**Raspberry Pi Pico / Pico W / Pico 2W:**
- GP26 (ADC0), GP27 (ADC1), GP28 (ADC2)
- GP29 (ADC3) - used for measuring VSYS voltage
- Only these 4 pins support ADC!

**Example:**
```python
# Pico - Use GP26, GP27, or GP28
sensor = ADC(Pin(26))  # ADC0

# Arduino Nano ESP32 - Use A0-A7 or ADC-capable pins
sensor = ADC(Pin(A0))  # or sensor = ADC(Pin(2)) if pin 2 supports ADC
```

## Converting ADC Values to Voltage

Arduino's 10-bit ADC with 5V reference:

**Arduino C++:**
```cpp
int rawValue = analogRead(A0);
float voltage = (rawValue / 1023.0) * 5.0;
```

**MicroPython (Pico with 3.3V reference):**
```python
sensor = ADC(Pin(26))
raw_value = sensor.read_u16()
voltage = (raw_value / 65535) * 3.3  # 16-bit, 3.3V reference
```

**Important:** Most MicroPython boards use **3.3V** reference, not 5V!

## Reading Temperature Sensor Example

**Arduino C++:**
```cpp
// TMP36 temperature sensor on A0
void loop() {
    int rawValue = analogRead(A0);
    float voltage = (rawValue / 1023.0) * 5.0;
    float temperatureC = (voltage - 0.5) * 100.0;

    Serial.print("Temperature: ");
    Serial.print(temperatureC);
    Serial.println("C");

    delay(1000);
}
```

**MicroPython:**
```python
from machine import Pin, ADC
import time

sensor = ADC(Pin(26))  # TMP36 on GP26

while True:
    raw_value = sensor.read_u16()
    voltage = (raw_value / 65535) * 3.3
    temperature_c = (voltage - 0.5) * 100.0

    print(f"Temperature: {temperature_c:.1f}C")
    time.sleep(1)
```

## PWM for LED Brightness Control

Arduino uses `analogWrite()`. MicroPython uses the `PWM` class:

**Arduino C++:**
```cpp
const int LED_PIN = 9;

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

**MicroPython:**
```python
from machine import Pin, PWM
import time

led = PWM(Pin(25))  # Create PWM object
led.freq(1000)      # Set frequency to 1000 Hz

while True:
    # Fade in
    for brightness in range(0, 65536, 256):  # 0-65535 (16-bit)
        led.duty_u16(brightness)
        time.sleep(0.01)

    # Fade out
    for brightness in range(65535, -1, -256):
        led.duty_u16(brightness)
        time.sleep(0.01)
```

**Key differences:**
- **Create PWM object** instead of using `analogWrite()`
- **Set frequency** with `.freq()` (typically 1000 Hz)
- **Set duty cycle** with `.duty_u16()` (0-65535, 16-bit)
- Arduino's 0-255 = MicroPython's 0-65535

## PWM Duty Cycle

**Duty cycle** is the percentage of time the signal is HIGH:
- 0% duty cycle = always OFF
- 50% duty cycle = half ON, half OFF
- 100% duty cycle = always ON

**Arduino C++:**
```cpp
analogWrite(LED_PIN, 0);    // 0% (off)
analogWrite(LED_PIN, 64);   // 25% brightness
analogWrite(LED_PIN, 128);  // 50% brightness
analogWrite(LED_PIN, 191);  // 75% brightness
analogWrite(LED_PIN, 255);  // 100% (full on)
```

**MicroPython:**
```python
led.duty_u16(0)      # 0% (off)
led.duty_u16(16384)  # 25% brightness
led.duty_u16(32768)  # 50% brightness
led.duty_u16(49152)  # 75% brightness
led.duty_u16(65535)  # 100% (full on)
```

## Converting Arduino PWM to MicroPython

To convert Arduino's 0-255 range to MicroPython's 0-65535:

```python
def arduino_to_micropython_pwm(arduino_value):
    """Convert Arduino PWM value (0-255) to MicroPython (0-65535)"""
    return int((arduino_value / 255) * 65535)

# Example
led.duty_u16(arduino_to_micropython_pwm(128))  # 50% brightness
```

Or use percentage:

```python
def percent_to_duty(percent):
    """Convert percentage (0-100) to duty cycle (0-65535)"""
    return int((percent / 100) * 65535)

led.duty_u16(percent_to_duty(75))  # 75% brightness
```

## Motor Speed Control

PWM is commonly used for motor speed control:

**Arduino C++:**
```cpp
const int MOTOR_PIN = 9;
const int DIR_PIN = 8;

void setup() {
    pinMode(MOTOR_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
}

void loop() {
    // Forward at 50% speed
    digitalWrite(DIR_PIN, HIGH);
    analogWrite(MOTOR_PIN, 128);
    delay(2000);

    // Forward at 100% speed
    analogWrite(MOTOR_PIN, 255);
    delay(2000);

    // Stop
    analogWrite(MOTOR_PIN, 0);
    delay(1000);
}
```

**MicroPython:**
```python
from machine import Pin, PWM
import time

motor = PWM(Pin(9))
motor.freq(1000)
direction = Pin(8, Pin.OUT)

while True:
    # Forward at 50% speed
    direction.on()
    motor.duty_u16(32768)  # 50%
    time.sleep(2)

    # Forward at 100% speed
    motor.duty_u16(65535)  # 100%
    time.sleep(2)

    # Stop
    motor.duty_u16(0)
    time.sleep(1)
```

## Servo Control

Servos are controlled with PWM at 50Hz frequency:

**Arduino C++:**
```cpp
#include <Servo.h>

Servo myServo;

void setup() {
    myServo.attach(9);
}

void loop() {
    myServo.write(0);     // 0 degrees
    delay(1000);
    myServo.write(90);    // 90 degrees
    delay(1000);
    myServo.write(180);   // 180 degrees
    delay(1000);
}
```

**MicroPython:**
```python
from machine import Pin, PWM
import time

servo = PWM(Pin(9))
servo.freq(50)  # Servos use 50 Hz

def set_servo_angle(angle):
    """Set servo angle (0-180 degrees)"""
    # Servo pulse width: 1ms (0°) to 2ms (180°)
    # At 50Hz, duty cycle = pulse_width / 20ms
    min_duty = 1638   # ~1ms pulse (0°)
    max_duty = 8192   # ~2ms pulse (180°)
    duty = int(min_duty + (angle / 180) * (max_duty - min_duty))
    servo.duty_u16(duty)

while True:
    set_servo_angle(0)    # 0 degrees
    time.sleep(1)
    set_servo_angle(90)   # 90 degrees
    time.sleep(1)
    set_servo_angle(180)  # 180 degrees
    time.sleep(1)
```

Servo control requires calculating duty cycles based on pulse width. Many MicroPython projects use a servo library to simplify this.

## RGB LED Control

Control an RGB LED with three PWM pins:

**MicroPython:**
```python
from machine import Pin, PWM
import time

red = PWM(Pin(10))
green = PWM(Pin(11))
blue = PWM(Pin(12))

red.freq(1000)
green.freq(1000)
blue.freq(1000)

def set_color(r, g, b):
    """Set RGB color (0-100 for each channel)"""
    red.duty_u16(int(r / 100 * 65535))
    green.duty_u16(int(g / 100 * 65535))
    blue.duty_u16(int(b / 100 * 65535))

while True:
    set_color(100, 0, 0)    # Red
    time.sleep(1)
    set_color(0, 100, 0)    # Green
    time.sleep(1)
    set_color(0, 0, 100)    # Blue
    time.sleep(1)
    set_color(100, 100, 0)  # Yellow
    time.sleep(1)
    set_color(0, 100, 100)  # Cyan
    time.sleep(1)
    set_color(100, 0, 100)  # Magenta
    time.sleep(1)
    set_color(100, 100, 100) # White
    time.sleep(1)
```

## Complete Example: Light-Controlled LED

Read a light sensor (LDR) and adjust LED brightness inversely:

**Arduino C++:**
```cpp
const int LDR_PIN = A0;
const int LED_PIN = 9;

void setup() {
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(9600);
}

void loop() {
    int lightLevel = analogRead(LDR_PIN);  // 0-1023

    // Invert: bright light = dim LED
    int brightness = map(lightLevel, 0, 1023, 255, 0);

    analogWrite(LED_PIN, brightness);

    Serial.print("Light: ");
    Serial.print(lightLevel);
    Serial.print(" -> Brightness: ");
    Serial.println(brightness);

    delay(100);
}
```

**MicroPython:**
```python
from machine import Pin, ADC, PWM
import time

ldr = ADC(Pin(26))
led = PWM(Pin(25))
led.freq(1000)

while True:
    light_level = ldr.read_u16()  # 0-65535

    # Invert: bright light = dim LED
    brightness = 65535 - light_level

    led.duty_u16(brightness)

    print(f"Light: {light_level} -> Brightness: {brightness}")
    time.sleep(0.1)
```

## PWM Frequency Considerations

Different applications need different PWM frequencies:

| Application | Frequency | Reason |
|-------------|-----------|---------|
| LED dimming | 500-2000 Hz | Avoids visible flicker |
| Motor control | 1000-20000 Hz | Smooth operation, reduces noise |
| Servo control | 50 Hz | Standard servo pulse timing |
| Piezo buzzer | Varies | Matches note frequency |
{:class="table table-single table-narrow"}

Set frequency with `.freq()`:

```python
led.freq(1000)    # 1000 Hz for LED
motor.freq(20000) # 20 kHz for motor
servo.freq(50)    # 50 Hz for servo
```

## Try It Yourself

**Exercise 1: Breathing LED**

Create a "breathing" LED effect that smoothly fades in and out continuously. Use a sine wave pattern for natural-looking breathing.

Hint: Use `math.sin()` to create smooth transitions.

**Answer:**

```python
from machine import Pin, PWM
import time
import math

led = PWM(Pin(25))
led.freq(1000)

angle = 0

while True:
    # Use sine wave for smooth breathing effect
    brightness = int((math.sin(angle) + 1) * 32767.5)  # 0-65535
    led.duty_u16(brightness)

    angle += 0.05  # Adjust for speed
    if angle > 2 * math.pi:
        angle = 0

    time.sleep(0.02)
```

**Exercise 2: Potentiometer-Controlled Motor**

Read a potentiometer on an ADC pin and control motor speed proportionally:
- Potentiometer at minimum = motor off
- Potentiometer at maximum = motor at full speed
- Print voltage and speed percentage

**Answer:**

```python
from machine import Pin, ADC, PWM
import time

pot = ADC(Pin(26))
motor = PWM(Pin(9))
motor.freq(1000)

while True:
    pot_value = pot.read_u16()
    voltage = (pot_value / 65535) * 3.3

    # Use pot value directly for motor speed
    motor.duty_u16(pot_value)

    speed_percent = (pot_value / 65535) * 100

    print(f"Voltage: {voltage:.2f}V | Speed: {speed_percent:.0f}%")
    time.sleep(0.1)
```

**Exercise 3: RGB LED Color Mixer**

Use 3 potentiometers (or simulate with variables) to control an RGB LED:
- Pot 1 controls red intensity
- Pot 2 controls green intensity
- Pot 3 controls blue intensity

Create smooth color transitions when values change.

**Answer:**

```python
from machine import Pin, ADC, PWM
import time

# RGB LED pins
red = PWM(Pin(10))
green = PWM(Pin(11))
blue = PWM(Pin(12))

red.freq(1000)
green.freq(1000)
blue.freq(1000)

# Potentiometers (use GP26, GP27, GP28 on Pico)
pot_r = ADC(Pin(26))
pot_g = ADC(Pin(27))
pot_b = ADC(Pin(28))

while True:
    # Read potentiometer values
    r_value = pot_r.read_u16()
    g_value = pot_g.read_u16()
    b_value = pot_b.read_u16()

    # Set LED colors
    red.duty_u16(r_value)
    green.duty_u16(g_value)
    blue.duty_u16(b_value)

    # Convert to percentages for display
    r_percent = (r_value / 65535) * 100
    g_percent = (g_value / 65535) * 100
    b_percent = (b_value / 65535) * 100

    print(f"R: {r_percent:.0f}% | G: {g_percent:.0f}% | B: {b_percent:.0f}%")
    time.sleep(0.1)
```
