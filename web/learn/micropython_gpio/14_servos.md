---
layout: lesson
title: Introduction to Servos
author: Kevin McAleer
type: page
cover: /learn/micropython_gpio/assets/raspberry_pi_pico_gpio.jpg
date: 2023-04-14
previous: 13_spi.html
next: 15_ultrasonic_range_finders.html
description: Learn about servos and how to control them with the Raspberry Pi Pico
  and MicroPython.
percent: 75
duration: 3
quiz_total: 33
navigation:
- name: Raspberry Pi Pico with MicroPython - GPIO Mastery
- content:
  - section: Overview
    content:
    - name: Introduction
      link: 00_intro.html
  - section: Introduction to GPIO Pins
    content:
    - name: GPIO Pin Types
      link: 01_gpio_pin_types.html
    - name: Pin Numbering
      link: 02_pin_numbering.html
    - name: Voltage Levels
      link: 03_voltage_levels.html
  - section: Digital I/O and PWM
    content:
    - name: Digital I/O
      link: 04_digital_io.html
    - name: Pulse Width Modulation (PWM)
      link: 05_pwm.html
    - name: Controlling LEDs with PWM
      link: 06_leds.html
    - name: Controlling Motors with PWM
      link: 07_motors.html
  - section: Analog I/O (ADC)
    content:
    - name: Analog Input (ADC)
      link: 08_adc.html
    - name: Using Potentiometers
      link: 09_potentiometers.html
    - name: Using Temperature Sensors
      link: 10_temperature_sensors.html
  - section: UART, I2C, and SPI
    content:
    - name: Introduction to UART
      link: 11_uart.html
    - name: Introduction to I2C
      link: 12_i2c.html
    - name: Introduction to SPI
      link: 13_spi.html
  - section: Interfacing with Servos and Ultrasonic Range Finders
    content:
    - name: Introduction to Servos
      link: 14_servos.html
    - name: Ultrasonic Range Finders
      link: 15_ultrasonic_range_finders.html
  - section: Ground Pins
    content:
    - name: Ground Pins
      link: 16_ground_pins.html
  - section: Summary and Review
    content:
    - name: Recap and Review
      link: 17_summary.html
---


## Overview

A servo is a motor that turns to a position you choose and holds it there, which makes it perfect for robot arms, legs and pan-tilt heads. In this lesson you'll wire one up and control it with PWM.

---

## What is a servo?

A [servo](/resources/how_it_works/servos.html) has three wires:

* **Power** (usually red) - 5V for hobby servos like the SG90
* **Ground** (brown or black)
* **Signal** (orange or yellow) - a PWM signal that sets the position

The servo reads the **pulse width**: a pulse is sent 50 times a second, and its length sets the angle. For an SG90, about 0.5ms is 0°, 1.5ms is 90° and 2.5ms is 180°.

---

## Connecting a servo to the Pico

* Power to `VBUS` (5V from USB, physical pin 40)
* Ground to `GND`
* Signal to `GP18` (physical pin 24). Any GPIO pin will work

One small servo is fine from USB power. For more than one, or bigger servos, use a separate 5V supply and connect its ground to the Pico's ground.

---

## Controlling a servo with MicroPython

At 50Hz each cycle lasts 20ms, and `duty_u16()` covers that whole 20ms with 0 to 65535. So 1ms is about `65535 / 20 = 3277`.

```python
from machine import Pin, PWM
from time import sleep

servo = PWM(Pin(18))
servo.freq(50)

def set_angle(angle):
    pulse_ms = 0.5 + (angle / 180) * 2.0          # 0.5ms to 2.5ms
    servo.duty_u16(int(pulse_ms * 65535 / 20))

for angle in (0, 90, 180, 90):
    set_angle(angle)
    sleep(1)

servo.deinit()
```

- `servo.freq(50)` sends 50 pulses per second, which is what servos expect
- `set_angle()` turns an angle into a pulse width, then into a duty value
- `servo.deinit()` stops the signal, so the servo relaxes

> ## Check your servo's range
>
> Servos vary. If yours buzzes or strains at 0° or 180°, narrow the range, for example 1.0ms to 2.0ms.

---

## Try it yourself

Use a potentiometer (lesson 9) to set the servo angle, so turning the knob turns the servo.

{% quiz %}
? What sets a servo's position?
- [ ] The voltage on the power wire
- [x] The width of the pulses on the signal wire
- [ ] The PWM frequency
> The servo measures how long each pulse lasts. Around 1.5ms is the middle position.

? At 50Hz, roughly what `duty_u16()` value gives a 1.5ms pulse?
- [ ] 7500
- [x] 4915
- [ ] 32768
> 1.5ms is 7.5% of a 20ms cycle, and 7.5% of 65535 is about 4915.
{% endquiz %}

---
