---
layout: lesson
title: Ultrasonic Range Finders
author: Kevin McAleer
type: page
cover: /learn/micropython_gpio/assets/raspberry_pi_pico_gpio.jpg
date: 2023-04-14
previous: 14_servos.html
next: 16_ground_pins.html
description: Learn how ultrasonic range finders work and how to use them with the
  Raspberry Pi Pico and MicroPython.
percent: 80
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

In this lesson, you will learn:

* What ultrasonic range finders are and how they work
* How to connect an HC-SR04 range finder to the Pico safely
* How to write MicroPython code to measure distance

---

## What are ultrasonic range finders?

[Ultrasonic range finders](/resources/how_it_works/ultrasonic.html) measure distance by sending out a burst of high-frequency sound and timing how long the echo takes to come back. Robots use them to spot obstacles.

Sound travels at about 343 metres per second, or 0.0343cm per microsecond. The sound goes there *and* back, so:

```
distance (cm) = echo time (µs) x 0.0343 / 2   ≈   echo time / 58
```

---

## Connecting an HC-SR04 to the Pico

* `VCC` to `VBUS` (5V)
* `GND` to `GND`
* `Trig` to `GP2`
* `Echo` to `GP3` **through a voltage divider** (1kΩ from Echo to GP3, then 2kΩ from GP3 to GND)

> ## Protect the Pico
>
> The standard HC-SR04 runs on 5V and its Echo pin outputs 5V, which is too much for a Pico input (see lesson 3). Use the divider, or buy the HC-SR04P, which works at 3.3V and can be powered from `3V3`.

---

## Reading distance measurements with MicroPython

1. Send a 10µs high pulse on `Trig`
2. Measure how long `Echo` stays high
3. Turn that time into a distance

```python
from machine import Pin, time_pulse_us
from time import sleep, sleep_us

trig = Pin(2, Pin.OUT)
echo = Pin(3, Pin.IN)

def distance_cm():
    trig.value(0)
    sleep_us(2)
    trig.value(1)
    sleep_us(10)
    trig.value(0)
    duration = time_pulse_us(echo, 1, 30000)   # give up after 30ms
    if duration < 0:
        return None                            # no echo: nothing in range
    return duration / 58

while True:
    d = distance_cm()
    print("Out of range" if d is None else f"Distance: {d:.1f} cm")
    sleep(0.5)
```

`time_pulse_us()` returns a negative number if no echo arrives before the timeout, so we check for that rather than printing a nonsense distance.

---

## Try it yourself

Combine this with lesson 6 so an LED lights up when something is closer than 20cm.

{% quiz %}
? Why do we divide by 58 (rather than 29) to get centimetres?
- [ ] Sound is slower in air than we think
- [x] The sound travels to the object and back again
- [ ] The sensor counts in half-microseconds
> The echo time covers the round trip, so the distance is half of what the sound travelled.

? Why does a standard HC-SR04's Echo pin need a voltage divider?
- [x] It outputs 5V, and the Pico's inputs are 3.3V only
- [ ] To make the echo louder
- [ ] To slow the pulse down
> The divider brings the 5V echo signal down to a safe level for the Pico.
{% endquiz %}

---
