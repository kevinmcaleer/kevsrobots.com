---
layout: lesson
title: Using Potentiometers
author: Kevin McAleer
type: page
cover: /learn/micropython_gpio/assets/raspberry_pi_pico_gpio.jpg
date: 2023-04-14
previous: 08_adc.html
next: 10_temperature_sensors.html
description: Learn how to use potentiometers with the Raspberry Pi Pico and MicroPython.
percent: 50
duration: 2
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

A potentiometer (or "pot") is a knob that gives you a voltage you can change by hand. It's the perfect first analog input. In this lesson you'll wire one up and read it.

---

## What are potentiometers?

`Potentiometers` are three-terminal resistors with a sliding contact that forms a variable voltage divider. Turn the knob and the middle pin's voltage moves smoothly between the voltages on the two outer pins. You can learn more about [how potentiometers work here](/resources/how_it_works/pots.html).

---

## Connecting a potentiometer to the Pico

1. Connect one outer pin to `3V3(OUT)` (physical pin 36)
2. Connect the other outer pin to `GND` (or `AGND`, physical pin 33, for the cleanest readings)
3. Connect the middle pin (the wiper) to `GP26` (physical pin 31)

A 10kΩ potentiometer is a good choice.

---

## Reading potentiometer values with MicroPython

```python
from machine import ADC
from time import sleep

pot = ADC(26)

while True:
    value = pot.read_u16()
    percent = value * 100 // 65535
    print(value, f"{percent}%")
    sleep(0.1)
```

Turn the knob and watch the numbers change. Thonny's **View > Plotter** draws them as a graph.

> ## ESP32 code won't work here
>
> Examples for the ESP32 use `adc.atten()` and `adc.read()`. The Pico doesn't have these: use `read_u16()`.

---

## Try it yourself

Combine this with lesson 6: use the potentiometer to set the LED brightness by passing `pot.read_u16()` straight to `led.duty_u16()`.

{% quiz %}
? Which potentiometer pin goes to the ADC input?
- [ ] Either outer pin
- [x] The middle pin (the wiper)
- [ ] None, it connects to 3V3
> The wiper's voltage changes as you turn the knob. The outer pins go to 3V3 and GND.
{% endquiz %}

---
