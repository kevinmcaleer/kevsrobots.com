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


<!-- ![Cover photo of a potentiometer connected to a Raspberry Pi Pico board](assets/raspberry_pi_pico_potentiometer.jpg){:class="cover"} -->

## Overview

Welcome to Lesson 9 of the `Raspberry Pi Pico with MicroPython - GPIO Mastery` course. In this lesson, we will learn how to use potentiometers with the Raspberry Pi Pico and MicroPython.

## Course Content

In this lesson, you will learn:

* What potentiometers are and how they work
* How to connect a potentiometer to the Raspberry Pi Pico
* How to write MicroPython code to read potentiometer values

---

## What are Potentiometers?

`Potentiometers` are three-terminal resistors with a sliding contact that forms a variable voltage divider. By adjusting the position of the sliding contact, the output voltage can be varied between the input voltage and ground. You can learn more about [how potentiometers work here](/resources/how_it_works/pots).

---

## Connecting a Potentiometer to Raspberry Pi Pico

To connect a potentiometer to your Raspberry Pi Pico board, follow these steps:

1. Connect the potentiometer's outer pins to the 3.3V and GND pins on the Raspberry Pi Pico board.
2. Connect the potentiometer's center pin to a GPIO pin on the Raspberry Pi Pico board.

---

## Reading Potentiometer Values with MicroPython

To read potentiometer values using MicroPython, you can use the `machine.ADC` module. Here's some example code to get you started:

```python
from machine import Pin, ADC
import time

potentiometer_pin = Pin(26, Pin.IN)
adc = ADC(potentiometer_pin)
adc.atten(ADC.ATTN_11DB)

while True:
    potentiometer_value = adc.read()
    print(potentiometer_value)
    time.sleep(0.1)
```

---

## Conclusion

In this lesson, you learned how to use potentiometers to control the input voltage to the Raspberry Pi Pico board, and how to read potentiometer values using MicroPython. You can use this knowledge to create a variety of projects with your Raspberry Pi Pico board.

---

## Key Results

After you have completed this lesson, you will know how to use potentiometers with the Raspberry Pi Pico and MicroPython. You will be able to connect a potentiometer to the Pico, write MicroPython code to read potentiometer values, and understand the basics of potentiometer operation.

---
