---
layout: lesson
title: Ultrasonic Range Finders
author: Kevin McAleer
type: page
cover: /learn/micropython_gpio/assets/raspberry_pi_pico_gpio.jpg
previous: 14_servos.html
next: 16_ground_pins.html
description: Learn how ultrasonic range finders work and how to use them with the
  Raspberry Pi Pico and MicroPython.
percent: 80
duration: 3
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

Welcome to Lesson 15 of the `Raspberry Pi Pico with MicroPython - GPIO Mastery` course. In this lesson, you will learn:

* What ultrasonic range finders are and how they work
* How to connect an ultrasonic range finder to the Raspberry Pi Pico
* How to write MicroPython code to read distance measurements from the ultrasonic range finder

---

## What are Ultrasonic Range Finders?

[Ultrasonic range finders](/resources/how_it_works/ultrasonic) are sensors that measure distance by emitting high-frequency sound waves and measuring the time it takes for the sound waves to bounce back from an object. They are commonly used in robotics and automation applications to detect the presence of objects and determine their distance.

---

## Connecting an Ultrasonic Range Finder to the Raspberry Pi Pico

To connect an ultrasonic range finder to the Raspberry Pi Pico, you will need to connect four pins:

* `VCC` - Connect this to a `5V` pin on the Raspberry Pi Pico
* `GND` - Connect this to a `GND` pin on the Raspberry Pi Pico
* `Trig` - Connect this to a `GPIO` pin on the Raspberry Pi Pico
* `Echo` - Connect this to another `GPIO` pin on the Raspberry Pi Pico

---

## Reading Distance Measurements with MicroPython

To read distance measurements from the ultrasonic range finder using MicroPython, you will need to:

1. Set the trigger pin as an output and the echo pin as an input
2. Send a high signal to the trigger pin for a brief amount of time
3. Measure the amount of time it takes for the echo pin to receive a high signal
4. Calculate the distance based on the time measurement

Here is an example MicroPython code that reads distance measurements from an ultrasonic range finder:

```python
import machine
import time

trig_pin = machine.Pin(2, machine.Pin.OUT)
echo_pin = machine.Pin(3, machine.Pin.IN)

while True:
    # Send a 10 microsecond pulse to the trigger pin
    trig_pin.value(0)
    time.sleep_us(2)
    trig_pin.value(1)
    time.sleep_us(10)
    trig_pin.value(0)

    # Measure the duration of the echo pulse
    duration = machine.time_pulse_us(echo_pin, 1)

    # Calculate the distance in centimeters
    distance = duration / 58

    print("Distance: {} cm".format(distance))
    time.sleep(1)
```

This code continuously reads distance measurements from the ultrasonic range finder and prints the results to the console.

---

## Summary

In this lesson, you learned what ultrasonic range finders are, how to connect them to the Raspberry Pi Pico, and how to use MicroPython to read distance measurements from them.

---
