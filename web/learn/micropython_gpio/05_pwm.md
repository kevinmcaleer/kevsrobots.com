---
layout: lesson
title: Pulse Width Modulation (PWM)
author: Kevin McAleer
type: page
cover: /learn/micropython_gpio/assets/raspberry_pi_pico_gpio.jpg
date: 2023-04-14
previous: 04_digital_io.html
next: 06_leds.html
description: Learn about Pulse Width Modulation (PWM) and controlling devices like
  LEDs and motors using the Raspberry Pi Pico with MicroPython.
percent: 30
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

Welcome to Lesson 5 of the `Raspberry Pi Pico with MicroPython - GPIO Mastery` course. In this lesson, we will explore Pulse Width Modulation (PWM) and how to control devices like LEDs and motors using the Raspberry Pi Pico with MicroPython.

---

## Course Content

In this lesson, you will learn:

* What Pulse Width Modulation (PWM) is and how it works
* How to configure Raspberry Pi Pico pins for PWM
* How to control devices like LEDs and motors using PWM and MicroPython

---

## Pulse Width Modulation

Pulse Width Modulation (PWM) is a technique used to control the amount of power delivered to a device, such as an LED or a motor. PWM works by rapidly turning the device on and off at a specific frequency, and varying the duty cycle (the percentage of time the device is on) to control the amount of power delivered to the device.

## Configuring Raspberry Pi Pico Pins for PWM

To use a pin on the Raspberry Pi Pico for PWM, you need to configure it for PWM. This is done using the `PWM` class in MicroPython. To configure a pin for PWM, you can use the following code:

```python
from machine import Pin, PWM

# Configure pin 0 for PWM with a frequency of 1000 Hz
pwm = PWM(Pin(0), freq=1000)
```

## Controlling Devices Using PWM

To control the amount of power delivered to a device using PWM, you can use the `duty()` method of the PWM class. The `duty()` method takes a value between 0 (device off) and 1023 (device fully on) to set the duty cycle of the PWM signal. For example, to set the LED brightness to 50%, you can use the following code:

```python
from machine import Pin, PWM

# Configure pin 0 for PWM with a frequency of 1000 Hz
pwm = PWM(Pin(0), freq=1000)

# Set the LED brightness to 50%
pwm.duty(512)
```

---

## Conclusion

In this lesson, you learned about Pulse Width Modulation (PWM), how to configure Raspberry Pi Pico pins for PWM, and how to control devices like LEDs and motors using PWM and MicroPython. You can use this knowledge to create dynamic and interactive projects with your Raspberry Pi Pico board.

---

## Key Results

After you have completed this lesson, you will have a solid understanding of Pulse Width Modulation (PWM) and how to control devices like LEDs and motors using the Raspberry Pi Pico with MicroPython. You will be able to configure pins for PWM, generate PWM signals, and control devices that require PWM input.

---
