---
layout: lesson
title: Analog Input (ADC)
author: Kevin McAleer
type: page
cover: /learn/micropython_gpio/assets/raspberry_pi_pico_gpio.jpg
date: 2023-04-14
previous: 07_motors.html
next: 09_potentiometers.html
description: Learn about analog input and the ADC on the Raspberry Pi Pico using MicroPython.
percent: 45
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

Welcome to Lesson 8 of the `Raspberry Pi Pico with MicroPython - GPIO Mastery` course. In this lesson, we will explore analog input and the Analog-to-Digital Converter (ADC) on the Raspberry Pi Pico using MicroPython.

---

## Course Content

In this lesson, you will learn:

* What analog input is and the role of ADC
* How to configure Raspberry Pi Pico pins for analog input
* How to read analog values using MicroPython

---

## Analog Input and ADC

Analog input is used to measure the continuous range of values of a physical quantity, such as light, sound, or temperature. In contrast, digital signals represent values in binary format (0s and 1s).

Raspberry Pi Pico uses ADC to convert the continuous analog signal to a discrete digital signal that can be processed by the computer.

## Configuring Raspberry Pi Pico Pins for Analog Input

To use a pin on the Raspberry Pi Pico for analog input, you need to configure it for analog to digital conversion (ADC). This is done using the `ADC` class in MicroPython. To configure a pin for ADC, you can use the following code:

```python
from machine import ADC, Pin

# Configure pin 0 for analog input
adc = ADC(Pin(0))
```

---

## Reading Analog Values

Once a pin has been configured for analog input, you can read analog values from it. To read the value of an analog input pin, you can use the `read_u16()` method of the `ADC` class. This method returns the value of the analog input pin as a 16-bit unsigned integer between `0` and `65535`. For example:

```python
from machine import ADC, Pin

# Configure pin 0 for analog input
adc = ADC(Pin(0))

# Read the value of the analog input pin
value = adc.read_u16()
```

The `value` variable will contain the value of the analog input pin.

---

## Conclusion

In this lesson, you learned about analog input, the role of ADC (Analog to Digital Converter), how to configure Raspberry Pi Pico pins for analog input, and how to read analog values using MicroPython. You can use this knowledge to measure physical quantities, such as light, sound, or temperature, and process them in your Raspberry Pi Pico projects.

---

## Key Results

After you have completed this lesson, you will have a solid understanding of analog input and the ADC on the Raspberry Pi Pico using MicroPython. You will be able to configure pins for analog input, read analog values from sensors, and process them in your MicroPython programs.

---
