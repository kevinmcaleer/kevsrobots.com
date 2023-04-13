---
layout: lesson
title: Digital I/O
author: Kevin McAleer
type: page
cover: /learn/micropython_gpio/assets/raspberry_pi_pico_gpio.jpg
previous: 03_voltage_levels.html
next: 05_pwm.html
description: Learn about digital input and output with Raspberry Pi Pico using MicroPython.
percent: 25
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

Welcome to Lesson 4 of the `Raspberry Pi Pico with MicroPython - GPIO Mastery` course. In this lesson, we will explore digital input and output with the Raspberry Pi Pico using MicroPython.

---

## Course Content

In this lesson, you will learn:

* What digital input and output are
* How to configure Raspberry Pi Pico pins for digital I/O
* How to read and write digital values using MicroPython

---

## Digital Input and Output

Digital input and output (I/O) refers to the process of sending and receiving digital signals to and from a microcontroller or computer. In the case of the Raspberry Pi Pico, digital I/O is used to communicate with other devices, such as sensors, switches, and LEDs.

Digital output is the process of sending digital signals from the Raspberry Pi Pico to other devices. This can be used to control LEDs or to send signals to other devices that can be used for various purposes.

Digital input is the process of receiving digital signals from other devices. This can be used to read values from sensors or switches.

## Configuring Raspberry Pi Pico Pins for Digital I/O

To use a pin on the Raspberry Pi Pico for digital I/O, you need to configure it as either an `input` or an `output`. This is done using the `Pin` class in MicroPython. To configure a pin as an `input`, you can use the following code:

```python
from machine import Pin

# Configure pin 0 as an input
pin = Pin(0, Pin.IN)
```

To configure a pin as an `output`, you can use the following code:

```python
from machine import Pin

# Configure pin 1 as an output
pin = Pin(1, Pin.OUT)
```

---

## Reading and Writing Digital Values

Once a pin has been configured for digital I/O, you can read and write digital values to and from it. To read the value of a digital input pin, you can use the value() method of the Pin class. For example:

```python
from machine import Pin

# Configure pin 0 as an input
pin = Pin(0, Pin.IN)

# Read the value of the input pin
value = pin.value()
```

The value variable will contain the value of the input pin.

To write a digital value to a pin, you can use the value() method of the Pin class. For example:

```python
from machine import Pin

# Configure pin 1 as an output
pin = Pin(1, Pin.OUT)

# Set the value of the output pin to 1
pin.value(1)
```

This will set the value of the output pin to 1.

---

## Conclusion

In this lesson, you learned about digital input and output, how to configure Raspberry Pi Pico pins for digital I/O, and how to read and write digital values using MicroPython. You can use this knowledge to communicate with other devices, such as sensors, switches, and LEDs.

---

## Key Results

After you have completed this lesson, you will have a solid understanding of digital input and output with the Raspberry Pi Pico using MicroPython. You will be able to configure pins for digital I/O, read digital values from sensors, and control digital devices such as LEDs and switches.

---
