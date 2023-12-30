---
layout: lesson
title: GPIO Pin Types
author: Kevin McAleer
type: page
cover: /learn/micropython_gpio/assets/burger_bot.jpg
date: 2023-04-14
previous: 00_intro.html
next: 02_pin_numbering.html
description: Learn about the different types of GPIO pins on the Raspberry Pi Pico
  and their basic functions.
percent: 10
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

Welcome to Lesson 1 of the `Raspberry Pi Pico with MicroPython - GPIO Mastery` course. In this lesson, we will discuss the different types of GPIO pins available on the Raspberry Pi Pico and their basic functions. We will cover the following types of pins:

- Digital I/O
- Analog I/O (ADC)
- PWM
- UART
- I2C
- SPI
- GND

---

## Course Content

GPIO (General Purpose Input/Output) pins are used to connect electronic components to the Raspberry Pi Pico. In this lesson, you will learn:

- Different types of GPIO pins on the Raspberry Pi Pico
- Their basic functions and how they can be used in various applications
- Pin numbering and how to identify pins on the Pico board

### Different types of GPIO pins

The Raspberry Pi Pico has several types of GPIO pins, including:

- `Digital I/O pins`, which can be used to send or receive digital signals
- `PWM` (Pulse Width Modulation) pins, which can be used to control the brightness of LEDs or the speed of motors
- `ADC` (Analog-to-Digital Converter) pins, which can be used to read analog signals from sensors
- `I2C` (Inter-Integrated Circuit) pins, which can be used to communicate with other devices using the I2C protocol
- `SPI` (Serial Peripheral Interface) pins, which can be used to communicate with other devices using the SPI protocol

---

### Basic functions of GPIO pins

`Digital I/O pins` can be used to send or receive digital signals, which can be used to control LEDs, read button presses, or communicate with other digital devices.

`PWM pins` can be used to control the brightness of LEDs, the speed of motors, or the position of servos by adjusting the duty cycle of the PWM signal.

`ADC pins` can be used to read analog signals from sensors, such as temperature or light sensors, and convert them into digital values that can be processed by the Pico.

`I2C` and `SPI pins` can be used to communicate with other devices, such as sensors, displays, or other microcontrollers, using the [I2C](/resources/glossary#i2c) or [SPI](/resources/glossary#spi) protocol.

---

## Key Results

After you have completed this lesson, you will have a solid understanding of the various GPIO pins available on the Raspberry Pi Pico. This knowledge will enable you to confidently use these pins in your projects, allowing you to interface with various devices and components.

---

## MicroPython code example

The code below imports the `machine` module, which provides access to hardware-related functionality on the Raspberry Pi Pico.

Then it initializes a GPIO pin with the number 25 as an output pin using the `machine.Pin` class, and assigns it to the variable `pin`.

Finally, it sets the value of the output pin to 1 using the `pin.value(1)` method. This means that the pin will output a voltage of 3.3V, which can be used to turn on a connected device, such as an LED.

```python
import machine
pin = machine.Pin(25, machine.Pin.OUT)
pin.value(1)
```

---
