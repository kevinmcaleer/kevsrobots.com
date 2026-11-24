---
layout: lesson
title: Introduction
author: Kevin McAleer
type: page
cover: /learn/micropython_gpio/assets/banners/00_intro.jpg
date: 2023-04-14
next: 01_gpio_pin_types.html
description: Learn how to control the GPIO pins on the Raspberry Pi Pico with MicroPython,
  and connect LEDs, motors, servos and sensors.
percent: 5
duration: 4
quiz_total: 54
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


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this course you will be able to wire up and control LEDs, buttons, motors, servos and sensors from the Pico's GPIO pins.

This course is written for the Raspberry Pi Pico and Pico W (RP2040) running MicroPython. Pins are 3.3V and not 5V tolerant, which comes up throughout.

> **Teachers**: This suits ages 11 and up who can already write a simple Python program. Allow 30 to 45 minutes per lesson. Each learner or pair needs a Pico, a breadboard, jumper wires, an LED with resistor and a button for lessons 1 to 6, and more parts for lessons 7 to 15. Lessons 1 to 3, 8 and 16 can be done on paper with a pinout diagram if hardware is short. Check batteries and any 5V module before powering up.

## Watch the videos

This course has two companion videos. Part 1 covers pin types, digital I/O and PWM; Part 2 covers analog input, the communication buses, servos and sensors.

{% include youtubeplayer.html id="M-0P6ipVwNU" %}

{% include youtubeplayer.html id="NlybkEO_2iU" %}

---

## Overview

Welcome to `Raspberry Pi Pico with MicroPython - GPIO Mastery`! The Pico's GPIO (General Purpose Input/Output) pins are how your code reaches out and touches the real world. In this course you'll learn what each kind of pin can do, how to wire things up safely, and how to drive LEDs, motors, servos and sensors from MicroPython.

## Course Content

The course is split into short sections:

1. **Introduction to GPIO pins** - pin types, pin numbering and voltage levels
2. **Digital I/O and PWM** - buttons, LEDs and motors
3. **Analog I/O (ADC)** - reading voltages, potentiometers and temperature sensors
4. **UART, I2C and SPI** - talking to other chips and modules
5. **Servos and ultrasonic range finders** - two robot favourites
6. **Ground pins** - why every circuit needs a common ground
7. **Recap and review**

## Key Results

After completing this course, you will be able to:

* Identify the different types of GPIO pins on the Pico and find them on the pinout
* Use digital inputs and outputs, including pull-up resistors for buttons
* Use PWM to fade LEDs, set motor speed and position servos
* Read analog voltages with the ADC
* Talk to other devices over UART, I2C and SPI
* Keep your Pico safe by respecting its 3.3V logic level

---

## What you'll need

* A Raspberry Pi Pico or Pico W with MicroPython installed (see [How to install MicroPython](/learn/how_to_install_micropython/))
* A computer with [Thonny](https://thonny.org) to edit and run code
* A breadboard, jumper wires, a few LEDs and 220Ω resistors, and a push button
* Optional parts for the later lessons: a 10kΩ potentiometer, a small DC motor and L293D motor driver chip, an SG90 servo, an HC-SR04 range finder and a DS18B20 or DHT11 temperature sensor

---

## Prerequisites

You should be comfortable writing simple Python programs. If you're brand new to MicroPython, take [Learn MicroPython - The basics](/learn/micropython/) first. If resistors, voltage and Ohm's law are new to you, [Electronics Basics](/learn/electronics_basics/) covers them.

---

## How the course works

Each lesson explains one idea, then shows real code you can run on your Pico. Most lessons end with a **Try it yourself** challenge and a quick quiz. Your quiz score and progress are saved in your browser and shown in the sidebar.

New keywords are highlighted inline like `this`.

> ## This is a note
>
> Important notes are highlighted like this

---

> ## Another aside
>
> Asides are also highlighted in sections, but in an alternative color
{:.bg-blue}

---

Let's get started!

---
