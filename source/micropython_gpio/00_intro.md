---
layout: lesson
title: Introduction
type: page
description: Learn how to control the GPIO pins on the Raspberry Pi Pico with MicroPython, and connect LEDs, motors, servos and sensors.
---

![Cover photo]({{page.cover}}){:class="cover"}

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
* Optional parts for the later lessons: a 10kΩ potentiometer, a small DC motor and motor driver (L293D or DRV8833), an SG90 servo, an HC-SR04 range finder and a DS18B20 or DHT11 temperature sensor

---

## Prerequisites

You should be comfortable writing simple Python programs. If you're brand new to MicroPython, take [Learn MicroPython - The basics](/learn/micropython/) first.

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
