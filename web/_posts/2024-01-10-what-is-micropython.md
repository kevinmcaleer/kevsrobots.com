---
title: What is MicroPython?
description: >- 
    Learn how to make a robot using Micropython, the Python programming language for microcontrollers.
layout: project
date: 2024-01-10
cover: /assets/img/groups/micropython.png
excerpt: >-
    MicroPython is a lean and efficient implementation of the Python 3 programming language that includes a small subset of the Python standard library and is optimised to run on microcontrollers and in constrained environments.
author: Kevin McAleer
difficulty: beginner
groups:
    - robots
    - raspberrypi
    - micropython
    - pico
tags:
    - micropython
    - microcontrollers
    - python
    - raspberry_pi

---

## What is MicroPython?

`MicroPython` is a lean and efficient implementation of the Python 3 programming language that includes a small subset of the Python standard library and is optimised to run on microcontrollers and in constrained environments.

MicroPython offers many sophisticated features like an interactive prompt, high-precision integers, closures, list comprehension, generators, and exception handling, all within a small footprint.

It needs only 256k of code space and 16k of RAM to operate. Its design closely matches standard Python, making it easy to move code from a desktop to a microcontroller or embedded system.

---

## Why is it called MicroPython?

MicroPython is a smaller version of the programming language Python, designed for use with MicroControllers, the name is a concatenation of the two words, Micro + Python where Micro refers to the MicroController.

- Micro + Python

`Python` gets it name from a TV show…

> “When he began implementing Python, Guido van Rossum was also reading the published scripts from ‘Monty Python’s Flying Circus’, a BBC comedy series from the 1970s. Van Rossum thought he needed a name that was short, unique, and slightly mysterious, so he decided to call the language Python”

---

## What Microcontrollers can run MicroPython?

MicroPython is versatile and can run on a wide range of microcontrollers from various manufacturers, catering to different requirements and applications.

Key players include the ESP8266 and ESP32 from Espressif Systems, known for their Wi-Fi capabilities; the Pyboard series, developed by the creator of MicroPython himself, Damien George; STM32 microcontrollers from STMicroelectronics, renowned for their performance and range of features; and the Raspberry Pi Pico, which uses the RP2040 chip developed by Raspberry Pi Foundation, famous for its affordability and ease of use.

Each of these microcontrollers brings unique features to the table, such as advanced connectivity, high processing power, or cost-effectiveness, making MicroPython suitable for a broad spectrum of projects and applications.

Here's a table listing some popular microcontrollers that can run MicroPython, along with their manufacturers and links for more information:

{% include micropython_boards.md %}
{:class="table table-striped table-hover"}

Each link leads to the official page or a primary resource for the respective microcontroller, providing detailed specifications and purchase options.

---

## What's the difference between MicroPython and CircuitPython?

`CircuitPython` and MicroPython are both Python implementations for microcontrollers, but they have distinct focuses and features. MicroPython, the precursor to CircuitPython, is a lean and efficient version of Python 3 designed for microcontrollers, emphasizing broad compatibility with a range of hardware and performance in constrained environments. 

CircuitPython (<https://circuitpython.org>), developed by [Adafruit](https://www.adafruit.com), is a fork of MicroPython that prioritizes ease of use and simplicity, particularly for beginners and in educational settings. It focuses on Adafruit's hardware but supports various other microcontrollers too.

One key difference is that CircuitPython emphasizes strong support for USB devices and comes with a built-in USB HID (Human Interface Device) and MIDI support.

Additionally, CircuitPython tends to have more frequent releases and updates, with a strong emphasis on community involvement and extensive documentation, making it particularly friendly for newcomers to programming and electronics.

---

## Is MicroPython the same as Python?

MicroPython and Python are closely related but not identical. MicroPython is a streamlined implementation of Python 3, designed specifically for microcontrollers and embedded systems. It aims to maintain as much compatibility with standard Python as possible, meaning most Python code can run on MicroPython with minimal modifications.

However, due to memory and processing constraints on microcontrollers, MicroPython omits some of Python's features and libraries, focusing instead on core functionalities essential for embedded applications. Additionally, MicroPython includes specialized libraries and modules to interact with hardware, which are not found in standard Python.

Therefore, while MicroPython shares Python's syntax and many of its capabilities, it is tailored to the unique requirements of low-resource environments.

---

## How long does it take to learn MicroPython?

The time it takes to learn MicroPython largely depends on your prior experience with programming and familiarity with Python. If you're already versed in Python, you could get comfortable with MicroPython in a matter of days, as it retains much of the syntax and structure of Python.

For those new to programming or coming from different programming languages, it might take a few weeks to grasp the basics. The learning curve is relatively gentle due to Python's clear syntax and MicroPython's extensive documentation and supportive community.

Additionally, hands-on experimentation with microcontrollers can significantly accelerate the learning process, making it possible to create simple projects within a few hours of initial learning.

---

## The history of MicroPython

`MicroPython`, a lean and efficient implementation of Python 3, was created by Damien George in 2013 to bring Python's ease of use to microcontrollers and small embedded systems. It was initially funded through a successful Kickstarter campaign, reflecting the growing interest in combining Python's simplicity with the world of embedded hardware. 

MicroPython's design focuses on minimal resource usage while maintaining Python's hallmark readability and functionality. Over time, it has gained popularity in the maker community and is now used in various applications, from educational tools to industrial equipment. This success is due in part to its wide support for different hardware platforms and its active, open-source community, which continually enhances its capabilities and reach.

---

## Where can I get MicroPython?

MicroPython is available from the offiial website: <https://micropython.org/>

It is available for a wide range of microcontrollers, including the Raspberry Pi Pico, ESP32, the BBC Micro:bit, and the Adafruit Circuit Playground Express.

---

## Where can I learn MicroPython?

We have a free course on MicroPython, which you can access here:

{% include card.html cardtitle="Learn Micropython - the basics" url="/learn/micropython/00_intro.html" img="/learn/micropython/assets/micropython.jpg" %}