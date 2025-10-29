---
title: "Pi to Pico W Bluetooth Communication"
description: >-
    Learn how to set up two-way Bluetooth communication between a Raspberry Pi and a Pico using MicroPython.
excerpt: >-
layout: showcase
date: 2024-09-08
author: Kevin McAleer
difficulty: beginner
cover: /assets/img/blog/pi_2_pico/cover.jpg
hero:  /assets/img/blog/pi_2_pico/hero.png
mode: light
tags: 
 - Raspberry Pi
 - Pico
 - Bluetooth
groups:
 - raspberrypi
 - pico
 - micropython
 - electronics
videos:
 - KYpNk5MRbI8
code:
 - https://github.com/kevinmcaleer/pi_to_pico_bluetooth
---

Lasy week I was working on [a project](/blog/two-way-bluetooth) that required two-way communication between two Raspberry Pi Picos, using MicroPython.

This week, I'll show you how to set up two-way Bluetooth communication between a Raspberry Pi and a Pico using MicroPython (or any bluetooth device capable of running MicroPython).

---

## What You'll Need

- Raspberry Pi Pico W / WH
- Raspberry Pi (I'm using a Raspberry Pi 5)

---

## Setting Up the Raspberry Pi

First, we need to set up the Raspberry Pi. I'm using a Raspberry Pi 5, but you can use any Raspberry Pi that has Bluetooth capabilities.

It's best to create virtual environments when working with Python, so we'll create a virtual environment for this project, and then activate it.

```bash
python3 -m venv venv
source venv/bin/activate
```

---

### Installing the bleak Library

We need to install the `bleak` library on the Raspberry Pi. This library allows us to communicate with Bluetooth devices.

```bash
pip install bleak
```

---

## Setting Up the Raspberry Pi Pico

Next, we need to set up the Raspberry Pi Pico. Flash the latest version of MicroPython onto the Pico - for a reminder of how to do this click here: [How to install MicroPython](/learn/how_to_install_micropython/).

---

## Setting Up the Code

Now that we have the Raspberry Pi and the Raspberry Pi Pico set up, we can start writing the code.

The repository has 4 lessons, each building on the previous one. The lessons are:

1. Lesson 1 - Basic Bluetooth Communication
2. Lesson 2 - Better Two-Way Communication
3. Lesson 3 - Temperature Sensor example

In each lesson folder there is a `pi_demo.py` file and a `pico_a.py` file. These files contain the code that runs on the Raspberry Pi and the Raspberry Pi Pico, respectively.

You can save the `pico_a.py` file to the Pico by dragging and dropping it onto the Pico drive, and if you rename that file to `main.py`, it will run automatically when the Pico is powered on.

You can find the code for this project on [GitHub]({{ page.code }}).

The lessons are used in the YouTube video thats linked at the top of this article.

---
