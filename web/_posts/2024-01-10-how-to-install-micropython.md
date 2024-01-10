---
title: How to install MicroPython
description: >- 
    Learn how to Install micropython on your microcontroller
layout: project
date: 2024-01-10
cover: /assets/img/blog/install_micropython/install_micropython.png
excerpt: >-
    Learn how to install MicroPython on your microcontroller
author: Kevin McAleer
difficulty: beginner
groups:
    - raspberrypi
    - micropython
    - pico
tags:
    - Micropython
    - Microcontrollers
    - Raspberry Pi
    - ESP32
    - ESP8266

---

## How to Install MicroPython on Your Microcontroller: A Step-by-Step Guide

[MicroPython](/blog/what-is-micropython) brings the simplicity and power of Python to microcontrollers, making it an ideal choice for a wide range of projects. Whether you’re a beginner or an experienced developer, installing MicroPython is a straightforward process.

This article will guide you through the general steps for installing MicroPython on most microcontrollers, followed by specific instructions for using Thonny, a popular Python IDE, to make the process even simpler.

## How to Install MicroPython

Here are two methods for installing MicroPython on a microcontroller:

1. [Download the MicroPython firmware for your microcontroller](#step-1-downloading-the-micropython-firmware)
1. [Connect your microcontroller to your computer](#step-2-connecting-your-microcontroller)
1. [Flash the MicroPython firmware onto the microcontroller](#step-3-flashing-the-firmware)
1. [Verify the installation](#step-4-verifying-the-installation)

Alternatively

1. [Install Thonny](#step-1-install-thonny)
1. [Open Thonny and connect your microcontroller](#step-2-open-thonny-and-connect-your-microcontroller)
1. [Choose the version](#step-3-choose-the-version)
1. [Write and run your first script](#step-5-writing-and-running-your-first-script)

---

### Downloading the MicroPython Firmware

The first step is to download the appropriate MicroPython firmware for your microcontroller. This can typically be found on the MicroPython website under the 'Downloads' section. Ensure you select the firmware that matches your specific microcontroller model.

Picture of the MicroPython download page
{:class="caption"}

![Picture of the MicroPython download page](/assets/img/blog/install_micropython/download.jpg){:class="img-fluid w-100 rounded-3 shadow-lg"}

---

### Connecting Your Microcontroller

Connect your microcontroller to your computer using a suitable USB cable. It's important that your computer recognizes the device, which might require installing drivers depending on your operating system and the microcontroller.

Connecting the USB cable
{:class="caption"}

![Connecting to USB](/assets/img/blog/install_micropython/usb.jpg){:class="img-fluid w-100 rounded-3 shadow-lg"}

---

### Flashing the Firmware

Flashing the firmware for most modern micropython boards is really simply:

1. Unplug the USB cable
1. Press and hold the `boot` (`bootsel` on some boards) button
1. Plug in the USB cable
1. Release the `boot` button
1. Drag the firmware file onto the microcontroller from Finder (on macOS) or Explorer (on Windows) - the file will end in `.uf2` which is a special firm that the microcontroller knows how to install
1. Voila, you're done!

---

### Verifying the Installation

After flashing, most microcontrollers will reboot automatically. You can verify the installation by connecting to the microcontroller's REPL (Read-Eval-Print Loop) interface, typically through a serial terminal program like PuTTY or an application like the [Arduino IDE](https://arduino.cc), or [Thonny](#installing-micropython-using-thonny).

---

## Installing MicroPython Using Thonny

Thonny is a beginner-friendly Python IDE that simplifies working with Python and MicroPython. Here’s how you can use Thonny to install MicroPython:

---

### Install Thonny

Download and install Thonny from its official website <https://thonny.org>. It's available for Windows, macOS, and Linux.

---

### Open Thonny and Connect Your Microcontroller

1. Launch Thonny
1. Before connecting your microcontroller to your computer via USB, hold down the `boot` or `bootsel` button on the microcontroller
1. Thonny automatically detects the device and displays it in the 'Device' dropdown menu at the bottom right of the screen.
1. Click `Install MicroPython...`

Installing Thonny
{:class="caption"}

![Thonny](/assets/img/blog/install_micropython/install.jpg){:class="img-fluid w-100 rounded-3 shadow-lg"}

---

### Choose the Version

1. Select the version of MicroPython you want to install (the Raspberry Pi Pico / Pico H is a good choice, in this scenario)
1. Click the `Install` button
1. The firmware will upload to the device and once its complete you can click the `Close` button

Selecting the MicroPython version
{:class="caption"}

![Thonny](/assets/img/blog/install_micropython/select.jpg){:class="img-fluid w-100 rounded-3 shadow-lg"}

---

### Writing and Running Your First Script

Once MicroPython is installed, you can write Python code directly in Thonny’s editor. To run your script on the microcontroller, simply click the 'Run' button. Thonny executes the script on the device and displays any output in the built-in shell.

Your first MicroPython program
{:class="caption"}

![Thonny](/assets/img/blog/install_micropython/program.jpg){:class="img-fluid w-100 rounded-3 shadow-lg"}

---

## Conclusion: Getting Started with MicroPython

With MicroPython installed on your microcontroller, you’re ready to explore the vast possibilities of microcontroller programming with Python. The simplicity and power of MicroPython, combined with tools like Thonny, make it accessible for everyone, from hobbyists to professionals, to create amazing projects. Whether you're building smart devices, experimenting with IoT, or learning the basics of electronics, MicroPython is an excellent starting point.

---

## Where can I learn MicroPython?

We have a free course on MicroPython, which you can access here:

{% include card.html cardtitle="Learn Micropython - the basics" url="/learn/micropython/00_intro.html" img="/learn/micropython/assets/micropython.jpg" %}
