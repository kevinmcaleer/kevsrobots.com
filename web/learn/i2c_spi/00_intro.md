---
layout: lesson
title: Introduction to I2C and SPI Communication
author: Kevin McAleer
type: page
cover: /learn/i2c_spi/assets/banners/00_intro.jpg
date: 2026-01-15
next: 13_course-video.html
description: Learn how to make your Raspberry Pi Pico talk to sensors and devices
  using I2C and SPI protocols
percent: 7
duration: 6
quiz_total: 36
videos:
- lthSloWM3Fk
date_updated: 2026-11-24
navigation:
- name: Talking to the World - Working with I2C and SPI
- content:
  - section: Introduction
    content:
    - name: Introduction to I2C and SPI Communication
      link: 00_intro.html
    - name: 'Video: Working with I2C'
      link: 13_course-video.html
  - section: Understanding I2C
    content:
    - name: What is I2C?
      link: 01_what-is-i2c.html
    - name: I2C Hardware Setup
      link: 02_i2c-hardware-setup.html
    - name: Scanning I2C Devices
      link: 03_scanning-i2c-devices.html
    - name: Reading I2C Sensors
      link: 04_reading-i2c-sensor.html
  - section: Working with Datasheets
    content:
    - name: Demystifying Datasheets
      link: 05_demystifying-datasheets.html
    - name: BME280 Deep Dive
      link: 06_bme280-deep-dive.html
  - section: Understanding SPI
    content:
    - name: What is SPI?
      link: 07_what-is-spi.html
    - name: SPI Hardware Setup
      link: 08_spi-hardware-setup.html
    - name: SPI Sensor Example
      link: 09_spi-sensor-example.html
  - section: Choosing and Troubleshooting
    content:
    - name: I2C vs SPI - Choosing the Right Protocol
      link: 10_i2c-vs-spi.html
    - name: Common Issues and Troubleshooting
      link: 11_common-issues.html
    - name: Final Project - Environmental Monitoring Station
      link: 12_final-project.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

![Course Cover](/learn/i2c_spi/assets/cover.jpg){:class="cover"}

By the end of this lesson you will be able to say what the course covers, what kit you need, and how the lessons are laid out.

## Overview

Welcome to "Talking to the World - Working with I2C and SPI"! This course will teach you how to connect your Raspberry Pi Pico to the incredible world of sensors, displays, and other devices using two essential communication protocols: I2C and SPI.

You'll move beyond simple GPIO pin control to unlock powerful capabilities - reading temperature and humidity from sensors, communicating with displays, and connecting to a vast ecosystem of electronic components. By the end of this course, you'll confidently wire up devices, scan for addresses, interpret datasheets, and build real-world sensor projects.

## Course Content

This course covers everything you need to become proficient with I2C and SPI:

- **I2C Fundamentals** - What I2C is, how it works, and when to use it
- **I2C Hardware Setup** - Wiring sensors with pull-up resistors and proper connections
- **Device Scanning** - Finding I2C devices on the bus and understanding addresses
- **Reading Sensors** - Practical examples with the BME280 temperature/humidity/pressure sensor
- **Datasheet Demystification** - How to read and understand component datasheets
- **SPI Fundamentals** - Understanding SPI protocol and its advantages
- **SPI Hardware Setup** - Wiring and configuring SPI devices
- **Protocol Comparison** - When to use I2C vs SPI for your projects
- **Troubleshooting** - Common issues and how to solve them
- **Final Project** - Building a complete environmental monitoring system

## Key Results

After completing this course, you will be able to:

- Connect I2C sensors to your Raspberry Pi Pico with proper wiring
- Scan for I2C devices and understand hexadecimal addresses
- Read temperature, humidity, and pressure data from the BME280 sensor
- Navigate datasheets to find essential information for any I2C/SPI device
- Wire and communicate with SPI devices
- Choose the right protocol (I2C or SPI) for your project needs
- Troubleshoot common communication problems
- Build a working environmental sensor project from scratch

## What You'll Need

### Hardware

- **Raspberry Pi Pico** (or Pico W) with MicroPython installed
- **BME280 sensor module** - I2C version on a breakout board (3.3 V, or a 3.3 V/5 V board with a regulator)
- **Breadboard** - For prototyping connections
- **Jumper wires** - Male-to-male and male-to-female
- **USB cable** - To connect Pico to your computer
- **Optional, for the SPI lessons**: a MAX31855 thermocouple breakout and a K-type thermocouple
- **Optional, for the final project**: a 128x64 SSD1306 I2C OLED display

### Software

- **Thonny IDE** - For writing and uploading MicroPython code
- **MicroPython firmware** - Pre-installed on your Pico
- **USB driver** - Usually installed automatically
- **Two driver files** - `bme280.py` (we write it together in lesson 4) and `ssd1306.py` (only for the final project). Neither is built into the MicroPython firmware, so you save them onto the Pico yourself. Lesson 4 shows how.

### Knowledge Prerequisites

Before starting this course, you should:

- Understand basic MicroPython programming (variables, functions, loops)
- Know how to connect to your Raspberry Pi Pico and run code
- Have basic familiarity with GPIO pins

If you need to brush up on these topics, check out the [Introduction to MicroPython](/learn/micropython/) course first. If voltage, current and resistors are new to you, the [Electronics Basics](/learn/electronics_basics/) course covers what you need for the pull-up resistor lesson.

## How the Course Works

### Code Examples

All code examples in this course are complete, tested, and ready to run on your Raspberry Pi Pico. Code blocks look like this:

```python
from machine import I2C, Pin

# Create I2C object
i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400000)
print("I2C initialized")
```

You can copy and paste these directly into Thonny.

### Try It Yourself Sections

Most lessons include hands-on exercises to reinforce learning. These sections encourage you to modify code, experiment with parameters, and apply concepts creatively.

### Common Issues

When working with hardware, things don't always go perfectly the first time. Each lesson includes troubleshooting guidance to help you solve problems independently.

### Important Notes

Throughout the course, you'll see highlighted notes:

> **Note**: Important information that will save you time and frustration.

> **Warning**: Common mistakes to avoid.

> **Tip**: Helpful suggestions to improve your projects.

## Course Structure

This course is organized into five sections:

1. **Introduction** - What you're reading now
2. **Understanding I2C** - Deep dive into I2C protocol with practical examples
3. **Working with Datasheets** - Learn to extract what you need from technical documentation
4. **Understanding SPI** - Explore the SPI protocol and its applications
5. **Choosing and Troubleshooting** - Make informed decisions and solve problems

Each section builds on the previous one, taking you from basic concepts to working projects.

## Learning Approach

This course emphasizes hands-on learning. Rather than memorizing specifications, you'll:

- Wire real components and see them work
- Run code examples and modify them
- Read actual datasheets for components you're using
- Troubleshoot real problems you'll encounter in projects
- Build a complete project that combines everything you've learned

By the end, you'll have the confidence to pick up any I2C or SPI device datasheet, wire it correctly, and get it working in your own projects.

## What's Next

After completing this course, you'll be ready for:

- **Driving many outputs from one bus** - The [PCA9685 course](/learn/pca9685/) uses the I2C skills from this course to control 16 servos with two wires
- **Adding Displays** - Use I2C/SPI displays to visualize sensor data (Video 2 in the series)
- **WiFi Communication** - Send sensor data over the internet (Video 3 in the series)
- **Weather Station Project** - Build a complete IoT weather station (Video 6 capstone)

> **Teachers**: This course suits ages 12 and up who already write simple MicroPython. Allow about 45 minutes per lesson, and 90 minutes for the final project. Plan on one Pico, one BME280 and a breadboard per pair of learners; the SPI lessons need a MAX31855 and a thermocouple, which is the dearest part of the kit, so one per group works well. With no hardware at all, learners can still read the lessons, work through the datasheet exercises in lessons 5 and 6, and do the bus-choice exercises in lesson 10 on paper. The driver files are copied to the Pico over USB, so a school network is not needed.

## Ready to Begin?

In the next lesson, we'll dive into I2C - what it is, why it's everywhere in electronics, and how it allows multiple devices to communicate using just two wires. Let's start talking to the world!
