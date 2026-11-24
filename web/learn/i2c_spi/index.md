---
layout: course_overview
title: Talking to the World - Working with I2C and SPI
author: Kevin McAleer
cover: /learn/i2c_spi/assets/cover.jpg
date: 2026-01-15
description: Master I2C and SPI communication protocols on Raspberry Pi Pico. Learn
  to connect sensors, scan for devices, read datasheets, and build real-world projects
  with practical hands-on examples using the BME280 sensor.
level: intermediate
course_duration: 2h 29m
lesson_count: 14
first_lesson: 00_intro.html
quiz_total: 36
outcomes:
- Wire an I2C sensor to a Raspberry Pi Pico, with the right pins, pull-up resistors
  and 3.3 V power
- Scan an I2C bus, read the hexadecimal addresses it returns and fix address problems
- Save a driver on the Pico and read temperature, pressure and humidity from a BME280
- Find the address, registers and data format you need in a datasheet and use them
  from MicroPython
- Wire and read an SPI device, and choose the right SPI mode, speed and chip select
- Choose between I2C and SPI for a project, and troubleshoot a bus that is not working
prerequisites:
- Basic MicroPython on the Pico, such as the Introduction to MicroPython course (variables,
  functions, loops)
- Helpful but not required, Electronics Basics for voltage, current and resistors
you_need:
- A Raspberry Pi Pico or Pico W with MicroPython installed, a USB cable and Thonny
- A BME280 sensor breakout (I2C), a breadboard and jumper wires
- Optional for the SPI lessons, a MAX31855 thermocouple breakout and a K-type thermocouple
- Optional for the final project, a 128x64 SSD1306 I2C OLED display
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
