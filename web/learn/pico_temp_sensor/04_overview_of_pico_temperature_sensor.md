---
layout: lesson
title: "Understanding the Pico\u2019s Temperature Sensor"
author: Kevin McAleer
type: page
cover: /learn/pico_temp_sensor/assets/banners/04_overview_of_pico_temperature_sensor.jpg
date: 2024-11-22
previous: 03_installing_micropython_on_pico.html
next: 05_how_the_temperature_sensor_works.html
description: "Dive into how the Raspberry Pi Pico\u2019s built-in temperature sensor\
  \ works and how to interact with it programmatically."
percent: 32
duration: 2
navigation:
- name: Using the Raspberry Pi Pico's Built-in Temperature Sensor
- content:
  - section: Introduction to the Raspberry Pi Pico and MicroPython
    content:
    - name: Course Introduction
      link: 01_course_introduction.html
    - name: Setting Up Thonny
      link: 02_setting_up_thonny.html
    - name: Installing MicroPython on the Pico
      link: 03_installing_micropython_on_pico.html
  - section: Understanding the Built-in Temperature Sensor
    content:
    - name: "Understanding the Pico\u2019s Temperature Sensor"
      link: 04_overview_of_pico_temperature_sensor.html
    - name: Reading Temperature with the ADC
      link: 05_how_the_temperature_sensor_works.html
  - section: Writing Your First Temperature Sensor Script
    content:
    - name: Converting ADC Values to Temperature
      link: 06_reading_temperature_with_adc.html
    - name: Formatting Temperature Readings
      link: 07_displaying_temperature_in_thonny_console.html
    - name: Enhancing Temperature Readings
      link: 08_formatting_temperature_readings.html
  - section: Enhancing Your Temperature Monitoring Project
    content:
    - name: Getting More Accurate Temperature Readings
      link: 09_converting_adc_to_temperature.html
    - name: Creating a Temperature Alert System
      link: 10_creating_a_temperature_alert_system.html
  - section: Project Challenges and Next Steps
    content:
    - name: Project Ideas and Extensions
      link: 11_project_ideas_and_extensions.html
    - name: Troubleshooting Temperature Readings
      link: 12_troubleshooting_temperature_readings.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

## Introduction

The Raspberry Pi Pico comes with a built-in temperature sensor that you can access using its ADC (Analog-to-Digital Converter). This lesson explains how the sensor works and prepares you to use it in your projects.

---

## How the Temperature Sensor Works

1. The temperature sensor is built into the Pico’s microcontroller chip: the RP2040 on the Pico and Pico W, and the RP2350 on the Pico 2. There is nothing to wire up.
2. It measures the temperature of the chip itself, not the air around it. The chip warms up a little when it is busy, so readings are usually a few degrees above room temperature.
3. The sensor is a tiny diode whose voltage drops as it gets warmer: about 0.706 V at 27°C, falling by about 1.721 mV for every degree Celsius. That voltage is fed to the ADC’s fifth input, channel 4, which is why you will see `machine.ADC(4)` in the code.
4. The ADC turns that voltage into a number, and a formula turns the number back into degrees Celsius.

---

## What You Will Learn

- The basics of the Pico’s ADC.
- How to read raw data from the temperature sensor.
- How to convert ADC values into human-readable temperatures.

---

## Understanding ADC

- The Pico’s ADC converts analog voltages into digital values. Channels 0 to 2 are the pins GP26 to GP28, and channel 4 is the internal temperature sensor.
- The ADC itself is 12-bit, so it really measures 4,096 steps (0 to 4095). MicroPython’s `read_u16()` scales that up to the range 0 to 65535, so your code works the same way on boards with different ADCs.
- The full range represents 0 V to about 3.3 V. Each real 12-bit step is about 0.8 mV, which is roughly half a degree Celsius for this sensor, so expect readings to move in small jumps rather than smoothly.
- The sensor is not very precise: treat a single reading as accurate to within a few degrees. Later lessons show how to smooth and calibrate it.

---

## Preparing for the Next Lesson

In the next lesson, we’ll write a script to read data from the temperature sensor and display it in the Thonny console. Make sure your Pico is set up with MicroPython and ready to go!

---

