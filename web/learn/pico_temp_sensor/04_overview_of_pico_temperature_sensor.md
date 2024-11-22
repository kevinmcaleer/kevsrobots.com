---
layout: lesson
title: "Understanding the Pico\u2019s Temperature Sensor"
author: Kevin McAleer
type: page
cover: assets/temperature_sensor_intro.png
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
    - name: Converting ADC Values to Temperature
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


## Introduction

The Raspberry Pi Pico comes with a built-in temperature sensor that you can access using its ADC (Analog-to-Digital Converter). This lesson explains how the sensor works and prepares you to use it in your projects.

---

## How the Temperature Sensor Works

1. The temperature sensor is integrated into the Pico’s RP2040 chip.
2. It measures the temperature of the chip itself and converts the reading into an ADC value.
3. The ADC value can be translated into a temperature in degrees Celsius using a formula.

---

## What You Will Learn

- The basics of the Pico’s ADC.
- How to read raw data from the temperature sensor.
- How to convert ADC values into human-readable temperatures.

---

## Understanding ADC

- The Pico’s ADC converts analog signals (like temperature) into digital values.
- ADC values range from 0 to 65535 on the Pico.
- The temperature sensor’s output can be processed using a specific formula.

---

## Preparing for the Next Lesson

In the next lesson, we’ll write a script to read data from the temperature sensor and display it in the Thonny console. Make sure your Pico is set up with MicroPython and ready to go!

---

