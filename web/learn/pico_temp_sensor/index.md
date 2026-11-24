---
layout: course_overview
title: Using the Raspberry Pi Pico's Built-in Temperature Sensor
author: Kevin McAleer
cover: /learn/pico_temp_sensor/assets/cover.jpg
date: 2024-11-22
description: Learn how to use the Raspberry Pi Pico's built-in temperature sensor
  to measure and display temperature readings  in the Thonny console. This course
  is designed for beginners and covers the fundamentals of MicroPython programming,  understanding
  the Pico's ADC, and creating your first temperature monitoring project.  Through
  step-by-step tutorials and practical examples, you'll gain the skills to confidently
  use the Pico for sensor-based projects.
level: beginner
course_duration: 1h 10m
lesson_count: 13
first_lesson: 00_intro.html
quiz_total: 36
outcomes:
- Set up a Raspberry Pi Pico with MicroPython and run programs from Thonny
- Read the Pico's built-in temperature sensor with the ADC
- Convert a raw ADC reading to voltage and then to degrees Celsius
- Smooth noisy readings by averaging and calibrate them with an offset
- Build a temperature alert that lights the on-board LED
- Diagnose the common reasons a temperature reading looks wrong
prerequisites:
- No programming experience needed
- Helpful but not required, the MicroPython course
you_need:
- A Raspberry Pi Pico, Pico W or Pico 2, and a data-capable USB cable
- A computer with Thonny installed (we do this in lesson 2)
- Optional, a room thermometer for calibration and an SSD1306 I2C OLED display
navigation:
- name: Using the Raspberry Pi Pico's Built-in Temperature Sensor
- content:
  - section: Introduction to the Raspberry Pi Pico and MicroPython
    content:
    - name: Course Introduction
      link: 00_intro.html
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
    - name: Summary and Next Steps
      link: 13_summary_and_next_steps.html
---
