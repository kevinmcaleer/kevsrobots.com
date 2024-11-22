---
layout: lesson
title: Formatting Temperature Readings
author: Kevin McAleer
type: page
cover: /learn/pico_temp_sensor/assets/cover.jpg
date: 2024-11-22
previous: 06_reading_temperature_with_adc.html
next: 08_formatting_temperature_readings.html
description: Enhance your temperature monitoring script by formatting the output for
  better readability and user experience.
percent: 56
duration: 1
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

Formatted output makes your script more user-friendly and visually appealing. In this lesson, you’ll format your temperature readings to include timestamps and clean text.

---

## Step 1: Adding Timestamps

1. Update your script to include timestamps:

   ```python
   import machine
   import time

   # Initialize the ADC for the temperature sensor
   sensor_adc = machine.ADC(4)

   while True:
       # Read the raw ADC value
       raw_value = sensor_adc.read_u16()
       
       # Convert the ADC value to voltage
       voltage = raw_value * 3.3 / 65535

       # Convert voltage to temperature
       temperature = 27 - (voltage - 0.706) / 0.001721

       # Get the current time
       timestamp = time.localtime()

       print(f"[{timestamp.tm_hour:02}:{timestamp.tm_min:02}:{timestamp.tm_sec:02}] Temperature: {temperature:.2f}°C")
       time.sleep(1)
   ```

---

## Step 2: Running the Script

1. Save the script as `formatted_temp.py`.
2. Run it in Thonny and observe the formatted output.

---

## What You’ll Learn

- How to include timestamps in your output.
- How to format the temperature reading for better readability.

---
