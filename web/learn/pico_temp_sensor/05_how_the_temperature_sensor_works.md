---
layout: lesson
title: Reading Temperature with the ADC
author: Kevin McAleer
type: page
cover: /learn/pico_temp_sensor/assets/cover.jpg
date: 2024-11-22
previous: 04_overview_of_pico_temperature_sensor.html
next: 06_reading_temperature_with_adc.html
description: Learn how to write a Python script to read raw data from the Raspberry
  Pi Pico's built-in temperature sensor using its ADC.
percent: 40
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

In this lesson, you’ll write your first script to interact with the Raspberry Pi Pico’s temperature sensor. You’ll read raw ADC values and display them in the Thonny console.

---

## Step 1: Understanding the Code

The Pico’s ADC provides a digital representation of the temperature sensor’s reading. In Python, you can use the `machine` module to access the ADC.

---

## Step 2: Writing the Script

1. Open Thonny.
2. Type the following code:

   ```python
   import machine
   import time

   # Initialize the ADC for the temperature sensor
   sensor_adc = machine.ADC(4)

   while True:
       # Read the raw ADC value
       raw_value = sensor_adc.read_u16()
       print("Raw ADC Value:", raw_value)
       time.sleep(1)
   ```

3. Save the script to your Pico as read_temp.py.

---

### Step 3: Running the Script

1. Run the script in Thonny by pressing the green Run button.
2. Observe the raw ADC values printed in the console.

---

### What You’ll See

The raw ADC value will change depending on the temperature of the Pico’s chip. In the next lesson, you’ll learn to convert these values into meaningful temperature readings.

---
