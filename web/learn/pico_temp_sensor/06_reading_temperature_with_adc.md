---
layout: lesson
title: Converting ADC Values to Temperature
author: Kevin McAleer
type: page
cover: /learn/pico_temp_sensor/assets/cover.jpg
date: 2024-11-22
previous: 05_how_the_temperature_sensor_works.html
next: 07_displaying_temperature_in_thonny_console.html
description: Learn how to convert raw ADC values from the temperature sensor into
  human-readable temperatures in degrees Celsius.
percent: 48
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

Raw ADC values aren’t particularly useful on their own. This lesson will show you how to use a formula to convert the ADC value into a temperature in degrees Celsius.

---

## The Conversion Formula

The Pico’s temperature sensor uses the following formula to calculate temperature in degrees Celsius:

```python
temperature = 27 - (raw_value - 0.706) / 0.001721
```

## Step 1: Updating Your Script
Modify your previous script to include the temperature calculation:

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

    print(f"Temperature: {temperature:.2f}°C")
    time.sleep(1)
```

---

### Step 2: Running the Script

1. Save the updated script as read_temp_with_conversion.py.
1. Run the script in Thonny.
1. Observe the temperature readings displayed in degrees Celsius.

---

### What You’ve Learned

* How to use the conversion formula to calculate temperature.
* How to display formatted output in the console.

In the next lesson, you’ll learn how to format and enhance the output for better readability.

---
