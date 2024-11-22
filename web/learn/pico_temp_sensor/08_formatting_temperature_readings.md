---
layout: lesson
title: Enhancing Temperature Readings
author: Kevin McAleer
type: page
cover: /learn/pico_temp_sensor/assets/cover.jpg
date: 2024-11-22
previous: 07_displaying_temperature_in_thonny_console.html
next: 09_converting_adc_to_temperature.html
description: Improve the readability of temperature readings by formatting them with
  custom labels and adding visual enhancements.
percent: 64
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

In this lesson, we’ll further enhance the readability and presentation of temperature readings by adding labels, visual separators, and conditional formatting to highlight certain temperature ranges.

---

## Step 1: Adding Visual Separators

To make the output visually appealing, you can add a simple separator line:

```python
print("-" * 30)
```

### Updated Code

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

    # Print formatted output
    print("-" * 30)
    print(f"Time: {timestamp.tm_hour:02}:{timestamp.tm_min:02}:{timestamp.tm_sec:02}")
    print(f"Temperature: {temperature:.2f}°C")
    print("-" * 30)
    time.sleep(1)
```

---

## Step 2: Conditional Formatting for Temperature Ranges

You can highlight specific temperature ranges by adding conditional formatting to the output:

```python
if temperature > 30:
    status = "High"
elif temperature < 20:
    status = "Low"
else:
    status = "Normal"
```

### Updated Code

```python
while True:
    # Read the raw ADC value
    raw_value = sensor_adc.read_u16()
    
    # Convert the ADC value to voltage
    voltage = raw_value * 3.3 / 65535

    # Convert voltage to temperature
    temperature = 27 - (voltage - 0.706) / 0.001721

    # Determine the status
    if temperature > 30:
        status = "High"
    elif temperature < 20:
        status = "Low"
    else:
        status = "Normal"

    # Get the current time
    timestamp = time.localtime()

    # Print formatted output with status
    print("-" * 30)
    print(f"Time: {timestamp.tm_hour:02}:{timestamp.tm_min:02}:{timestamp.tm_sec:02}")
    print(f"Temperature: {temperature:.2f}°C - Status: {status}")
    print("-" * 30)
    time.sleep(1)
```

---

## Step 3: Testing the Enhanced Script

1. Save the updated script as `enhanced_temp_readings.py`.
2. Run the script in Thonny.
3. Observe how the temperature readings are formatted and how the status changes based on the temperature range.

---

## Example Output

```raw
------------------------------
Time: 15:34:21
Temperature: 28.45°C - Status: Normal
------------------------------
Time: 15:34:22
Temperature: 32.10°C - Status: High
------------------------------
```

---

## What You’ve Learned

- How to use separators and labels for better readability.
- How to implement conditional formatting for dynamic feedback.
- How to present clean, user-friendly output.

---
