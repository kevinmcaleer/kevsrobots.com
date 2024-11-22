---
layout: lesson
title: Creating a Temperature Alert System
author: Kevin McAleer
type: page
cover: /learn/pico_temp_sensor/assets/cover.jpg
date: 2024-11-22
previous: 09_converting_adc_to_temperature.html
next: 11_project_ideas_and_extensions.html
description: Build a simple temperature alert system that notifies you when the temperature
  exceeds or falls below a set threshold.
percent: 80
duration: 3
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

In this lesson, you'll build a practical application: a temperature alert system. This system will notify you when the temperature exceeds a maximum threshold or falls below a minimum threshold.

---

## Step 1: Defining Temperature Thresholds

To create the alert system, define the acceptable temperature range:
```python
MIN_TEMP = 20  # Minimum acceptable temperature in °C
MAX_TEMP = 30  # Maximum acceptable temperature in °C
```

---

## Step 2: Writing the Alert Logic

Update your script to check if the temperature is outside the defined range and print an appropriate alert message:
```python
import machine
import time

# Initialize the ADC for the temperature sensor
sensor_adc = machine.ADC(4)

# Define temperature thresholds
MIN_TEMP = 20  # Minimum acceptable temperature in °C
MAX_TEMP = 30  # Maximum acceptable temperature in °C

while True:
    # Read the raw ADC value
    raw_value = sensor_adc.read_u16()
    
    # Convert the ADC value to voltage
    voltage = raw_value * 3.3 / 65535

    # Convert voltage to temperature
    temperature = 27 - (voltage - 0.706) / 0.001721

    # Check temperature thresholds
    if temperature < MIN_TEMP:
        print(f"Alert: Temperature is too low! ({temperature:.2f}°C)")
    elif temperature > MAX_TEMP:
        print(f"Alert: Temperature is too high! ({temperature:.2f}°C)")
    else:
        print(f"Temperature is normal: {temperature:.2f}°C")

    # Wait for 1 second before checking again
    time.sleep(1)
```

---

## Step 3: Running the Script

1. Save the script as `temperature_alert_system.py`.
2. Run the script in Thonny.
3. Observe the alert messages in the console when the temperature exceeds or falls below the thresholds.

---

## Example Output

When the temperature is normal:
```
Temperature is normal: 25.84°C
```

When the temperature is too high:
```
Alert: Temperature is too high! (31.20°C)
```

When the temperature is too low:
```
Alert: Temperature is too low! (19.45°C)
```

---

## Step 4: Customizing Alerts

You can enhance the alert system by:
1. Adding visual alerts (e.g., blinking an LED when the temperature is out of range).
2. Sending notifications via email or messaging platforms.
3. Logging the temperature data for analysis.

---

## What You’ve Learned

- How to define and use thresholds in your temperature monitoring script.
- How to create a simple alert system for high and low temperatures.
- How to enhance your script with additional features for real-world applications.

---

In the next lesson, we’ll explore how to expand this system with more advanced features and ideas.

---
