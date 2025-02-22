---
layout: lesson
title: Converting ADC Values to Temperature
author: Kevin McAleer
type: page
cover: /learn/pico_temp_sensor/assets/cover.jpg
date: 2024-11-22
previous: 08_formatting_temperature_readings.html
next: 10_creating_a_temperature_alert_system.html
description: Learn how to accurately convert ADC readings from the Pico's temperature
  sensor into temperature values in degrees Celsius.
percent: 72
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

The Raspberry Pi Pico's ADC provides raw digital values that correspond to the temperature of the RP2040 chip. To make these readings meaningful, we need to convert them into temperature values in degrees Celsius. In this lesson, you’ll learn how to use a mathematical formula for this conversion.

---

## Step 1: Understanding the Conversion Formula

The built-in temperature sensor measures the voltage output of the chip's temperature sensing circuit. This voltage is then converted to an ADC value, which can be transformed into a temperature in degrees Celsius using the formula:

```python
temperature = 27 - (voltage - 0.706) / 0.001721
```

### Explanation:

- `27` is the nominal room temperature in degrees Celsius.
- `0.706` is the reference voltage at room temperature.
- `0.001721` is the voltage change per degree Celsius.

---

## Step 2: Reading and Converting ADC Values

Update your script to include the conversion formula:

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

    # Print the temperature
    print(f"Temperature: {temperature:.2f}°C")
    time.sleep(1)
```

---

## Step 3: Running the Script

1. Save the script as `convert_adc_to_temp.py`.
2. Run the script in Thonny.
3. Observe the temperature readings in the console, now displayed in degrees Celsius.

---

## Step 4: Verifying the Readings

To ensure the readings are accurate:

- Compare the readings with a thermometer in the same environment.
- Note that the Pico's sensor measures the temperature of the chip, which may be slightly higher than the ambient temperature due to heat generated by the chip.

---

## What You’ve Learned

- How to use a formula to convert ADC values to temperature.
- How to write a script that continuously displays temperature readings in degrees Celsius.

---

## Example Output

```raw
Temperature: 26.84°C
Temperature: 26.85°C
Temperature: 26.87°C
```

---
