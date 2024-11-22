---
title: Formatting Temperature Readings
description: Enhance your temperature monitoring script by formatting the output for better readability and user experience.
layout: lesson
type: page
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
