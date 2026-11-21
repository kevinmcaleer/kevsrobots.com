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

       # Get the current time as a tuple:
       # (year, month, day, hour, minute, second, weekday, yearday)
       timestamp = time.localtime()
       hour = timestamp[3]
       minute = timestamp[4]
       second = timestamp[5]

       print(f"[{hour:02}:{minute:02}:{second:02}] Temperature: {temperature:.2f}°C")
       time.sleep(1)
   ```

> **MicroPython is not quite the same as Python on your computer.** On a PC, `time.localtime()` returns an object with named fields such as `tm_hour`. In MicroPython it returns a plain tuple, so `timestamp.tm_hour` fails with `AttributeError: 'tuple' object has no attribute 'tm_hour'`. Use the position in the tuple instead: `timestamp[3]` is the hour, `timestamp[4]` the minute and `timestamp[5]` the second.

### Where does the time come from?

The Pico has a clock but no battery to keep it running, so it forgets the time whenever it loses power. When you run a script from Thonny, Thonny sets the Pico’s clock from your computer as it connects, so the timestamps are correct. If the Pico runs on its own (for example from a USB power bank), the clock starts again from midnight on 1 January 2021 and the timestamps show the time since power-on instead. A Pico W can fetch the real time over Wi-Fi with the `ntptime` module.

---

## Step 2: Running the Script

1. Save the script as `formatted_temp.py`.
2. Run it in Thonny and observe the formatted output. It should look something like this:

```raw
[14:32:07] Temperature: 26.84°C
[14:32:08] Temperature: 26.84°C
[14:32:09] Temperature: 27.31°C
```

---

## What You’ll Learn

- How to include timestamps in your output.
- That `time.localtime()` returns a tuple in MicroPython, and how to pick values out of it.
- How to format the temperature reading for better readability.

---
