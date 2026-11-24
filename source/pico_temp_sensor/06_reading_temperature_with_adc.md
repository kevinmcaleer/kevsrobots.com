---
title: Converting ADC Values to Temperature
description: Learn how to convert raw ADC values from the temperature sensor into human-readable temperatures in degrees Celsius.
layout: lesson
type: page
cover: /learn/pico_temp_sensor/assets/banners/06_reading_temperature_with_adc.jpg
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

## Introduction

Raw ADC values aren’t particularly useful on their own. This lesson will show you how to use a formula to convert the ADC value into a temperature in degrees Celsius.

---

## The Conversion Formula

The conversion happens in two steps.

First, turn the raw reading into a voltage. `read_u16()` gives a number from 0 to 65535 that covers 0 V to 3.3 V:

```python
voltage = raw_value * 3.3 / 65535
```
{: .no-snakie }

Second, turn the voltage into a temperature with the formula from the RP2040 datasheet:

```python
temperature = 27 - (voltage - 0.706) / 0.001721
```
{: .no-snakie }

Note that the second formula uses the **voltage**, not the raw ADC value. Putting `raw_value` straight into it gives a nonsense answer of about minus eight million degrees.

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
