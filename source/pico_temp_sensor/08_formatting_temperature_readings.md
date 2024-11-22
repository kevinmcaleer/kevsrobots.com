---
title: Enhancing Temperature Readings
description: Improve the readability of temperature readings by formatting them with custom labels and adding visual enhancements.
layout: lesson
type: page
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
