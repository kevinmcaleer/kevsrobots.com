---
layout: lesson
title: Enhancing Temperature Readings
author: Kevin McAleer
type: page
cover: /learn/pico_temp_sensor/assets/banners/08_formatting_temperature_readings.jpg
date: 2024-11-22
previous: 07_displaying_temperature_in_thonny_console.html
next: 09_converting_adc_to_temperature.html
description: Improve the readability of temperature readings by formatting them with
  custom labels and adding visual enhancements.
percent: 56
duration: 5
quiz_total: 36
date_updated: 2026-11-24
navigation:
- name: Using the Raspberry Pi Pico's Built-in Temperature Sensor
- content:
  - section: Introduction to the Raspberry Pi Pico and MicroPython
    content:
    - name: Course Introduction
      link: 00_intro.html
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
    - name: Getting More Accurate Temperature Readings
      link: 09_converting_adc_to_temperature.html
    - name: Creating a Temperature Alert System
      link: 10_creating_a_temperature_alert_system.html
  - section: Project Challenges and Next Steps
    content:
    - name: Project Ideas and Extensions
      link: 11_project_ideas_and_extensions.html
    - name: Troubleshooting Temperature Readings
      link: 12_troubleshooting_temperature_readings.html
    - name: Summary and Next Steps
      link: 13_summary_and_next_steps.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to label readings with a status such as Low, Normal or High using `if`, `elif` and `else`, and separate readings with a divider line.

Now that the Pico can tell us the temperature, let's make it tell us what it *means*. Instead of "29.8", we want "29.8, Normal". Adding a status is the first step towards a machine that makes decisions, and it uses one of the most important ideas in programming: choosing between options.

```mermaid
flowchart TD
    A["temperature"] --> B{"above 30?"}
    B -- yes --> C["status = High"]
    B -- no --> D{"below 20?"}
    D -- yes --> E["status = Low"]
    D -- no --> F["status = Normal"]
```

---

## Step 1: Adding visual separators

To make the output easier to read, add a simple separator line. `"-" * 30` repeats the dash 30 times:

```python
print("-" * 30)
```
{: .no-snakie }

Here is the whole program with the separator:

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

    # Get the current time (a tuple in MicroPython)
    timestamp = time.localtime()

    # Print formatted output
    print(f"Time: {timestamp[3]:02}:{timestamp[4]:02}:{timestamp[5]:02}")
    print(f"Temperature: {temperature:.2f}°C")
    print("-" * 30)
    time.sleep(1)
```

## Step 2: Conditional formatting for temperature ranges

You can describe specific temperature ranges by adding a status to the output. Python checks each condition from the top and uses the first one that is true:

```python
if temperature > 30:
    status = "High"
elif temperature < 20:
    status = "Low"
else:
    status = "Normal"
```
{: .no-snakie }

Here is the complete script with the status added:

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

    # Determine the status
    if temperature > 30:
        status = "High"
    elif temperature < 20:
        status = "Low"
    else:
        status = "Normal"

    # Get the current time (a tuple in MicroPython)
    timestamp = time.localtime()

    # Print formatted output with status
    print(f"Time: {timestamp[3]:02}:{timestamp[4]:02}:{timestamp[5]:02}")
    print(f"Temperature: {temperature:.2f}°C - Status: {status}")
    print("-" * 30)
    time.sleep(1)
```

## Step 3: Testing the enhanced script

1. Save the updated script as `enhanced_temp_readings.py`.
2. Run the script in Thonny.
3. Watch how the status changes with the temperature.

## Example output

```raw
Time: 15:34:21
Temperature: 29.80°C - Status: Normal
------------------------------
Time: 15:34:22
Temperature: 30.27°C - Status: High
------------------------------
```

Tip: the chip warms up quickly if you hold a finger on it, which is an easy way to see the status change to **High**.

**Why this matters:** a status word is what lets another part of the program, or a person glancing at a screen, act without doing any maths.

**Common mistakes:**

- Writing `if temperature > 30:` without the colon, or with the wrong indentation, which raises a `SyntaxError` or `IndentationError`.
- Using `=` where you meant a comparison. `=` stores a value, while `>` and `<` compare.
- Putting the `else` branch before the others. Python checks from the top, so the order matters.

## What you've learned

- How to use separators and labels for better readability.
- How to use `if`, `elif` and `else` to choose a status.
- How to present clean, user-friendly output.

---

## Try it Yourself

1. Change the limits to 28 and 22 so the status flips more easily in your room.
2. Add a fourth status, `"Hot"`, for readings above 35, using another `elif` in the right place.
3. Challenge: make the separator line change character with the status, for example `"!" * 30` when it is High.

## Common Issues

- **Problem**: `IndentationError: unexpected indent` or `expected an indented block`.
- **Solution**: Line up the statements under each `if`, `elif` and `else` with four spaces.
- **Why**: Python uses indentation, not brackets, to decide which lines belong to which branch.

- **Problem**: The status is always `Normal`, even when you warm the chip.
- **Solution**: Print the temperature and compare it with your limits, then adjust the limits. The chip may simply not have reached 30.
- **Why**: Many rooms are cooler than the default limits, and the chip is only a few degrees above the air.

- **Problem**: `NameError: name 'status' isn't defined`.
- **Solution**: Make sure every branch, including `else`, sets `status` before you print it.
- **Why**: A variable only exists once something has given it a value.

## Check your understanding

{% quiz %}
? What does `"-" * 30` produce?
- [ ] The number negative 30
- [x] A line of 30 dash characters
- [ ] An error
> Multiplying a string repeats it, so this gives 30 dashes in a row.

? A reading is 25°C and the code uses `> 30`, `< 20`, `else`. What is the status?
- [ ] High
- [ ] Low
- [x] Normal
> 25 is not above 30 and not below 20, so it falls through to `else`.

? Why does the order of `if` and `elif` branches matter?
- [ ] It does not matter
- [x] Python uses the first condition that is true
- [ ] Python always runs the last branch
> Once a branch matches, Python skips the rest, so an earlier, broader condition can hide a later one.
{% endquiz %}
