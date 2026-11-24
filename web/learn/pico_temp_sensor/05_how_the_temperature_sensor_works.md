---
layout: lesson
title: Reading Temperature with the ADC
author: Kevin McAleer
type: page
cover: /learn/pico_temp_sensor/assets/banners/05_how_the_temperature_sensor_works.jpg
date: 2024-11-22
previous: 04_overview_of_pico_temperature_sensor.html
next: 06_reading_temperature_with_adc.html
description: Learn how to write a Python script to read raw data from the Raspberry
  Pi Pico's built-in temperature sensor using its ADC.
percent: 35
duration: 4
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

By the end of this lesson you will be able to write a MicroPython program that reads the raw value from the temperature sensor once a second and shows it in the Shell.

Let's build your first sensor script. We are not converting anything to degrees yet. First we simply listen to what the sensor says, because seeing the raw numbers makes the next lesson's maths feel much less like magic.

```mermaid
flowchart LR
    A["machine.ADC(4)"] --> B["read_u16()"]
    B --> C["print the number"]
    C --> D["sleep 1 second"]
    D --> B
```

---

## Step 1: Understanding the code

The Pico's ADC gives a digital version of the sensor's voltage. In MicroPython you reach it through the `machine` module. Two lines do the work: `machine.ADC(4)` opens channel 4, and `read_u16()` returns one reading as a number from 0 to 65535.

## Step 2: Writing the script

1. Open Thonny, with the MicroPython interpreter selected.
2. Type the following code:

   ```python
   import machine
   import time

   # Initialize the ADC for the temperature sensor (channel 4)
   sensor_adc = machine.ADC(4)

   while True:
       # Read the raw ADC value
       raw_value = sensor_adc.read_u16()
       print("Raw ADC Value:", raw_value)
       time.sleep(1)
   ```

3. Save the script to your Pico as `read_temp.py`.

## Step 3: Running the script

1. Run the script in Thonny by pressing the green Run button.
2. Watch the raw ADC values printed in the Shell, one per second.
3. The `while True:` loop runs forever. Click the red **Stop** button (or press **Ctrl+C**) to stop it.

## What you'll see

You should see numbers somewhere around 13,000 to 15,000 at room temperature, for example:

```raw
Raw ADC Value: 14018
Raw ADC Value: 14034
Raw ADC Value: 14002
```

The raw value changes with the temperature of the Pico's chip. Because the sensor's voltage *falls* as the chip warms up, the number goes **down** when the Pico gets warmer. Try holding your finger on the chip in the middle of the board and watch it drop.

**Why this matters:** every sensor you will ever connect to a Pico, from a light sensor to a joystick, is read the same way. Open an ADC, call `read_u16()`, and turn the number into something useful.

**Common mistakes:**

- Forgetting `time.sleep(1)`. The loop then prints thousands of lines a second and floods the Shell.
- Calling `sensor_adc.read()`. In MicroPython the method is `read_u16()`.
- Expecting the number to go up as the chip gets hotter. It goes down, and the next lesson explains why.

> **Teachers**: this is a good moment for learners to predict before they measure. Ask them what will happen to the number when they put a finger on the chip, then run the program. A warm finger, not ice or hot water, is all that is needed, and there is no safety risk beyond ordinary care with the USB connection.

---

## Try it Yourself

1. Change `time.sleep(1)` to `time.sleep(0.2)`. How does the output change?
2. Warm the chip with a finger for ten seconds, then let go. Write down the highest and lowest number you see.
3. Challenge: add a variable `count` that goes up by one each loop and print it beside the reading, so you can see how many readings you have taken.

## Common Issues

- **Problem**: `AttributeError: 'ADC' object has no attribute 'read'`.
- **Solution**: Use `read_u16()` instead of `read()`.
- **Why**: MicroPython's ADC returns a 16-bit value through `read_u16()`.

- **Problem**: Nothing prints and Thonny seems stuck.
- **Solution**: Press Stop, check the Shell shows the MicroPython `>>>` prompt, then run again.
- **Why**: The program may still be running from an earlier attempt, or Thonny may be set to the wrong interpreter.

- **Problem**: The number never changes at all.
- **Solution**: Check you used `machine.ADC(4)` and not another channel number.
- **Why**: Other channels read pins GP26 to GP28, which are not connected to anything.

## Check your understanding

{% quiz %}
? What does `machine.ADC(4)` open?
- [ ] Pin GP4
- [x] The internal temperature sensor channel
- [ ] The fourth LED
> Channel 4 of the ADC is the built-in sensor, not a GPIO pin.

? What range does `read_u16()` return?
- [ ] 0 to 255
- [ ] 0 to 4095
- [x] 0 to 65535
> `read_u16()` scales the 12-bit reading up to a 16-bit value.

? You warm the chip with your finger. What happens to the raw number?
- [ ] It goes up
- [x] It goes down
- [ ] It stays the same
> The sensor's voltage falls as the chip warms, so the raw reading drops.
{% endquiz %}
