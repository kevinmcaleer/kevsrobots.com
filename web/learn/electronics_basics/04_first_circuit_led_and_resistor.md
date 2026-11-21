---
layout: lesson
title: 'Your First Circuit: LED and Resistor'
author: Kevin McAleer
type: page
cover: /learn/electronics_basics/assets/banners/04_first_circuit_led_and_resistor.jpg
date: 2026-11-21
previous: 03_the_breadboard.html
next: 05_multimeter_measuring_voltage.html
description: 'Build the first stage of the night light: an LED on a breadboard, with
  the resistor value chosen using Ohm''s law.'
percent: 30
duration: 5
date_updated: 2026-11-21
navigation:
- name: Electronics basics
- content:
  - section: Introduction
    content:
    - name: Electronics basics
      link: 00_intro.html
  - section: The basics
    content:
    - name: Voltage, Current and Resistance
      link: 01_voltage_current_resistance.html
    - name: Ohm's Law
      link: 02_ohms_law.html
    - name: The Breadboard
      link: 03_the_breadboard.html
    - name: 'Your First Circuit: LED and Resistor'
      link: 04_first_circuit_led_and_resistor.html
  - section: Measuring
    content:
    - name: 'Multimeter: Measuring Voltage'
      link: 05_multimeter_measuring_voltage.html
    - name: 'Multimeter: Current and Resistance'
      link: 06_multimeter_current_and_resistance.html
  - section: Building blocks
    content:
    - name: Series and Parallel Circuits
      link: 07_series_and_parallel.html
    - name: Voltage Dividers
      link: 08_voltage_dividers.html
    - name: Reading Datasheets
      link: 09_reading_datasheets.html
    - name: Switches and Pull Resistors
      link: 10_switches_and_pull_resistors.html
    - name: Transistors as Switches
      link: 11_transistors_as_switches.html
    - name: Powering Your Projects
      link: 12_powering_your_projects.html
  - section: Into the Pico
    content:
    - name: Connecting to the Pico
      link: 13_connecting_to_the_pico.html
  - section: Project
    content:
    - name: 'Project: Pico Night Light'
      link: 14_project_night_light.html
    - name: Summary
      link: 15_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

## Overview

Time to light something up. In this lesson we build stage one of our **night light**: a battery pack, a resistor and an LED. The star of the lesson is choosing the resistor, which brings Ohm's law to life.

## Why an LED needs a resistor

An LED (light emitting diode) is not like a bulb. Once it has enough voltage to turn on, its own resistance drops almost to nothing. If you connect it straight to a battery, the current rises until the LED or the battery gives up, usually the LED. A **current limiting resistor** in series keeps the current at a safe level.

An LED also drops a fixed voltage when it is on, called its **forward voltage (Vf)**. A red LED is about 2.0 V. The resistor has to take up the rest of the supply voltage.

## Choosing the resistor

The recipe, using Ohm's law on the resistor alone:

```text
resistor voltage = supply voltage - LED forward voltage
R = resistor voltage / wanted current
```

Our numbers: a 4.5 V pack, a red LED (Vf about 2.0 V) and 10 mA (a nice visible glow that is safe for most LEDs).

```text
resistor voltage = 4.5 - 2.0 = 2.5 V
R = 2.5 / 0.010 = 250 ohm
```

250 ohm is not a standard value. Resistors come in a series called **E12**: 10, 12, 15, 18, 22, 27, 33, 39, 47, 56, 68, 82, then times ten, a hundred and so on. We pick the next value up, which gives slightly less current, which is the safe direction: **270 ohm**.

Let the computer do the picking:

```python
# Run on your computer or the Pico
# Choose a current limiting resistor for an LED

E12 = (10, 12, 15, 18, 22, 27, 33, 39, 47, 56, 68, 82)

def next_e12(ohms):
    """Smallest standard E12 value that is at least `ohms`."""
    decade = 1
    while decade * 10 * 10 < ohms:   # find a decade near the answer
        decade *= 10
    for d in (decade, decade * 10, decade * 100):
        for base in E12:
            if base * d >= ohms:
                return base * d

def led_resistor(supply, vf, milliamps):
    wanted = (supply - vf) / (milliamps / 1000)
    chosen = next_e12(wanted)
    actual_ma = (supply - vf) / chosen * 1000
    return wanted, chosen, actual_ma

for supply in (4.5, 3.3):
    wanted, chosen, actual = led_resistor(supply, 2.0, 10)
    print(f"{supply} V supply: need {wanted:.0f} ohm -> use {chosen} ohm -> {actual:.1f} mA")
```

Output:

```text
4.5 V supply: need 250 ohm -> use 270 ohm -> 9.3 mA
3.3 V supply: need 130 ohm -> use 150 ohm -> 8.7 mA
```

The second line is the 3.3 V case, which we will use when the Pico takes over.

## Build it

You need: breadboard, 3 x AA battery pack, one red LED, one 270 ohm resistor (red, violet, brown) and two jumpers.

1. Connect the battery pack to the power rails (red to +, black to -). Leave the battery switch off, or one lead out, while you build.
2. Push the LED into the board so its legs are in two different rows. Long leg to the row on the left, say row 10.
3. Place the resistor from the LED's long-leg row (row 10) to the positive rail, using a jumper if needed.
4. Connect a jumper from the short-leg row to the negative rail.
5. Connect the battery. The LED should glow.

The path is: battery positive, resistor, LED, battery negative. Order of resistor and LED does not matter, the current is the same everywhere in a loop.

Safety: never connect the battery's positive directly to negative with a bare wire. That is a short circuit. The wire and battery heat up quickly and can burn you. If anything feels warm, disconnect it.

**Why this matters:** this exact pattern, supply, resistor, LED, ground, is on nearly every board you will ever buy, from status LEDs on your Pico to the lights in a robot's eyes.

## Try it Yourself

1. Swap the 270 ohm resistor for 1k. What happens to the brightness? Use the script to predict the current first.
2. Try a green LED and a blue LED. Look up their forward voltage (we explain how in lesson 9), and rerun the script.
3. Challenge: use the script to find a resistor for a white LED (Vf about 3.0 V) on 4.5 V at 10 mA.

## Common Issues

**Problem**: The LED does not light.
**Solution**: Flip the LED around.
**Why**: LEDs only conduct one way. Long leg to positive, short leg to negative.

**Problem**: The LED is very dim.
**Solution**: Check the resistor value. A brown, black, red resistor is 1k, not 100 ohm. Also check the battery is fresh.
**Why**: Ten times the resistance means about a tenth of the current.

**Problem**: The LED flashed brightly and died.
**Solution**: Replace it, and check the resistor is actually in the loop.
**Why**: Without a resistor, the current is limited only by the battery.
