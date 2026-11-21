---
layout: lesson
title: Reading Datasheets
author: Kevin McAleer
type: page
cover: /learn/electronics_basics/assets/banners/09_reading_datasheets.jpg
date: 2026-11-21
previous: 08_voltage_dividers.html
next: 10_switches_and_pull_resistors.html
description: 'Learn to find the numbers that matter in a datasheet: LED forward voltage
  and current, resistor colour codes and transistor ratings.'
percent: 60
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

Every component has a datasheet: a short document from the maker that says what the part can and cannot do. They look intimidating, but you only need a handful of numbers from each. In this lesson we learn where to find them for the parts in our night light, and how to read resistor colour bands, which are a tiny datasheet printed on the part.

## What to look for

Datasheets usually have a table of **absolute maximum ratings** (do not exceed these or the part may be damaged) and a table of **typical characteristics** (what you can expect in normal use). A good rule: design around the typical values, and stay well under the maximums.

## LEDs

Search for the LED's datasheet, or the shop listing, and look for:

- **Forward voltage (Vf)**: the voltage across the LED when it is on, typically listed at a test current of 20 mA. Colours vary:
  - Red and yellow: about 1.8 to 2.1 V
  - Green: about 2.0 to 2.2 V for common ones
  - Blue and white: about 3.0 to 3.3 V
- **Forward current (If)**: the continuous current, usually 20 mA for a standard 5 mm LED. We run at about 10 mA, so there is plenty of margin, and it is plenty bright.
- **Maximum reverse voltage**: usually only 5 V. Fitting an LED backwards on a bigger supply can damage it.

Note that a blue or white LED with Vf of 3.0 V or more will barely light, or not at all, from a 3.3 V Pico pin once you add a resistor. For the Pico, red, yellow and green LEDs are the easy choice.

Let's use datasheet numbers in code:

```python
# Run on your computer or the Pico
# Resistor for an LED, straight from datasheet numbers
led_datasheet = {
    "red":    {"vf": 2.0, "max_ma": 20},
    "green":  {"vf": 2.1, "max_ma": 20},
    "blue":   {"vf": 3.1, "max_ma": 20},
}

supply = 3.3
wanted_ma = 10

for colour, data in led_datasheet.items():
    headroom = supply - data["vf"]
    if headroom < 0.5:
        print(f"{colour:<6}: only {headroom:.1f} V left for the resistor, too tight")
    else:
        ohms = headroom / (wanted_ma / 1000)
        print(f"{colour:<6}: {ohms:.0f} ohm for {wanted_ma} mA")
```

Output:

```text
red   : 130 ohm for 10 mA
green : 120 ohm for 10 mA
blue  : only 0.2 V left for the resistor, too tight
```

## Resistor colour codes

Resistors are too small for printed numbers, so they use coloured bands. For the usual 4-band resistor, read from the end *away* from the gold or silver band:

- Band 1: first digit
- Band 2: second digit
- Band 3: multiplier (how many zeros)
- Band 4: tolerance (gold is 5 percent)

The colours as digits:

```text
black 0, brown 1, red 2, orange 3, yellow 4, green 5, blue 6, violet 7, grey 8, white 9
```

Examples:

- Red, red, brown, gold: 22 x 10 = 220 ohm
- Orange, orange, brown, gold: 33 x 10 = 330 ohm
- Brown, black, orange, gold: 10 x 1000 = 10k
- Yellow, violet, red, gold: 47 x 100 = 4.7k

A tiny decoder, so you can check your reading:

```python
# Run on your computer or the Pico
# Decode a 4-band resistor colour code

DIGITS = ["black", "brown", "red", "orange", "yellow",
          "green", "blue", "violet", "grey", "white"]

def decode(band1, band2, multiplier):
    value = (DIGITS.index(band1) * 10 + DIGITS.index(band2)) * 10 ** DIGITS.index(multiplier)
    return value

for bands in [("red", "red", "brown"),
              ("orange", "orange", "brown"),
              ("brown", "black", "orange"),
              ("yellow", "violet", "red")]:
    print(" ".join(bands), "=", decode(*bands), "ohm")
```

Output:

```text
red red brown = 220 ohm
orange orange brown = 330 ohm
brown black orange = 10000 ohm
yellow violet red = 4700 ohm
```

If you can't tell which end to start from, use your multimeter. It never lies, and that is what lesson 6 was for.

## Transistors

For the night light's transistor, look for:

- **Collector current (Ic) max**: a BC547 allows about 100 mA, a 2N2222 about 600 to 800 mA depending on the maker.
- **Current gain (hFE)**: how many times the base current is multiplied. Often 100 or more, but it varies a lot, so design with the minimum.
- **Pinout**: crucial. Many 2N2222 packages are E-B-C from the flat side, while the BC547 is C-B-E. Always check the datasheet for your exact part, because getting the legs swapped means it will not work.

**Why this matters:** a datasheet lookup takes two minutes and avoids burnt parts and mystery faults. It is a great habit for every part in your robot.

## Try it Yourself

1. Look at your own LED's shop page or datasheet. Write down its Vf and maximum current.
2. Decode five resistors from your kit by eye, then check each one with the multimeter.
3. Challenge: search for your transistor's datasheet and note its pinout, maximum Ic and minimum hFE.

## Common Issues

**Problem**: I cannot find a datasheet for my kit LEDs.
**Solution**: Use the typical values by colour above, and start with a larger resistor, then reduce it until the brightness is right.
**Why**: Starting high is the safe direction.

**Problem**: The colour bands are hard to read.
**Solution**: Measure the resistor with your multimeter instead.
**Why**: Brown, red and orange look similar in poor light.
