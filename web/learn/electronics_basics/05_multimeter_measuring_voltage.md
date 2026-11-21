---
layout: lesson
title: 'Multimeter: Measuring Voltage'
author: Kevin McAleer
type: page
cover: /learn/electronics_basics/assets/banners/05_multimeter_measuring_voltage.jpg
date: 2026-11-21
previous: 04_first_circuit_led_and_resistor.html
next: 06_multimeter_current_and_resistance.html
description: Learn to use a multimeter to measure battery and circuit voltages, and
  check the night light is getting what you think it is.
percent: 36
duration: 4
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

A multimeter is your eyes inside a circuit. It tells you what the voltage actually is, rather than what you hope it is. In this lesson we learn the dial, the probes, and how to measure DC voltage on the night light.

## Meet the meter

A basic meter has:

- A **dial** to select what to measure.
- Two **probes**: black goes in the socket marked **COM**, red goes in the socket marked **V** (often combined with ohm, written VΩmA).
- A third socket, usually marked **10A**, for big currents. Leave it alone for now.

For battery and breadboard voltages, turn the dial to **DCV** (a V with a straight line, or V with a solid and dashed line above it). The wavy V symbol is for AC mains, which we never touch in this course.

Many cheap meters have several DC ranges: 200 mV, 2 V, 20 V, 200 V. Pick the smallest range higher than what you expect. For 4.5 V, choose 20 V. An auto-ranging meter picks for you.

## Measuring voltage: probes in parallel

Voltage is measured **across** something. Touch the probes to the two points you care about, without breaking the circuit.

1. Measure the battery pack: red probe on the red rail, black on the blue rail. Three fresh AA cells read around 4.6 V to 4.8 V under no load. Do not worry if it is not exactly 4.5.
2. Measure across the resistor: probes on each end while the circuit is on. You should see about 2.5 V.
3. Measure across the LED: probes on its two legs. You should see about 2.0 V.

Now add them up: 2.5 V (resistor) plus 2.0 V (LED) is 4.5 V, which is the battery. This is a fundamental rule: the voltages around a loop add up to the supply. You have just confirmed the calculation from lesson 4 with real hardware.

## Voltages to ground

You can also measure each point against ground by keeping the black probe on the blue (negative) rail and moving the red probe around the circuit. This gives a "voltage map" of the circuit. It is how engineers debug: "I expected 3.3 V here, what do I get?"

Let's record readings and compare them with the maths:

```python
# Run on your computer or the Pico
# Compare your multimeter readings with what Ohm's law predicts
supply = 4.5
vf_led = 2.0
resistor = 270

expected_current = (supply - vf_led) / resistor
expected_resistor_volts = expected_current * resistor

print(f"Expected across resistor: {expected_resistor_volts:.2f} V")
print(f"Expected across LED:      {vf_led:.2f} V")
print(f"Expected current:         {expected_current * 1000:.1f} mA")

# Type in what you measured:
measured_resistor_volts = 2.40
difference = measured_resistor_volts - expected_resistor_volts
print(f"Difference from expected: {difference:+.2f} V")
```

Output:

```text
Expected across resistor: 2.50 V
Expected across LED:      2.00 V
Expected current:         9.3 mA
Difference from expected: -0.10 V
```

Small differences of a tenth of a volt are normal. LEDs vary, and batteries sag when loaded.

**Why this matters:** later, when the Pico reads our LDR divider, you can measure the divider with the meter and know whether a strange reading is the code or the wiring.

**Common mistakes:**

- Leaving the dial on a current or resistance setting. Always return to DCV when measuring voltage.
- Reversed probes: a negative reading just means they are swapped. Nothing breaks.
- Touching both probe tips together with one finger. Your skin adds resistance, and may give odd readings on resistance mode.

## Try it Yourself

1. Measure a fresh AA cell, then one that has been used a lot. How much lower is the old one?
2. Measure the LED voltage with a red LED, then swap in a green LED. Is it different?
3. Challenge: measure the battery with the circuit connected, then with it disconnected. The loaded voltage is slightly lower. That drop is the battery's internal resistance at work.

## Common Issues

**Problem**: The meter shows 0.00 or OL.
**Solution**: Check the dial position, the range, and that the probes touch metal, not the plastic.
**Why**: OL means "over limit" or open loop, i.e. nothing connected, or the range is too small.

**Problem**: The reading jumps around.
**Solution**: Press the probes firmly against the legs, or clip them on.
**Why**: A poor contact flickers in and out.
