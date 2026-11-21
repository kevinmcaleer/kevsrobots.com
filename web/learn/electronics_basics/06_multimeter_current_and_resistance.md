---
layout: lesson
title: 'Multimeter: Current and Resistance'
author: Kevin McAleer
type: page
cover: /learn/electronics_basics/assets/banners/06_multimeter_current_and_resistance.jpg
date: 2026-11-21
previous: 05_multimeter_measuring_voltage.html
next: 07_series_and_parallel.html
description: Measure current by putting the meter in the circuit, and resistance and
  continuity with the power off.
percent: 42
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

Voltage is measured across a part. Current and resistance are different, and getting the method wrong can blow the meter's fuse. In this lesson we measure the night light's LED current, check resistor values, and use the continuity beeper to find broken connections.

## Measuring current: the meter joins the loop

Current is the flow *through* the loop, so the meter has to become part of the loop. You break the circuit and let the current run through the meter.

1. Switch off or disconnect the battery.
2. Move the red probe to the **mA** socket (or 10A for large currents, but start with mA). The black probe stays in COM.
3. Turn the dial to **DCA** or **mA** with the straight line symbol.
4. Remove the jumper that goes from the resistor to the positive rail. Touch the red probe to the rail side and the black probe to the resistor side, so current flows through the meter.
5. Connect the battery. The reading should be about 9 mA, the same number we calculated.

**The big mistake:** never measure voltage or measure a battery directly with the meter in the current setting. A current meter has almost zero resistance inside, so across a battery it is a short circuit. Most meters have a fuse that pops, and a few do not have a fuse at all. Always switch the dial and move the plug back after you finish measuring current.

## Measuring resistance: power off

To measure resistance, the meter sends out its own tiny current. That means:

- **The circuit must be switched off**, and ideally the part removed from the circuit.
- Dial on the ohm setting (the omega symbol). Pick a range above the expected value.
- Probes across the resistor. It does not matter which way round.

Try it on the 270 ohm resistor. Real resistors are within 5 percent (gold band) or 1 percent, so 270 might show 262 to 283. If you leave the resistor in a circuit with other parts, the reading is also affected by the parts in parallel, so take it out first.

## Continuity: the beeper

The diode and sound symbol is the **continuity** setting. When there is a low resistance path, under about 50 ohm, the meter beeps. Use it to:

- Check a jumper wire is not broken inside the insulation.
- Find out which breadboard holes are joined.
- Check a switch does close when you press it.

## The calculation to compare

When you measure current and voltage you can work out resistance, and compare it with the meter's resistance reading:

```python
# Run on your computer or the Pico
# Work out resistance from measured voltage and current
measured_volts = 2.45      # across the resistor, from lesson 5
measured_milliamps = 9.1   # from the meter in current mode

ohms = measured_volts / (measured_milliamps / 1000)
print(f"Resistance from V and I: {ohms:.0f} ohm")
print("Resistance from the meter should be close to 270 ohm")
```

Output:

```text
Resistance from V and I: 269 ohm
Resistance from the meter should be close to 270 ohm
```

**Why this matters:** when a robot misbehaves, you can check the current it draws against what you expect. A motor drawing far more current than it should is a clear sign something is stuck or short.

**Common mistakes:**

- Forgetting to move the red probe back from the mA socket.
- Measuring resistance with the power on. This gives a garbage reading, and could damage the meter.
- Using the 200 ohm range for a 10k resistor and seeing OL. Try a higher range.

## Try it Yourself

1. Measure the current through the LED with a 270 ohm resistor, then a 1k resistor. Do the readings match your Ohm's law predictions?
2. Measure each resistor in your kit and write the value on its bag.
3. Challenge: measure the resistance of the LDR in the light, then covered with your hand. Write both numbers down. We will use them in lesson 8.

## Common Issues

**Problem**: The meter reads 0.00 mA when the circuit is on.
**Solution**: Check the red probe is in the mA socket and the fuse is not blown.
**Why**: A meter with a blown fuse acts as an open circuit in current mode.

**Problem**: Resistance readings change when I touch the legs.
**Solution**: Hold the probes only by their insulated handles.
**Why**: Your body is a resistor too, usually several hundred kilo ohms, which adds in parallel on high value resistors.
