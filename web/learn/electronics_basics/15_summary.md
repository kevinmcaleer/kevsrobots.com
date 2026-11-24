---
layout: lesson
title: Summary
author: Kevin McAleer
type: page
cover: /learn/electronics_basics/assets/banners/15_summary.jpg
date: 2026-11-21
previous: 14_project_night_light.html
description: A recap of what you learned in Electronics basics, and where to go next
  with MicroPython, GPIO and robotics.
percent: 100
duration: 4
quiz_total: 48
date_updated: 2026-11-24
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

By the end of this lesson you will be able to recap the main ideas of the course and choose what to learn next.

## Overview

Well done, makers. You started with a battery and an LED, and finished with a Pico watching the light level and switching a lamp for you. Let's recap what you have in your toolbox, and look at where to go next.

Every stage of the night light follows the same sense, think, act loop:

```mermaid
flowchart LR
    S["Sense<br>LDR divider"] --> T["Think<br>Pico and hysteresis"]
    T --> A["Act<br>transistor and LED"]
    A -.-> S
```

## What you learned

- **Voltage, current and resistance** are the push, the flow and the narrowing of the pipe.
- **Ohm's law** (V = I x R) lets you work out any one from the other two, including the resistor for an LED.
- **Breadboards** connect holes in rows, with power rails along the edges, and some rails are split.
- **Multimeters** measure voltage across a part, current through it, and resistance with the power off.
- **Series and parallel** circuits share current or voltage, and make the voltage divider possible.
- **Voltage dividers** turn a changing resistance into a changing voltage, which is how an LDR senses light.
- **Datasheets** give you the numbers that keep parts alive, and colour bands decode a resistor value.
- **Pull resistors** stop inputs from floating, and debouncing stops one press from counting five times.
- **Transistors** let a small signal switch a bigger load.
- **Power** needs the right supply, a regulator where needed, a common ground, and motors need their own.
- **The Pico** runs at 3.3 V, has limited current per pin, and reads analog voltages on GP26, GP27 and GP28.

## Your night light checklist

- LDR divider read on GP26
- LED switched through a transistor on GP15
- Button override on GP14
- Hysteresis stops flicker at dusk

If it all works, you have just built a miniature robot: sense, think, act.

> **Teachers**: this makes a good end-of-unit assessment. Ask learners to demonstrate the night light, explain each stage using the sense, think, act model, and show one calculation (the LED resistor or the divider) from memory.

## What's Next

Keep the momentum going with one of these:

- [Learn MicroPython - The basics](/learn/micropython/) for the programming side of the Pico
- [Raspberry Pi Pico with MicroPython - GPIO Mastery](/learn/micropython_gpio/) for deeper work with pins, PWM, ADC, I2C and more
- [Robotics 101](/learn/robotics_101/) to see how sensors, controllers and motors form a complete robot
- [Motors and motor drivers](/learn/motors_and_drivers/) for the next step in controlling bigger loads, building on the transistor and power lessons here

## Try it Yourself

1. Redesign the night light to use a different LED colour, and recalculate the resistor.
2. Keep your breadboard layout. Next time you need a sensor, you already know the pattern.
3. Challenge: build a "night plant monitor" that does the opposite, lighting an LED when it is bright.

## Common Issues

- **Problem**: I want to go further but the next course feels big.
- **Solution**: Take the GPIO course one lesson at a time and keep your night light on the desk.
- **Why**: Everything there uses ideas from this course, and it is easier to learn with a circuit in front of you.

## Check your understanding

{% quiz %}
? What is the voltage divider's job in the night light?
- [ ] Powering the LED
- [ ] Debouncing the button
- [x] Turning a changing resistance into a changing voltage the Pico can read
> The LDR and fixed resistor convert the amount of light into a voltage for the ADC.

? A new circuit has two supplies, a Pico on USB and a battery pack for a load. What must you remember?
- [ ] Join the positives
- [ ] Use a bigger resistor
- [x] Join the grounds
> Separate positive supplies, shared ground.

? Which course builds on the transistor and power lessons for bigger loads?
- [x] Motors and motor drivers
- [ ] Robotics 101
- [ ] Learn MicroPython - The basics
> Motors need their own supply and a driver, which that course covers.
{% endquiz %}
