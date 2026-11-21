---
layout: lesson
title: Electronics basics
author: Kevin McAleer
type: page
cover: /learn/electronics_basics/assets/banners/00_intro.jpg
date: 2026-11-21
next: 01_voltage_current_resistance.html
description: A friendly first course in electronics for robot builders, built around
  a light-sensing night light controlled by a Raspberry Pi Pico.
percent: 6
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

Every robot you will ever build is a pile of electronics with a brain on top. Motors, sensors, LEDs and the Pico itself all follow the same few rules, and once you know them, wiring stops being scary guesswork and becomes something you can reason about.

In this course we learn those rules by building one thing: a **light-sensing night light**. It starts as a handful of parts on a breadboard, an LED, a resistor and a battery pack. Lesson by lesson we add a light dependent resistor (LDR), a voltage divider, a transistor and a button, and in the last lessons we hand the whole thing to a Raspberry Pi Pico that watches the room and switches the light on when it gets dark.

You do not need any electronics experience. If you can plug a USB cable in, you can do this course.

## Course Content

- Voltage, current and resistance, explained with a water pipe
- Ohm's law and how to use it for real calculations
- Breadboards and building your first LED circuit
- Using a multimeter to measure voltage, current and resistance
- Series and parallel circuits
- Voltage dividers with potentiometers and LDRs
- Reading datasheets and resistor colour codes
- Switches, pull-up and pull-down resistors
- Transistors as switches
- Powering projects: batteries, regulators, 3.3V versus 5V, common ground
- Connecting circuits to the Pico: GPIO limits and the ADC
- The finished night light, with MicroPython code

## Key Results

After completing this course, you will:

- Pick the right resistor for an LED using Ohm's law
- Build and debug circuits on a breadboard
- Measure what a circuit is actually doing with a multimeter
- Build a voltage divider and read it with the Pico ADC
- Switch an LED with a transistor, and know why motors need more
- Power a project safely and share a ground between supplies
- Have a working night light you built and understand end to end

## What you'll need

- A solderless breadboard and a set of male to male jumper wires
- A pack of assorted LEDs and a resistor kit (E12 values, 100 ohm to 100k ohm)
- A light dependent resistor (LDR), a 10k potentiometer and a push button
- An NPN transistor such as a 2N2222 or BC547
- A cheap multimeter
- An AA battery pack (3 x AA with leads)
- A Raspberry Pi Pico with headers soldered on, and a USB cable
- A computer with [Thonny](https://thonny.org/) (or any MicroPython editor) installed

Everything here is cheap. A starter kit from most electronics shops covers nearly the whole list.

## How the course works

Each lesson introduces one idea, shows it on a real circuit, and ends with a **Try it Yourself** section and a **Common Issues** section. Do the exercises. Electronics is a hands-on skill, and a circuit you have built yourself sticks in your head far better than one you have read about.

The maths is kept gentle. Where a calculation matters, you will see it written out in plain text and as a short Python snippet. Snippets marked as "run on your computer or the Pico" use only standard Python, so you can paste them into any Python prompt or into Thonny with the Pico connected. Snippets that touch hardware are MicroPython for the Pico.

A few conventions:

- Pin numbers like **GP26** mean the Pico's GPIO numbers, not the physical pin positions. We give the physical pin too where it helps.
- Pico pins in this course avoid GP0 to GP7, which the [MicroPython Robotics](/learn/micropython_robotics/) course uses for motor drivers, so you can keep both projects on the same board later.
- Safety notes are short and practical. Nothing here is dangerous at these voltages, but batteries deserve respect, and we will tell you why.

New to the Pico? You can start right away, but the [Learn MicroPython - The basics](/learn/micropython/) course is a good companion for the later lessons. Want the big picture of robots first? Have a look at [Robotics 101](/learn/robotics_101/).

Ready? Grab your breadboard and let's build something that glows.
