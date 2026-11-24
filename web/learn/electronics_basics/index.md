---
layout: course_overview
title: Electronics basics
author: Kevin McAleer
cover: /learn/electronics_basics/assets/cover.jpg
date: 2026-11-21
description: 'Learn the electronics every robot builder needs: voltage, current, resistance,
  Ohm''s law, breadboards, multimeters, voltage dividers and transistors. You will
  build a light-sensing night light on a breadboard, then hand it over to a Raspberry
  Pi Pico running MicroPython.'
level: beginner
course_duration: 1h 37m
lesson_count: 16
first_lesson: 00_intro.html
quiz_total: 48
outcomes:
- Use Ohm's law to pick the right resistor for an LED
- Build circuits on a breadboard without guessing
- Measure voltage, current and resistance with a multimeter
- Build a voltage divider and read it with the Pico ADC
- Switch a load with a transistor
- Power a project safely and understand why motors need their own supply
prerequisites:
- None. This is a good first course.
- Robotics 101 (optional, for the big picture)
- Learn MicroPython - The basics (helpful for lessons 13 to 14)
you_need:
- A solderless breadboard (half size or full size)
- Jumper wires (male to male)
- A pack of assorted LEDs
- A resistor kit (100 ohm to 100k ohm, E12 values)
- A light dependent resistor (LDR)
- A 10k potentiometer
- A push button (tactile switch)
- An NPN transistor such as a 2N2222 or BC547
- A cheap multimeter
- An AA battery pack (3 x AA with leads)
- A Raspberry Pi Pico with headers and a USB cable
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
