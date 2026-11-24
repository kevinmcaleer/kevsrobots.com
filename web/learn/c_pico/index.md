---
layout: course_overview
title: Getting Started with C on the Raspberry Pi Pico
author: Kevin McAleer
cover: /learn/c_pico/assets/cover.jpg
date: 2025-06-16
description: Learn how to program the Raspberry Pi Pico using the C programming language.
  This beginner-friendly course covers the fundamentals of C, how to set up your development
  environment, and how to write simple programs that run on real hardware. By the
  end of the course, you'll have a solid grasp of programming basics and will create
  your first hardware project; a blinking LED.
level: beginner
course_duration: 0h 50m
lesson_count: 10
first_lesson: 00_intro.html
quiz_total: 30
outcomes:
- Read and write small C programs using variables, conditions, loops and functions
- Install the Raspberry Pi Pico C/C++ SDK and build an example program into a .uf2
  file
- Write a CMakeLists.txt file for your own Pico project
- Print messages from the Pico to your computer over USB serial
- Control a GPIO pin as an output from C
- Wire up and blink an external LED, and explain why it needs a resistor
prerequisites:
- No programming or electronics experience is needed
- Comfort typing commands in a terminal helps, such as Introduction to the Linux Command
  Line on Raspberry Pi OS
you_need:
- A Raspberry Pi Pico, Pico W or Pico 2 (with headers soldered for the breadboard
  project)
- A micro-USB cable for Pico and Pico W, or a USB-C cable for Pico 2 (data cable,
  not charge-only)
- A computer running Windows, macOS or Linux
- A breadboard, one LED, one 330 ohm resistor and two jumper wires
- Lessons 2 to 5 need only a computer with a C compiler such as gcc
navigation:
- name: Getting Started with C on the Raspberry Pi Pico
- content:
  - section: Introduction
    content:
    - name: Introduction to Programming the Raspberry Pi Pico in C
      link: 00_intro.html
  - section: Programming Fundamentals
    content:
    - name: What is C?
      link: 02_what_is_c.html
    - name: Variables and Data Types
      link: 03_variables_and_types.html
    - name: Conditionals and Loops
      link: 04_conditionals_and_loops.html
    - name: Functions in C
      link: 05_functions.html
  - section: Raspberry Pi Pico Setup
    content:
    - name: Setting Up the Toolchain
      link: 06_setting_up_toolchain.html
    - name: Your First Program in C
      link: 07_first_program.html
  - section: GPIO Basics
    content:
    - name: GPIO Basics
      link: 08_gpio_basics.html
    - name: Blinking an LED
      link: 09_blinking_led.html
  - section: Summary and Next Steps
    content:
    - name: Summary and Next Steps
      link: 10_summary.html
---
