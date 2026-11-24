---
layout: course_overview
title: Intermediate level MicroPython
author: Kevin McAleer
cover: /learn/intermediate_micropython/assets/cover.png
date: 2024-07-07
description: Take your MicroPython further. Build robot code with classes, inheritance
  and decorators, organise it into modules and packages, respond to the world with
  interrupts and timers, and manage your Pico from the command line with mpremote.
  Every example is checked on MicroPython, not just standard Python.
level: intermediate
course_duration: 1h 24m
lesson_count: 16
first_lesson: 00_intro.html
quiz_total: 45
outcomes:
- Design robot code as classes, using inheritance and composition where they help
- Explain what MicroPython does differently from CPython, and write code that works
  on a Pico
- Split a project into modules and packages and install libraries with mip
- Respond to buttons with pin interrupts and run repeating jobs with timers
- Copy files to and run code on a board from the command line with mpremote
prerequisites:
- '{''The Learn MicroPython - the basics course, or equivalent'': ''variables, loops,
  functions and the machine.Pin class''}'
you_need:
- A Raspberry Pi Pico, Pico W or Pico 2 running MicroPython
- A computer with Thonny installed
- A micro USB or USB-C cable that carries data
- One push button, one LED, a 330 ohm resistor and a breadboard with jumper wires
  (lessons 11 and 12)
- A Pico W and Wi-Fi (lesson 9 only)
navigation:
- name: Intermediate level MicroPython
- content:
  - section: Introduction
    content:
    - name: Introduction to Intermediate MicroPython
      link: 00_intro.html
  - section: Object Oriented Programming
    content:
    - name: Object-Oriented Programming in MicroPython
      link: 02_oop.html
    - name: Classes and Objects
      link: 02a_classes.html
    - name: Abstraction
      link: 03_abstraction.html
    - name: Inheritance
      link: 04_inheritance.html
    - name: Encapsulation
      link: 05_encapsulation.html
    - name: Polymorphism
      link: 06_polymorphism.html
  - section: Decorators
    content:
    - name: Decorators
      link: 06a_decorators.html
  - section: Modules and Packages
    content:
    - name: Modules
      link: 07_modules.html
    - name: Packages
      link: 08_packages.html
    - name: PyPi and MIP
      link: 09_pypi.html
  - section: DocStrings
    content:
    - name: Docstrings
      link: 10_docstrings.html
  - section: Interrupts and Timers
    content:
    - name: Interrupts
      link: 11_interrupts.html
    - name: Timers
      link: 12_timers.html
  - section: MP Remote
    content:
    - name: mpremote
      link: 13_mp_remote.html
  - section: Conclusion
    content:
    - name: Summary
      link: 14_summary.html
---
