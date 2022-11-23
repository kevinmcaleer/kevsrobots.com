---
layout: lesson
title: The REPL
author: Kevin McAleer
type: page
previous: 15_functions.html
next: 17_modules.html
description: Read Evaluation Print Loop
percent: 85
navigation:
- name: Learn MicroPython - The basics
- content:
  - section: Overview
    content:
    - name: Introduction
      link: 00_intro.html
  - section: Introduction
    content:
    - name: Why is it called MicroPython?
      link: 01_why_is_it_called_micropython.html
    - name: Where to get MicroPython
      link: 02_where_to_get_micropython.html
    - name: How to Install MicroPython
      link: 03_installing_micropython.html
    - name: Why use MicroPython?
      link: 04_why_use_micropython.html
    - name: Python Development Environments
      link: 05_ides.html
    - name: Our first program
      link: 06_our_first_program.html
    - name: Example 01
      link: 07_hello_world.html
  - section: Variables and Reserved Words
    content:
    - name: Variables and Constants
      link: 08_variables.html
    - name: Example 02
      link: 09_example02.html
    - name: Values & Variables Types
      link: 10_values_and_variable_types.html
    - name: Reserved Words
      link: 11_reserved_words.html
  - section: Controlling the Flow
    content:
    - name: If, elif, else
      link: 12_if_elif_else.html
    - name: Loops
      link: 13_loops.html
    - name: Operators
      link: 14_operators.html
  - section: Functions and Modules
    content:
    - name: Functions
      link: 15_functions.html
    - name: The REPL
      link: 16_repl.html
    - name: Modules
      link: 17_modules.html
---


## The REPL

MicroPython has an input and output console thats called the `REPL`, which stands for Read Evaluation Print Loop. This is because the `REPL` `reads` in commands, `evaluates` them, `prints` out the results to the console and then `loops` back to the reading in stage.

You will have noticed the `REPL` at the bottom of the screen, its where we see the output of all the code we've run so far.

You can type MicroPython code directly into the `REPL`, which is useful for quickly testing things or examining the attached MicroPython device.

![Screenshot of the REPL](assets/repl.png){:class="img-fluid w-100 shadow-lg"}

A screenshot of the `REPL` showing the currently installed version of MicroPython.
