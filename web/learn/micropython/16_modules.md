---
layout: lesson
title: Modules
author: Kevin McAleer
type: page
previous: 15_functions.html
description: Learn about the built-in Modules
percent: 85
navigation:
- name: Learn MicroPython
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
    - name: Variables
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
    - name: Modules
      link: 16_modules.html
---


# Modules
MicroPython has many libraries that are built into the firmware. We still need to `import` these if we want to use them in our code.

This is because when we `import` a library MicroPython has to read all the lines of code in that library and store the functions in `RAM` (Random Access Memory). MicroPython devices have limited `RAM` so we need to ensure we only import the libraries we need for each specific program.

Lets have alook at all the libraries on our MicroPython device:

##### Example
```python
help('modules')
```