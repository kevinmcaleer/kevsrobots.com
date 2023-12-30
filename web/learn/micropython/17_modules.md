---
layout: lesson
title: Modules
author: Kevin McAleer
type: page
cover: /learn/micropython/assets/micropython.jpg
date: 2022-12-04
previous: 16_repl.html
next: 18_summary.html
description: Learn about the built-in Modules
percent: 95
duration: 3
navigation:
- name: Learn MicroPython - The basics
- content:
  - section: Overview
    content:
    - name: Introduction
      link: 00_intro.html
    - name: Learn MicroPython Introduction Video
      link: 00_videos.html
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
  - section: Summary and Review
    content:
    - name: Summary and Review
      link: 18_summary.html
---


![Picture of a honeycomb close up](assets/modules.jpg){:.cover}

## Modules

MicroPython has many modules that are built into the firmware. We still need to `import` these if we want to use them in our code.

This is because when we `import` a module MicroPython has to read all the lines of code in that module and store the functions in `RAM` (Random Access Memory). MicroPython devices have limited `RAM` so we need to ensure we only import the modules we need for each specific program.

Lets have alook at all the modules on our MicroPython device. 

##### Example

1. Type the following directly into the `REPL`:

```python
help('modules')
```

![Screenshot of all the installed Modules](assets/modules.png){:class="img-fluid w-100 shadow-lg"}

A screenshot of all the installed Modules
{:.caption}

---

You'll notice that most of the modules have `u` as the first letter, this is because they are the simplified MicroPython version of standard Python libraries (in some cases). The `u` is an ascii version of the Mu or `μ` symbol which is the Greek letter for Micro.

A module is just another MicroPython program, one that just contains other functions. Modules tend to by collections of related functions. 

## Table of Built-in Modules
Below is a table of the most commonly used modules:

Module     | Alias     | Description
-----------|-----------|-------------------------------------------------------------------------------------------------------------------------
`math`     | `cmath`   | contains functions such as `radians`, `sin`, `cos`, `sqrt`, `tan`
`dht`      | -         | a class for use with the `dht` range of temperature sensors
`framebuf` | -         | the Frame Buffer class - useful for creating graphics and working with displays
`gc`       | -         | the Garbage Collector - helps keep memory unfragmented
`neopixel` | -         | a class for use with Adafruit Neopixels, or any APA102 and WS1218 RGB LED Strips
`network`  | -         | you'll use this a lot if you're working with Wi-Fi, webservers and APIs
`umachine` | `machine` | contains lots of board specific functions and constants
`utime`    | `time`    | you'll use this to access the `sleep` function which is used in nearly all physcial computing projects, such as robotics
{:class="table"}

This isn't an extensive list, and each variety of board may have a slightly differnt list of modules.

---

## DIR

To investigate each module you can use the `dir()` function:

##### Example

From the `REPL` type:
```python
dir(machine)
```

In the example above you'll discover a list of all the functions, classes and Constants in the module named `machine`.

---
