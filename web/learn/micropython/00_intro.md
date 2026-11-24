---
layout: lesson
title: Introduction
author: Kevin McAleer
type: page
cover: /learn/micropython/assets/micropython.jpg
date: 2022-12-04
next: 00_videos.html
description: Get started with MicroPython, what it is, where to download it and which
  software to use to develop MicroPython code.
percent: 5
duration: 4
quiz_total: 57
date_updated: 2026-11-24
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


![Cover photo of students in a classroom](assets/cover.jpg){:class="cover"}

---

Ahoy there makers! Every robot needs a brain, and one of the friendliest ways to program that brain is MicroPython. In this course we'll go from a blank Raspberry Pi Pico to confidently writing your own programs, one small idea at a time.

If you'd rather watch than read, Kev recorded this course as a video series too. Here is the first one, and the others are in the next lesson.

{% include youtubeplayer.html id="TkuDQ-SzXwU" %}

---

## Overview

Welcome to `Learn MicroPython - The basics`. This course is for people who are new to MicroPython, and to programming in general. We'll take each concept step by step, show you ***real code*** you can run, and explain ***how*** and ***why*** it works.

Our target board is the Raspberry Pi Pico family: the Pico, Pico W, Pico 2 and Pico 2 W. Every program in the course also runs in the simulator inside [Snakie](/learn/snakie_getting_started/), so you can start learning before your board arrives.

```mermaid
flowchart LR
    A["Install MicroPython"] --> B["Pick an editor"]
    B --> C["Hello World"]
    C --> D["Variables and types"]
    D --> E["Decisions and loops"]
    E --> F["Functions and modules"]
```

## Course Content

In this course you will learn:

* What MicroPython is, why you should use it, where to download it and which editor to write it in
* Your first program: `Hello World`
* Variables, constants and the different types of value
* Reserved words
* `if`, `elif` and `else`, loops, and operators
* Functions, the REPL and modules

## Key Results

After you have completed this course you will be able to:

* Install MicroPython on a Raspberry Pi Pico and run a program on it
* Store and combine values using variables and the main data types
* Make a program choose what to do, and repeat things, with `if` and loops
* Write your own functions and import modules
* Use the REPL to test ideas and explore your board

That is the solid foundation you need before moving on to motors, sensors and robots.

## What you'll need

* A computer, tablet or phone to read the course material from
* A computer that can connect to a MicroPython board such as the [Raspberry Pi Pico](https://www.raspberrypi.com)
* A Raspberry Pi Pico, Pico W, Pico 2 or Pico 2 W
* A micro USB cable that carries data, not just power (every Pico board uses micro USB)
* An editor: [Snakie](/learn/snakie_getting_started/) (recommended) or [Thonny](https://www.thonny.org)

You don't need to know any programming already. The lessons on loops and functions use only a couple of buttons or an LED, and most of the course needs no wiring at all.

## How the course works

New keywords are introduced, explained and highlighted, like `this`, inline in the text.

Longer examples are shown in boxes you can read, copy, and often run straight in Snakie:

{% capture snakie_code %}
print('this is an example')
{% endcapture %}
{% include snakie.html code=snakie_code name="intro.py" %}

Every lesson starts with what you'll be able to do by the end, and finishes with **Try it Yourself** exercises, a **Common Issues** list and a short quiz. Your quiz score is saved in your browser and shown in the sidebar. Mistakes are normal, so check Common Issues before you decide something is broken.

Important notes are highlighted in sections like these:

> ## This is a note
>
> Important notes are highlighted like this.

> ## Another aside
>
> Asides are also highlighted, in a different colour.
{:.bg-blue}

## Where next

This is the first course in the MicroPython learning pathway. When you finish, try [GPIO Mastery](/learn/micropython_gpio/) to control LEDs and motors, [MicroPython Robotics Projects](/learn/micropython_robotics/) to build real robots, or [Intermediate MicroPython](/learn/intermediate_micropython/) for classes and packages.

> **Teachers**: this course suits learners aged about 10 and up with no coding background. Allow 20 to 30 minutes per lesson, so roughly 8 to 10 hours in total. Each learner or pair needs one Pico and a data-capable USB cable, but lessons 1, 4, 8 to 14, 16 (partly) and 17 to 18 can all be done with no hardware at all using the simulator in the Snakie web app. School computers are often locked down, so test early that you can install Thonny, or that the web app opens, and that the USB port is not blocked. Lessons 2 and 3 involve flashing firmware, which is best done once by the teacher on a shared set of boards.
