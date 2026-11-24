---
layout: lesson
title: Snakie Blocks - From Blocks to Python
author: Kevin McAleer
type: page
cover: /learn/snakie_blocks/assets/banners/00_intro.jpg
date: 2026-11-20
next: 01_the_blocks_workspace.html
description: Learn to program a Raspberry Pi Pico with drag-and-drop blocks in Snakie,
  and learn to read the MicroPython they write
percent: 7
duration: 3
quiz_total: 36
date_updated: 2026-11-24
navigation:
- name: Snakie Blocks - From Blocks to Python
- content:
  - section: Overview
    content:
    - name: Snakie Blocks - From Blocks to Python
      link: 00_intro.html
    - name: The Blocks Workspace
      link: 01_the_blocks_workspace.html
  - section: Blocks Basics
    content:
    - name: Turtle Blocks - Draw a Square
      link: 02_turtle_draw_a_square.html
    - name: Variables, Maths and Loops
      link: 03_variables_maths_and_loops.html
    - name: Making Decisions
      link: 04_making_decisions.html
  - section: Blocks and Hardware
    content:
    - name: Hardware Blocks - LEDs and Buttons
      link: 05_leds_and_buttons.html
    - name: PWM, Servos and Buzzers
      link: 06_pwm_servos_and_buzzers.html
    - name: Functions
      link: 07_functions.html
    - name: My Parts
      link: 08_my_parts.html
  - section: From Blocks to Python
    content:
    - name: Reading the Python
      link: 09_reading_the_python.html
    - name: Simple and Advanced Blocks
      link: 10_simple_and_advanced_blocks.html
  - section: Project
    content:
    - name: Project - Reaction Timer
      link: 11_project_reaction_timer.html
    - name: Summary and Next Steps
      link: 12_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

{% include youtubeplayer.html id="QVe9meIHZtk" %}

## Overview

Snakie has a **Blocks** workspace: drag-and-drop programming, a lot like Scratch, but for a real Raspberry Pi Pico. The twist is the pane next door. As you snap blocks together, Snakie writes the MicroPython beside them, line by line. That Python isn't a preview. It's the actual program that runs.

That makes blocks a brilliant way to start. You get something working in the first minute, and you learn to read Python without ever sitting through a syntax lesson. By the end of this course you'll build a reaction timer game from blocks and understand every line of the code it wrote.

Here's a blocks program you can run right now. Press **Run**, then change the `400` in either pane and watch the other one follow.

{% capture snakie_code %}
import time
from machine import Pin
pin_15 = Pin(15, Pin.OUT)
while True:
    pin_15.toggle()
    time.sleep_ms(400)
{% endcapture %}
{% include snakie.html code=snakie_code name="blink.py" view="split" %}

## Course Content

- The Blocks workspace and the split view
- Drawing with the turtle, with no hardware at all
- Variables, maths, loops and decisions
- Hardware blocks: LEDs, buttons, PWM, servos and buzzers
- Functions, and blocks for the parts on your own breadboard
- Reading the Python: hover, click a line, and Show me the Python
- Simple and advanced blocks, and the grey Python blocks
- Project: a reaction timer game

## Key Results

After this course you will be able to:

- Build working MicroPython programs from blocks
- Explain what each block writes in Python
- Control LEDs, buttons, buzzers and servos from blocks
- Edit a program in either pane, blocks or code
- Build and understand a complete reaction timer game

## What you'll need

- [Snakie](https://app.snakie.org), in your browser or the desktop app
- Nothing else for most lessons, because the built-in simulator runs the programs
- For the button lesson and the project: a Raspberry Pi Pico, a breadboard, an LED with a 330Ω resistor, a push button, a piezo buzzer and some jumper wires. The simulator can't press a button yet, so those need the real thing

## How the course works

Each lesson shows a program as blocks and as Python. Most programs are in a live Snakie box like the one above, with three views: **Code**, **Split** and **Blocks**. Run them, change them, and break them on purpose. Each box also has an **Open in Snakie** link that takes your version into the full app.

You may see a banner in a box saying *This file uses advanced blocks*. That's about the grey `import` blocks at the top of the program. You can close it, or click **Show them**. Lesson 10 explains what advanced blocks are.

Every lesson ends with things to try, fixes for common problems and a short quiz.

> **Teachers**: nearly every lesson works on a laptop or Chromebook with no hardware plugged in. Only pressing a real button, in the button lesson and the final project, needs a Pico.

## Prerequisites

If you've never opened Snakie before, start with [Getting Started with Snakie](/learn/snakie_getting_started/). It takes about an hour and shows you around the app.

Let's snap some blocks together.
