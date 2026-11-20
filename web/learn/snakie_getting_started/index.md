---
layout: lesson
title: Getting Started with Snakie
author: Kevin McAleer
type: page
cover: /learn/snakie_getting_started/assets/banners/00_intro.jpg
date: 2026-11-20
next: 01_web_or_desktop.html
description: Meet Snakie, the robotics-first MicroPython editor, and see what you
  will build in this course
percent: 8
duration: 3
quiz_total: 20
date_updated: 2026-11-20
navigation:
- name: Getting Started with Snakie
- content:
  - section: Overview
    content:
    - name: Getting Started with Snakie
      link: 00_intro.html
    - name: Web App or Desktop App
      link: 01_web_or_desktop.html
    - name: A Tour of the Workspaces
      link: 02_tour_of_the_workspaces.html
  - section: Your First Programs
    content:
    - name: Your First Program on the Simulator
      link: 03_first_program_on_the_simulator.html
    - name: The REPL and the Console
      link: 04_the_repl_and_console.html
    - name: Seeing Your Pins
      link: 05_seeing_your_pins.html
  - section: Real Hardware
    content:
    - name: Finding Your Board
      link: 06_finding_your_board.html
    - name: Flashing MicroPython
      link: 07_flashing_micropython.html
    - name: Connect, Run and Stop
      link: 08_connect_run_stop.html
    - name: Files on Your Computer and on the Board
      link: 09_files_local_and_device.html
  - section: Project
    content:
    - name: Project - Traffic Light
      link: 10_project_traffic_light.html
    - name: Summary
      link: 11_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

## Watch the video first

Kev introduced Snakie on a livestream. Watch it to see the whole app in action, then use these lessons to try each part yourself.

{% include youtubeplayer.html id="QVe9meIHZtk" %}

---

## Overview

[Snakie](https://app.snakie.org/) is a MicroPython editor built for people who make robots. It does the things you'd expect from an editor (write code, run it on a board, browse files) and then goes further: it draws your board and the pins your code uses, it has an oscilloscope, a multimeter and a plotter built in, and it comes with a simulator so you can run MicroPython with no hardware at all.

In this course you'll get to know Snakie from the very beginning. You'll run your first programs on the simulator, then move to a real Raspberry Pi Pico: find your board, flash MicroPython onto it, connect, and build a working traffic light.

![Snakie's Code workspace running the traffic light program](/learn/snakie_getting_started/assets/snakie_code_workspace.jpg){:class="img-fluid w-100"}

## Course Content

- Choosing between the web app and the desktop app
- A tour of the four workspaces: Blocks, Code, Electronics and Build
- Writing and running your first program on the simulator
- Talking to MicroPython directly with the REPL
- Seeing the pins your code uses in the Board view
- Finding your board in the Board Finder
- Flashing MicroPython firmware
- Connecting a board, and using Run, Stop and Reset
- Working with files on your computer and on the board
- Project: a traffic light on the simulator and on a real Pico

## Key Results

By the end of this course you will be able to:

- Open Snakie in a browser or install it on your computer
- Write, run and stop a MicroPython program
- Use the REPL to try out one line at a time
- Put MicroPython onto a new board
- Save a program to a board so it runs every time the board powers up

## What you'll need

For the first half of the course, nothing but a computer. The simulator runs everything.

For the hardware lessons and the project:

- A Raspberry Pi Pico. Any of the Pico family will do; the examples use a Pico 2 W
- A USB data cable (a charge-only cable will not work)
- A breadboard, three LEDs (red, amber and green), three 330 ohm resistors and four jumper wires
- Chrome or Edge if you use the web app, because they can talk to USB boards

## How the course works

Each lesson is short and has one job. Code appears in a live Snakie box like the one below. Press **Run** to try it right here in the page, change it, and press **Open in Snakie** to carry on in the full app.

{% capture snakie_code %}
print("Hello from Snakie!")
{% endcapture %}
{% include snakie.html code=snakie_code name="hello.py" %}

Each lesson ends with a **Try it Yourself** challenge and a quick quiz. Your quiz score and progress are kept in the lesson sidebar.

## Prerequisites

None. If you have never written Python before, that's fine. When you want to learn the language itself, [Learn MicroPython - The basics](/learn/micropython/) pairs well with this course.
