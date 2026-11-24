---
title: Introduction
description: What MicroPython firmware is, why a new Raspberry Pi Pico needs it, and what you will do in this course.
layout: lesson
type: page
cover: /learn/how_to_install_micropython/assets/banners/00_intro.jpg
redirect_from: /learn/how_to_install_micropython/01_intro.html
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to explain what MicroPython firmware is and why a brand new Pico needs it before it can run your Python code.

Ahoy there makers! A brand new Raspberry Pi Pico is a bit like a robot with no brain loaded yet. The chip is there, the pins are there, but it does not know a word of Python. Before we can write a single line of code, we have to copy MicroPython onto it. That is what this short course is all about, and it only takes a few minutes once you know which file to pick.

## Overview

MicroPython is a version of Python that fits on a tiny microcontroller. The program that lives on the board and understands your Python code is called **firmware**. Installing it is often called **flashing**, because it is written into the Pico's flash memory, which keeps its contents when the power is off.

You only need to do this once per board (and again if you want to update it). After that you plug the Pico in, write Python, and run it.

```mermaid
flowchart LR
    A["Find your<br>Pico model"] --> B["Pick the matching<br>UF2 firmware file"]
    B --> C["Put the Pico in<br>BOOTSEL mode"]
    C --> D["Copy the file<br>onto the Pico"]
    D --> E["Pico restarts<br>running MicroPython"]
    E --> F["Check it in<br>the REPL"]
```

## Course Content

- Which Pico do I have, and which firmware file does it need
- Installing MicroPython with Thonny
- Installing by dragging a UF2 file onto the Pico
- Checking from the REPL that it worked
- Updating MicroPython, and wiping a Pico clean
- Where to go next

## Key Results

After completing this course, you will:

- Know the difference between the Pico, Pico H, Pico W, Pico WH, Pico 2 and Pico 2 W
- Pick the right firmware file for your board from micropython.org
- Install MicroPython two different ways
- Prove from the REPL which version of MicroPython and which board you are using
- Fix the most common install problems

## What you'll need

- A Raspberry Pi Pico, Pico W, Pico 2 or Pico 2 W (headers or no headers, it makes no difference to this course)
- A micro USB cable that carries data. Some cheap cables only charge, and they will not work
- A computer running Windows, macOS or Linux
- [Thonny](https://thonny.org) (free), or the site's own Snakie IDE. See [Getting Started with Snakie](/learn/snakie_getting_started/)

## How the course works

Each lesson teaches one step. Some lessons end with **Try it Yourself** exercises, a **Common Issues** list for when things go wrong, and a short quiz to check you have understood. Commands you type into the REPL are shown in text boxes, and the one small program in the course can be run straight in the page.

You do not need any programming experience. If you have a Pico in your hand, follow along. If you do not yet, you can still read through, and the first two lessons need no hardware at all.

> **Teachers**: This course suits a first club or classroom session with Picos. Allow 20 to 30 minutes per lesson, and about an hour if every learner installs for the first time. Each learner or pair needs one Pico and one data-capable USB cable. The "Which Pico and which firmware" lesson needs no hardware. On locked-down school computers, ask IT beforehand whether USB drives can be written to and whether Thonny can be installed or run from a USB stick.

## Before you start

This is the very first course in the MicroPython learning pathway, so there is nothing to do first. When you finish, carry on with [Learn MicroPython - The basics](/learn/micropython/) and [Raspberry Pi Pico with MicroPython - GPIO Mastery](/learn/micropython_gpio/), or follow the [MicroPython Learning Pathway](/learn/learning_pathways/micropython).

---

## About the Raspberry Pi Pico

The Raspberry Pi Pico is a family of low-cost microcontroller boards from Raspberry Pi Ltd. A microcontroller is a tiny computer on a single chip, designed to run one program and talk to sensors, motors and displays. The Pico has 40 pins along its edges, 26 of which are general purpose input/output (GPIO) pins you can use for your own circuits. We cover those in [GPIO Mastery](/learn/micropython_gpio/).

The Pico can be programmed in several languages, including MicroPython, CircuitPython and C/C++. This course is about MicroPython, because it is the friendliest way to start building robots.
