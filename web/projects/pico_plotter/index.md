---
title: "Pico Plotter"
description: "A simple, low-cost, 3d printable plotter using a Raspberry Pi Pico and MicroPython"
excerpt: >-
  A simple, low-cost, 3D printable plotter using a Raspberry Pi Pico and MicroPython. This project is designed to be built from scratch using basic components, making it accessible for beginners and hobbyists.
layout: showcase
date: 2025-06-25
date_updated: 2025-06-26
author: Kevin McAleer
difficulty: beginner
cover: /projects/pico_plotter/assets/cover.jpg
hero:  /projects/pico_plotter/assets/hero.png
mode: light
videos:
  - uSTXCGNlpE0
  - Xa7ocyVCKYo
  - ETFdvdC-9Oo
tags:
 - Raspberry Pi
 - pico
 - plotter
groups:
 - pico
 - micropython
code:
 - https://www.github.com/kevinmcaleer/pico_plotter
stl:
 - name: Base
   link: /projects/pico_plotter/assets/pico_plotter_base_v2.stl
 - name: Sharpie Holder
   link: /projects/pico_plotter/assets/pico_plotter_pen_holder_sharpie_v2.stl
 - name: Gear
   link: /projects/pico_plotter/assets/pico_plotter_gear.stl
 - name: Tooth Rail
   link: /projects/pico_plotter/assets/pico_plotter_tooth_rail.stl
 - name: Wheel
   link: /projects/pico_plotter/assets/pico_plotter_wheel.stl
 - name: Y Gantry
   link: /projects/pico_plotter/assets/pico_plotter_y_gantry.stl
 - name: Y Tooth Rail
   link: /projects/pico_plotter/assets/pico_plotter_y_tooth_rail.stl
 - name: Roller
   link: /projects/pico_plotter/assets/pico_plotter_roller.stl 
---

## Inspiration

This project was inspired by the desire to create a simple, low-cost plotter that can be used with a Raspberry Pi Pico. The idea is to use a stepper motor to control the movement of a pen, allowing for basic drawing and plotting tasks.

This is a 'ground-up' project, meaning that it is designed to be built from scratch using basic components, making it accessible for beginners and hobbyists. I've written the code in MicroPython, that takes G-code from other software, and converts it to stepper motor movements.

## Components

{% include gallery.html titles="Components" descriptions="Pico Plotter Parts" images="/projects/pico_plotter/assets/components.jpg" noborder=true cols=1 %}

---

## Hardware

The design is heavily inspired by the [Arduino mini cnc plotter](https://www.thingiverse.com/thing:4579436) project by Daz_projects, though I've created my own version of this design in Fusion 360, and included some enhancements to the design, such as end-stops to prevent the motors from moving too far, and a more robust design for the pen holder that works with Sharpies.

## Bill of Materials

| **Item**       | **Description**   | **Qty** | **Price** | **Total** |
|----------------|-------------------|:-------:|----------:|----------:|
| Stepper Motors | 28BYJ pack of 5   |    1    |     17.99 |     17.99 |
| Pico           | Raspberry Pi Pico |    1    |      4.00 |      4.00 |
| Dupont Cables  | Pack of 3 types   |    1    |      5.99 |      5.99 |
| Servo Helper   |                   |    1    |      6.00 |      6.00 |
| MicroSwitches  | pack of 10        |    1    |      6.99 |      6.99 |
{:class="table table-striped"}

---

## Assembly

{% include gallery.html images="assets/plotter01.jpg,assets/plotter02.jpg,assets/plotter03.jpg,assets/plotter04.jpg,assets/plotter05.jpg,assets/plotter05.jpg,assets/plotter07.jpg,assets/plotter08.jpg,assets/plotter09.jpg,assets/plotter10.jpg,assets/plotter11.jpg,assets/plotter12.jpg,assets/plotter13.jpg,assets/plotter14.jpg," cols=1 noborder=true small_title=true %}

---

## Software

This project uses 3 files of MicroPython code to control the plotter:

- `stepper.py`: This file contains the code to control the stepper motors.
- `main.py`: This is the main file that runs the plotter, taking G-code
- `gcode-interpreter.py`: This file interprets G-code commands and converts them to stepper motor movements.

All 3 files need to be copied to the pico; you can use the Thonny IDE to do this, or copy them directly to the Pico using a USB cable.  

You can find the code on [GitHub](https://www.github.com/kevinmcaleer/pico_plotter).

---

## CNCJS

CNCJS is a web-based interface for controlling CNC machines, including plotters. It provides a user-friendly interface for sending G-code commands to the Pico Plotter. You can install CNCJS on your Raspberry Pi and connect it to the Pico Plotter to control it remotely.

Our MicroPython code is designed to work with CNCJS, allowing you to send G-code commands directly from the CNCJS interface. This makes it easy to control the plotter and create drawings, and import drawings from other software.

---
