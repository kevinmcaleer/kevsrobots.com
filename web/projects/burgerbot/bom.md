---
title: "Bill of Materials"
description: >-
    An open-source, burger-shared robot you can build yourself
excerpt: >-
    
layout: multipage
date: 2024-12-04
author: Kevin McAleer
difficulty: beginner
cover: /projects/burgerbot/assets/img/cover.jpg
hero:  /projects/burgerbot/assets/img/hero.png
mode: light
tags:
 - robots
 - burgerbot
groups:
 - robots
navigation:
 - name: "Overview"
   link: index
 - name: "Bill of Materials"
   link: bom
 - name: "Circuit Diagram"
   link: circuit
 - name: "Wiring"
   link: wiring
 - name: "Assembly"
   link: assembly
 - name: "Code"
   link: code
 - name: "Downloadable STL files"
   link: stl
---

Here is the Bill of Materials for the version 3 of BurgerBot. This is the base version of the robot, with no additional modules. The drawing module will be added in a later version.

Item              | Description                                                   | Quantity |  Price |  Total
------------------|---------------------------------------------------------------|:--------:|-------:|------:
Raspberry Pi Pico | Microcontroller (Raspberry Pi Pico, Pico W, Pico 2, Pico 2 W) |    1     |  $4.00 |  $4.00
DC Motor          | N20 150RPM Motors 3-6V                                        |    2     |  $2.00 |  $4.00
MX1508            | H-Bridge Motor Driver                                         |    1     |  $1.00 |  $1.00
Ultrasonic Sensor | HC-SR04P (3.3v version)                                       |    1     |  $1.00 |  $1.00
Battery Box       | 4xAA Battery Holder                                           |    1     |  $1.00 |  $1.00
Wheels            | 2x Moon buggy Wheels                                          |    1     |  $4.50 |  $4.50
M3 Sccrews        | 12x M3 10mm screws                                            |    1     |  $1.00 |  $1.00
Custom PCB        | Custom PCB for the Raspberry Pi Pico                          |    1     | $10.00 | $10.00
Total             |                                                               |          |        | $25.50
{:class="table table-striped"}

---

## Additional items you may need

Depending what you already have in your parts bin, you may need to purchase the following items:

- 4xAA Batteries
- PLA Filament
- Raspberry Pi Pico Headers
- Raspberry Pi Pico USB Cable
- SG90 Servo Motor (for the drawing version)
- Bluetooth and Wifi connectivity for the Raspberry Pi Pico W & Pico 2 W versions
- Solder, soldering flux, wire
- Header pins for the Motors, Battery input, Servo header

---

## Tools

To build the robot you will need the following tools:

Tool           | Description
---------------|---------------------------------------------------
3D Printer     | to print out the motor mounting brackets
Screwdriver    | to assemble the robot
Wire Cutters   | to cut the wires to length
Soldering Iron | to solder the wires to the motors and motor driver
Computer       | to program the Raspberry Pi Pico with MicroPython
{:class="table table-sm"}

---

{% include centered_nav.html next="Circuit Diagram" next_link="circuit.html" previous="Overview" previous_link="index.html" %}

---
