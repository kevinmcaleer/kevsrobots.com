---
layout: project
title: Iskarni Hack
description: "Ikea Head Lamp Hack with MicroPython"
difficulty: Intermediate
short_title: Iskarni Hack
short_description: "Ikea Head Lamp Hack with MicroPython"
date: 2023-04-09
author: Kevin McAleer
excerpt: >- 
   Lets hack the Ikea Iskarna Head lamp to make it programmable with MicroPython using the Pimoroni Plasma 2040 W, a sound sensor, an Adafruit NeoPixel ring and some 3d printed parts.
cover: /assets/img/blog/ikea_hack/ikea_hack.jpg
tags: 
 - Raspberry Pi
 - Pimoroni
 - Plasma Stick 2040 W
 - IKEA
 - LED Strip
 - 3D Printing
groups:
 - hacks
videos:
 - fXgVCk3UTk0
repo:
 - https://www.github.com/kevinmcaleer/chatrgb
---

## Overview

* Saw this in ikea and thought it would be a fun project to add IoT to
* Envisaged it responding to a persons voice and glowing when listening
* Could be connected to our Alf AI Assistant to bring that to life
* Also could be connected to Home Assistant and MQTT

---

## Challanges

* Need to add a microcontroller to the head, with ability to control the RGB LED Lights inside
* Unknown what kind of RGB LED is inside already
* Unknown what voltage the RGB LED light requires
* May not have enough space inside for new components - might need to 3d print some parts

---

## Bill of Materials

Item                 | Description                                    | Qty |   Cost
---------------------|------------------------------------------------|:---:|------:
Head Lamp            | ISKÄRNA by Ikea                                |  1  | £25.00
Plasma Stick 2040 W  | Pimoroni Plasma Stick 2040 W                   |  1  | £12.00
Sound sensor         | DollaTek Sensor Module Sound Sensor            |  1  |  £3.99
RGB LED Ring light   | Adafruit NeoPixel Ring - 5050 RGB LED 16 pixel |  1  |  £9.90
3d Printing filament | 3d printing filament for base                  |  1  |      -
{:class="table table-striped"}

---

## Wiring

![Wiring Diagram](/assets/img/blog/ikea_hack/ikea01.jpg){:class="img-fluid w-100 rounded-1"}

---

## Sound Sensor

![Wiring Diagram](/assets/img/blog/ikea_hack/ikea02.jpg){:class="img-fluid w-100 rounded-1"}

---

## 3D Printable STL Downloads

Here are the 2 STL files:

* [`top.stl`](/assets/stl/ikea_hack/top.stl) - The top section
* [`base.stl`](/assets/stl/ikea_hack/base.stl) - the base section

---

## MicroPython code

Follow this link for the code repository <https://www.github.com/kevinmcaleer/chatrgb>
