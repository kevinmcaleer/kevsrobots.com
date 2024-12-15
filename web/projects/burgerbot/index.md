---
title: "BurgerBot"
description: >-
    An open-source, burger-shared robot you can build yourself
excerpt: >-
    
layout: multipage
date: 2024-12-02
author: Kevin McAleer
difficulty: beginner
cover: /assets/img/burgerbot/cover.jpg
hero:  /assets/img/burgerbot/hero.png
mode: light
tags:
 - robots
 - burgerbot
groups:
 - robots
videos:
 - C_IUvI7KuP4
 - Ysi1OJqzwD0
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

cover: assets/img/cover.jpg
hero: assets/img/hero.png
---

## What is BurgerBot?

BurgerBot is a simple 3d printable robot you can build yourself. It's a flexible design that can be used for a variety of applcations including, simple follow or avoidance, line following, and even drawing with the addition of the pen module.

{% include hero_left.html image="assets/img/burgerbot.jpg" title="The 3d printable robot you can build yourself" content="BurgerBot starts with a simple base." %}

---

## Evolution of BurgerBot

BurgerBot has evolved over the years, from a simple robot with a Raspberry Pi Pico, to a more complex robot with a Raspberry Pi Pico 2 W and a custom PCB.

![evolution](assets/img/evolution01.png){:class="w-100 rounded-3"}

The ltest version of BurgerBot is based on a custom PCB (Printed Circuit Board) that makes it easier to build and more reliable.

For the best results use a Pico H, Pico WH, Pico 2 H, or Pico 2 WH as these have the header pins already soldered in place, and include a small plastic spaceer to keep the PCB off the surface of the robot. This makes it easier to connect a USB cable to the Pico for programming.

![evolution](assets/img/wiring02.png){:class="w-100 rounded-3"}

---

## Features

BurgerBot is made up of a base, a front face for the Rangefinder, and a back section that can house the battery pack. The raspberry Pi can either be mounted on top (for the non-drawing version), or inside the robot (for the drawing version).

- 3d printable
- Raspberry Pi Pico powered
- 2x DC motors
- 1x Servo motor (for the drawing version)
- 1x Ultrasonic sensor
- Bluetooth and Wifi connectivity for the Raspberry Pi Pico W & Pico 2 W versions

{% include hero_right.html image="assets/img/base.png" title="Simple and quick to print" content="All the components are designed to be quick to print" button="Download STLs" link="stl" button_colour="ocre" %}

---

{% include centered_nav.html next="Bill of Materials" next_link="bom.html" %}

---
