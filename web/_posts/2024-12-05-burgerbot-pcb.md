---
title: "Making a Custom PCB for BurgerBot"
description: >-
    Lets give BurgerBot v3 a custom PCB to make it work better with the pen holder
excerpt: >-
    Creating a custom PCB for BurgerBot has been on my bucket list for at least a year, and last Sunday the audience voted for a mid-week stream
layout: showcase
date: 2024-12-05
date_updated: 2024-12-15
author: Kevin McAleer
difficulty: beginner
cover: /projects/burgerbot/assets/img/cover.jpg
hero:  /projects/burgerbot/assets/img/hero.png
mode: light
tags:
 - pcb
groups:
 - pcb
 - robots
 - electronics
videos:
 - C_IUvI7KuP4
 - Ysi1OJqzwD0
---

## Why make a custom PCB?

Creating a custom PCB for BurgerBot has been on my bucket list for at least a year, and last Sunday the audience voted for a mid-week stream. I took this opportinity to design a PCB for BurgerBot v3 live on the stream.

---

## Design Goals

The goals for the PCB design are:

- Fix the issue with the pen holder server (the previous version didn't have power to the servo when on battery - I know, I know...)
- 5v Power for Servo (pen up and down); its actually 6v because 4x AA batteries will power it
- Option for encoders on motors, or just regular motors ( + / - )
- connectors for motors & power (possbly JST style connectors or just header pin style)
- Ultrasonic range finder connectivity (either through the hole or via a female header pin)
- Raspberry Pi Pico Powered (drop in a Pico H, Pico WH, Pico 2 H or Pico 2 WH)

---

## Design

I've detailed the design in this [project](/projects/burgerbot) page, which is a new style of project page for the site; it separates out the overview, bill of materials, circuit diagram, wiring, assembly, code and downloadable STL files into separate pages.

---

If you find this interetsing let me know and I'll do more of these in the future. I'm also considering doing a series of PCB design streams, where we design a PCB for a project live on stream.

---
