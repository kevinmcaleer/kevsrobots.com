---
title: Motion Controlled Arm
description: >-
    Build a motion controlled robotic arm using a Raspberry Pi and a camera.
layout: showcase
date: 2024-03-04
author: Kevin McAleer
difficulty: beginner
excerpt: >-
    In this project, we will build a motion controlled robotic arm using a Raspberry Pi and a camera
cover: /assets/img/blog/motioncontrol/cover.png
hero: /assets/img/blog/motioncontrol/hero.png
mode: light
tags:
  - arm
  - robotic arm
  - raspberry pi
  - pico
  - camera
groups:
  - robots
  - raspberrypi
  - 3dprinting
vidoes:
  - 
---

## Introduction

I've wanted to produce those cool pan shots you see on high production videos for a while now, and I've been thinking about how to do it. I've seen a few different ways to do it, but I think the best way is to use a motion controlled robotic arm.

So this project is about how to create a motion controlled robotic arm using a Raspberry Pi and a camera.

## Bill of Materials

| Item           | Description                                 | Price  | Qty | Total  |
| -------------- | ------------------------------------------- | ------ | --- | ------ |
| Stepper Motors | NEMA 17 Stepper Motors - Pack of 5 (Amazon) | £35.00 | 1   | £35.00 |
| Connectors     | Flange Coupling Connectors (Pack of 4)      | £7.99  | 2   | £15.98 |
{:class="table table-striped"}

---

## 3D Printable STL files

Here are the 3d printable files you will need to print for this project:

* [Arm B - Red Side](/assets/stl/motioncontrol/arm_b_red_side.stl)
* [Arm B - White Side](/assets/stl/motioncontrol/arm_a_white_side.stl)
* [Arm A - Red Side](/assets/stl/motioncontrol/arm_a_red_side.stl)
* [Arm A - White Side](/assets/stl/motioncontrol/arm_a_white_side.stl)

---

## Build Log

* Parts take a **Long** time to print (14 hours for 1 part of the Elbow)
* Dr Chris Parrott is concerned the design will not be structurally strong enough, suggests using Servos for simplicity
* May be a bit large - over-engineered
