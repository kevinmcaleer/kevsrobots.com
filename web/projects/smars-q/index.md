---
title: "SMARS Q"
description: "An Arduino Uno Q Based Robot"
excerpt: >-
    This project showcases the design and construction of the SMARS Q robot, an Arduino Uno-based robot designed for various tasks.
layout: showcase
date: 2025-10-19
author: Kevin McAleer
difficulty: beginner
cover: /projects/smars-q/assets/cover.jpg
hero:  /projects/smars-q/assets/hero.png
mode: light
videos:
  - MWjuvcfVdYQ
  - C0-YFSS-cbw
  - 4HFb-FGQ8_w
tags:
 - arduino
 - robotics
groups:
 - python
 - robotics
 - arduino
code:
 - https://www.github.com/kevinmcaleer/smars_q
# stl:
#  - name: Body
#    link: /projects/thinkman/assets/thinkman_body_v1.stl
#  - name: Top
#    link: /projects/thinkman/assets/thinkman_top_v1.stl
#  - name: Flap
#    link: /projects/thinkman/assets/thinkman_flap_v1.stl
---

{% include projects/nav.html %}

Arduino recently launched the [Arduino Uno Q](https://www.arduino.cc/products/boards/arduino-uno-q), which is a new version of the classic Arduino Uno board, but with some exciting new features. The Uno Q is a Single Board Computer (SBC) similar to the Raspberry Pi but in the classic Arduino form factor.

The Uno Q is twice as fast as the Raspberry Pi Zero 2 W, and about 40% as fast as the Raspberry Pi 5.

{% include gallery.html images="assets/speed.jpg" titles="Speed comparison" cols=1%}

---

## SMARS Q

As this is an Arduino in the Uno form factor I immediately thought about putting this into a [SMARS](/smars) robot, and exploring what new things we can do with it.

{% include gallery.html images="assets/smars-outline.jpg" titles="SMARS Q Robot" links="/smars"%}

---

One of things that is new is that the SBC can run special programs writting in the Arduino App Lab, which as a combination of classic Arduino C++ code, and Python code. This opens up a whole new world of possibilities for programming the SMARS Q robot.

---

## Arduino App Lab

The [Arduino App Lab](https://docs.arduino.cc/software/app-lab/tutorials/getting-started/) splits the responsibities of the code between things that need realtime operations, such as reading sensors and controlling motors, and things that can be run in a higher level language such as Python, such as computer vision and AI.

Arduino make programming this easier by providing a special set of pre-built building blocks of code called [Bricks](https://docs.arduino.cc/software/app-lab/tutorials/bricks/), which can be used to quickly build applications for the Uno Q. 

Bricks are actually Docker Containers, which means it will be easier in future to create your own custom bricks for specific tasks (this isn't easily achiveable right now, but Arduino have said they are working on it).

---
