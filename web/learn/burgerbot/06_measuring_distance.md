---
layout: lesson
title: Measuring Distance
author: Kevin McAleer
type: page
cover: /learn/burgerbot/assets/burgerbot.jpg
date: 2023-04-27
previous: 05_basic_movement.html
next: 07_object_avoidance.html
description: Learn how to measure distance using the range finder on BurgerBot.
percent: 63
duration: 2
navigation:
- name: Build your own BurgerBot
- content:
  - section: Overview
    content:
    - name: Overview
      link: 00_intro.html
    - name: Why BurgerBot?
      link: 01_why_video.html
    - name: What is BurgerBot?
      link: 01_what_is_burger_bot.html
  - section: Assembling the parts
    content:
    - name: Assembly
      link: 02_assembly.html
    - name: BurgerBot Assembly video
      link: 02_video.html
  - section: Installing the code
    content:
    - name: Getting the code
      link: 03_getting_the_code.html
    - name: Loading the code
      link: 04_loading_the_code.html
  - section: Programming in MicroPython
    content:
    - name: Basic Movement
      link: 05_basic_movement.html
    - name: Measuring Distance
      link: 06_measuring_distance.html
    - name: Object Avoidance
      link: 07_object_avoidance.html
  - section: Upgrades
    content:
    - name: BurgerBot Upgrades video
      link: 08_video.html
    - name: Line Following
      link: 08_line_following.html
    - name: Line Following Code
      link: 08_line_following_code.html
    - name: Camera
      link: 09_camera.html
---


## Ultrasonic Range Finder

Burgerbot includes a simple range finder sensor at the front of the robot.

![Line folllow animation](assets/line_follow_animation.gif)

Ultrasonic range finders are versatile and widely used devices that measure distance by utilizing the principles of sound wave propagation. These handy gadgets have found their way into a plethora of applications, from robotics and automation to obstacle detection and parking assistance. 

---
## How Ultrasonic Range Finders Work

Ultrasonic range finders operate using the basic principles of echolocation, a technique employed by animals such as bats and dolphins to navigate and locate objects. Here's a step-by-step explanation of the process:

**Transmitter**: The range finder consists of a transmitter that emits ultrasonic sound waves, typically at frequencies between 20 kHz and 200 kHz. These frequencies are higher than the audible range for humans, which is why they're called `"ultrasonic"`.

**Sound Wave Propagation**: The emitted sound waves travel through the air and eventually reach an object or surface in their path.

**Reflection**: Upon hitting the object, the sound waves reflect back towards the range finder.

**Receiver**: The device has a built-in receiver that detects the reflected sound waves, often using a piezoelectric element that converts the mechanical vibration of the sound waves into an electrical signal.

**Time-of-Flight Calculation**: The range finder measures the time it takes for the emitted sound waves to travel to the object and bounce back. This is known as the "time of flight."

**Distance Calculation** : Using the time of flight and the known speed of sound in air (approximately 343 meters per second), the range finder calculates the distance to the object using the following formula:

Distance = (Time of Flight x Speed of Sound) / 2

The division by 2 accounts for the fact that the sound waves travel to the object and back, covering twice the actual distance.

---

## MicroPython Example

Here is example code for using a range finder. This is simplified within the burgerbot:

<script src="https://gist.github.com/kevinmcaleer/c7e2a95adeaf6215617f9dcfe5ce880a.js"></script>

---

## Using the range finder with BurgerBot

To measure distance with BurgerBot simple use the distance property - it will take a measurement and return it:

```python
bot = Burgerbot()
distance = bot.distance
```

---
