---
layout: lesson
title: Line Following Code
author: Kevin McAleer
type: page
cover: /learn/burgerbot/assets/burgerbot.jpg
previous: 08_line_following.html
next: 09_camera.html
description: null
percent: 91
duration: 1
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


## Wiring up the Line sensor

Attach the following wires to the pico

Pico GPIO Pin | Line Sensor | Comments
--------------|------------|---
3.3v out      | V+ | Solder the end of a female dupont cable to the 3.3v out on the Pico
GND           | GND | Solder the end of a female dupont cable to the ground (GND) out on the Pico
GPIO 17       | Signal| Solder the end of a female dupont cable to GPIO 17 on the Pico
{:class="table table-code-fit"}

[![Wiring Diagram](assets/wiring.jpg){:class="img-fluid w-100"}](assets/wiring.jpg)

## MicroPython code

Using a line following sensor is fun and simple! After you attach the special wires (called Dupont connectors) to your robot, you can make it work with some cool code.

Remember, we need to slow down our robot's `speed` so it doesn't go too fast and miss the line it's trying to follow. Also, we want our robot to check the line very often, so it can stay on track while moving along the line.

<script src="https://gist.github.com/kevinmcaleer/088f8409d03e0b5cbdeb09dd9dfad086.js"></script>

---
