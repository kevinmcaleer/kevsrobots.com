---
layout: lesson
title: Loading the code
author: Kevin McAleer
type: page
cover: /learn/burgerbot/assets/burgerbot.jpg
date: 2023-04-27
previous: 03_getting_the_code.html
next: 05_basic_movement.html
description: Learn how to upload the burgerbot.py file to a Raspberry Pi Pico using
  Thonny IDE.
percent: 42
duration: 1
quiz_total: 11
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
    - name: Drawing with a Pen
      link: 08_pen.html
    - name: Line Following
      link: 08_line_following.html
    - name: Line Following Code
      link: 08_line_following_code.html
    - name: Camera
      link: 09_camera.html
---


## Uploading the code

To upload the `burgerbot.py` file to a Raspberry Pi Pico using [Thonny](https://thonny.org), follow these steps:

1. Connect the Raspberry Pi Pico to your computer using a USB cable.
1. Open Thonny on your computer.
1. Go to the **Tools** menu and select **Options**.
1. In the **Options** window, go to the **Interpreter** tab.
1. Select **MicroPython (Raspberry Pi Pico)** from the list.
1. Click **OK** to close the **Options** window.
1. Go to the **File** menu and select **Open**, then open `burgerbot.py` from your computer.
1. Go to the **File** menu and select **Save as**, choose **Raspberry Pi Pico**, and save it as `burgerbot.py`.
1. Create a new file, type `from burgerbot import Burgerbot` and run it with the green **Run** button. If there's no error, the library is installed.

> ## Make it run on its own
>
> Save a program on the Pico as `main.py` and it will run automatically whenever the robot is switched on, without a computer attached.

---
