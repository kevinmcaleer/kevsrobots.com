---
layout: lesson
title: Camera
author: Kevin McAleer
type: page
cover: /learn/burgerbot/assets/burgerbot.jpg
date: 2023-04-27
previous: 08_line_following_code.html
description: null
percent: 100
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


## Vision

Another fun upgrade for your Burgerbot is a camera. The ESP32-Cam module is a cheap and easy to use camera module that can be powered from the Pico's `5v` and `GND` pins. The Camera module hosts its own webserver and can stream the video over Wi-Fi to a browser.

[![Camera Module](assets/camera_module.jpg){:class="img-fluid w-100"}](assets/camera_module.jpg)

---

## Setting up the camera

The first time you plug in the camera it will broadcast its own Wi-Fi hotspot ID that you will need to connect to. You can then connect it to your local Wi-Fi hotspot with the password too. The next time it restarts it will remember these settings and connect up. You will need to check the devices connected to your router to find the IP address of the camera, however once you have this you can type that into your browser and being viewing what the robot can see.

---

## The Camera holder 3D Printable part

The camera module requires the existing BurgerBot range finder holder is replaced with a new 3d printable part.

Click here to download the new Camera holder:

* [`camera_holder.stl`](/assets/stl/burgerbot_v2/camera_holder.stl) - The Camera Holder

---
