---
layout: lesson
title: Getting the code
author: Kevin McAleer
type: page
cover: /learn/burgerbot/assets/burgerbot.jpg
date: 2023-04-27
previous: 02_video.html
next: 04_loading_the_code.html
description: Learn how to use the BurgerBot MicroPython library to control the robotic
  vehicle.
percent: 42
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


## BurgerBot MicroPython library

BurgerBot has a nice Python Library that controls the robotic vehicle, which includes motors and sensors, such as a distance sensor, line sensor and a servo for the pen.

---

## Table of Contents

{:toc}
* toc

---

## Required Libraries

This library uses the [Pimoroni flavoured MicroPython](https://www.github.com/pimoroni/pimoroni-pico/releases).

The following libraries are required for the BurgerBot to function properly (and they are all inlcuded in the Pimoroni MicroPython):

- `motor` and `pico_motor_shim` from the `motor` module.
- `REVERSED_DIR` from the `pimoroni` module.
- `Servo` from the `servo` module.
- `connect_to_wifi`, `logging`, `access_point`, `dns`, `server`, and `redirect` from the `phew` module.
- `gc` and `Pin` from the `machine` module.

---

## Class and Methods

The `BurgerBot` class contains the following methods:

Method | Description
---|---
`__init__()` | Initializes the BurgerBot class with a list of motors, `MOTOR_PINS`, and sets the `__speed` variable to 0.
`line_detected() -> bool` | Returns a boolean value indicating whether a line has been detected by the line sensor.
`pen_middle()` | Moves the pen servo to the middle position.
`pen_down()` | Moves the pen servo to the down position.
`pen_up()` | Moves the pen servo to the up position.
`forward(duration)` | Drives the motors forward, the 'duration' is optional and moves for the time specified
`turnleft(duration)` | Turns the motors left, the 'duration' is optional and moves for the time specified
`turnright(duration)` | Turns the motors right, the 'duration' is optional and moves for the time specified
`stop()` | Stops the motors.
`left_motor(speed)` | Sets the speed of the left motor to the provided value.
`right_motor(speed)` | Sets the speed of the right motor to the provided value.
`speed() -> float` | Returns the current speed of the motors.
`speed(value)` | Sets the speed of the motors to the provided value.
{:class="table table-code-fit"}
---

## Variables

The BurgerBot also contains the following variables:

- `motors`: a list of two `Motor` objects representing the left and right motors.
- `pen_servo`: a `Servo` object representing the pen servo.
- `line_sensor`: a `Pin` object representing the line sensor pin.

---

## Example demo

<script src="https://gist.github.com/kevinmcaleer/7e81c0959cd7b0b248f7636aaa637d58.js"></script>

---
