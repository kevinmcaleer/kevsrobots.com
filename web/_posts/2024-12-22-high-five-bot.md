---
title: "High Five Bot"
description: >-
    The robot that gives you a well deserved high five
excerpt: >-
    The High Five Bot is a robot that gives you a well deserved high five. The robot is powered by a Raspberry Pi Pico and uses an ultrasonic range finder to detect your waiting
layout: showcase
date: 2024-12-21
author: Kevin McAleer
difficulty: beginner
cover: /assets/img/blog/high_five_bot/cover.jpg
hero:  /assets/img/blog/high_five_bot/hero.png
mode: light
tags:
 - robots
groups:
 - pico
 - micropython
 - robots
videos:
 - f4p6wBqGEJI
stl:
 - name: "base"
   link: "/assets/stl/high_five_bot/base.stl"
 - name: "top"
   link: "/assets/stl/high_five_bot/top.stl"
 - name: "hand"
   link: "/assets/stl/high_five_bot/hand.stl"
 - name: "face"
   link: "/assets/stl/high_five_bot/face.stl"
 - name: "spacer"
   link: "/assets/stl/high_five_bot/spacer.stl"
 - name: "hood"
   link: "/assets/stl/high_five_bot/hood.stl"
 - name: "pico_holder"
   link: "/assets/stl/high_five_bot/pico_holder.stl"
---

## Overview

The High Five Bot is a robot that gives you a well deserved high five. The robot is powered by a Raspberry Pi Pico and uses an ultrasonic range finder to detect your waiting hand and a stepper motor to move the hand.

---

## Design Goals

The goals for the High Five Bot are:

- Detect a hand using an ultrasonic range finder
- Move the hand using a stepper motor
- Be powered by a Raspberry Pi Pico
- To be a fun project for my 400th YouTube Video

---

## Bill of Materials

The parts used in the High Five Bot are:

Item              | Description                                    | Qty | Price | Total
------------------|------------------------------------------------|:---:|------:|-----:
Raspberry Pi Pico | Microcontroller                                |  1  | £4.00 | £4.00
Stepper Motor     | 24BYJ-48 Stepper                               |  1  | £2.00 | £2.00
Range Finder      | HC-SR04 Ultrasonic Range Finder (3.3v version) |  1  | £1.00 | £1.00
M2 Screws         | For mounting the Raspberry Pi Pico             |  4  | £0.10 | £0.40
Total             |                                                |     |       | £7.40
{:class="table table-striped"}

---

## Design

The design of the High Five Bot is simple. The ultrasonic range finder is mounted on the front of the robot, and the stepper motor is mounted to the base with a slot for the arm to swivel round. The Raspberry Pi Pico is mounted on the back of the robots head. The Stepper motor kind of just hangs around, because I didn't have a good way to mount it.

The hands resting position is behind the robot and when you move your hand in front of the robot, the ultrasonic range finder detects your hand and the stepper motor moves the hand to give you a high five.

---

## Stepper Motors

The stepper motor used in the High Five Bot is a 24BYJ-48 stepper motor. The stepper motor is a unipolar stepper motor and is driven by a ULN2003 driver board. The stepper motor is powered by the Raspberry Pi Pico and is controlled using the MicroPython `stepper_28byj48.py` driver.

Stepper Motors are unlike servo motors - instead of using [PWM](/resources/glossary#pwm) to control the position of the motor, stepper motors use a series of pulses to move the motor a set distance. The stepper motor used in the High Five Bot is a 5-wire stepper motor, which means it has 4 coils and a common wire.

We need to send a series of pulses to the stepper motor to move it a set distance. The `stepper_28byj48.py` driver takes care of this for us, but lets have a look what happens:

The sequence of pulses to move the stepper motor is as follows:

```raw
Step | A | B | C | D
-----|---|---|---|---
  1  | 1 | 0 | 0 | 0
  2  | 1 | 1 | 0 | 0
  3  | 0 | 1 | 0 | 0
  4  | 0 | 1 | 1 | 0
  5  | 0 | 0 | 1 | 0
  6  | 0 | 0 | 1 | 1
  7  | 0 | 0 | 0 | 1
  8  | 1 | 0 | 0 | 1
```

Each step moves the stepper motor a number of degrees, and the number of degrees per step is determined by the stepper motor itself. The `stepper_28byj48.py` driver takes care of this for us, and we can simply tell the driver to move the stepper motor a number of steps.

We can also control the speed of the stepper motor by changing the delay between each step. The delay is set in the driver and can be changed to make the stepper motor move faster or slower.

![Stepper Motor](/assets/img/blog/high_five_bot/stepper_motors.jpg){:class="img-fluid w-100 rounded-3"}

---

## Wiring

To wire up the High Five Bot, you will need to connect the stepper motor and the ultrasonic range finder to the Raspberry Pi Pico. The wiring diagram is as follows:

![Wiring](/assets/img/blog/high_five_bot/wiring.jpg){:class="img-fluid w-100 rounded-3"}

Raspberry Pi Pin | Stepper Motor Pin | Ultrasonic Range Finder Pin
----------------:|:-----------------:|:--------------------------:
        0 (GP00) |                   |            ECHO
        1 (GP01) |                   |            TRIG
         2 (3V3) |                   |             VCC
         3 (GND) |                   |             GND
        4 (GP10) |        IN1        |              -
        5 (GP11) |        IN2        |              -
        6 (GP12) |        IN3        |              -
        7 (GP13) |        IN4        |              -
        8 (VSYS) |        VCC        |              -
         9 (GND) |        GND        |              -
{:class="table table-striped"}

---

## Assembly

High Five Bot is made up of a few 3D printed parts, along with the electronics:

Part        | Description                      | File
------------|----------------------------------|------------------------------------------------------
Base        | The base of the robot            | [Download](/assets/stl/high_five_bot/base.stl)
Top         | The top of the robot             | [Download](/assets/stl/high_five_bot/top.stl)
Hand        | The hand of the robot            | [Download](/assets/stl/high_five_bot/hand.stl)
Face        | The face of the robot            | [Download](/assets/stl/high_five_bot/face.stl)
Spacer      | The spacer for the stepper motor | [Download](/assets/stl/high_five_bot/spacer.stl)
Hood        | The hood for the robot           | [Download](/assets/stl/high_five_bot/hood.stl)
Pico Holder | The Raspberry Pi Pico holder     | [Download](/assets/stl/high_five_bot/pico_holder.stl)
{:class="table table-striped"}

Here is the Step-by-Step guide to assembling the High Five Bot:

![High Five Bot Assembly](/assets/img/blog/high_five_bot/assembly01.png){:class="img-fluid w-100 rounded-3"}

![High Five Bot Assembly](/assets/img/blog/high_five_bot/assembly02.png){:class="img-fluid w-100 rounded-3"}

![High Five Bot Assembly](/assets/img/blog/high_five_bot/assembly03.png){:class="img-fluid w-100 rounded-3"}

![High Five Bot Assembly](/assets/img/blog/high_five_bot/assembly04.png){:class="img-fluid w-100 rounded-3"}

![High Five Bot Assembly](/assets/img/blog/high_five_bot/assembly05.png){:class="img-fluid w-100 rounded-3"}

![High Five Bot Assembly](/assets/img/blog/high_five_bot/assembly06.png){:class="img-fluid w-100 rounded-3"}

![High Five Bot Assembly](/assets/img/blog/high_five_bot/assembly07.png){:class="img-fluid w-100 rounded-3"}

![High Five Bot Assembly](/assets/img/blog/high_five_bot/assembly08.png){:class="img-fluid w-100 rounded-3"}

![High Five Bot Assembly](/assets/img/blog/high_five_bot/assembly09.png){:class="img-fluid w-100 rounded-3"}

![High Five Bot Assembly](/assets/img/blog/high_five_bot/assembly10.png){:class="img-fluid w-100 rounded-3"}

![High Five Bot Assembly](/assets/img/blog/high_five_bot/assembly11.png){:class="img-fluid w-100 rounded-3"}

---

## Code

The code for the High Five Bot is written in MicroPython and is available on my GitHub page <https://www.github.com/kevinmcaleer/high_five_bot>.

The code is made up of two drivers, and the main program. The drivers are for the stepper motor and the ultrasonic range finder. The main program is a simple loop that checks the distance from the ultrasonic range finder and moves the stepper motor accordingly.

You will need to copy the `range_finder.py` and `stepper_28byj48.py` drivers to the Raspberry Pi Pico, along with the `main.py` program.

<script src="https://gist.github.com/kevinmcaleer/09b0c47347c3e5df9be34e2e1e160941.js"></script>

---
