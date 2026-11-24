---
layout: lesson
title: Introduction to MicroPython Robotics Projects
author: Kevin McAleer
type: page
cover: /learn/micropython_robotics/assets/banners/00_intro.jpg
date: 2025-05-20
next: 02_dc_motors.html
description: Get started with building robots using the Raspberry Pi Pico and MicroPython,
  learn about motors, sensors, and putting it all together.
percent: 8
duration: 7
quiz_total: 36
date_updated: 2026-11-24
redirect_from: /learn/micropython_robotics/01_intro.html
navigation:
- name: MicroPython Robotics Projects with the Raspberry Pi Pico
- content:
  - section: Introduction
    content:
    - name: Introduction to MicroPython Robotics Projects
      link: 00_intro.html
  - section: Motors and Movement
    content:
    - name: Controlling DC Motors with the Raspberry Pi Pico
      link: 02_dc_motors.html
    - name: Using H-Bridge Motor Drivers
      link: 03_hbridge_control.html
  - section: Sensors and Input
    content:
    - name: Line Following with IR Sensors
      link: 04_line_following.html
    - name: Obstacle Avoidance with Ultrasonic Sensors
      link: 05_ultrasonic_sensor.html
  - section: Robot Assembly
    content:
    - name: Building a Two-Wheel Drive Robot
      link: 06_build_robot.html
    - name: Servo Scanning with Ultrasonic Sensors
      link: 07_servo_scanning.html
  - section: Remote Control
    content:
    - name: Bluetooth Control with HC-05
      link: 08_bluetooth_control.html
    - name: Wi-Fi Control with the Raspberry Pi Pico W
      link: 09_wifi_control.html
  - section: Autonomous Behavior
    content:
    - name: Creating Simple Autonomous Behavior
      link: 10_autonomous_robot.html
  - section: Final Project
    content:
    - name: Final Project - Build Your Own Robot
      link: 11_final_project.html
  - section: Summary
    content:
    - name: Course Summary and What's Next
      link: 12_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

Ahoy there makers! By the end of this lesson you will be able to describe the robot we are going to build, check you have the right kit, and know how the pins on the Pico will be used throughout the course.

Robots don't have to be complicated. With a Raspberry Pi Pico, MicroPython and a handful of parts you can build a two-wheeled robot that drives, sees obstacles, follows a line and takes orders from your phone. Kev made a video asking whether MicroPython can handle real robotics, so watch it first, then let's build.

{% include youtubeplayer.html id="6YqzOQ7OZkQ" %}

---

## Overview

This course goes beyond blinking LEDs. You will learn how a **motor driver** lets a tiny microcontroller push big motors around, how **sensors** turn the real world into numbers your code can act on, and how to join the two so the robot can decide things for itself. Every lesson adds one idea to the same robot, and every program is complete, so you can save it to your Pico and run it.

Which board? The code is written for the **Raspberry Pi Pico** (RP2040) and works unchanged on the Pico W, and on the Pico 2 as far as the standard `machine` calls go. Only the Wi-Fi lesson needs a **Pico W** (or Pico 2 W).

```mermaid
flowchart LR
    A["Motors and driver"] --> B["Two-motor drive"]
    B --> C["Sensors: line and distance"]
    C --> D["Build the chassis"]
    D --> E["Servo scanner"]
    E --> F["Bluetooth and Wi-Fi"]
    F --> G["Autonomous behaviour"]
    G --> H["Your own robot"]
```

## Course Content

- Driving one DC motor with an L298N and PWM
- H-bridges, and a reusable two-motor `robot.py`
- Line following with IR sensors
- Measuring distance with the HC-SR04 and avoiding obstacles
- Building and testing the chassis, with a safe power plan
- Pointing a servo and scanning with the ultrasonic sensor
- Remote control with an HC-05 Bluetooth module
- Remote control from a web page on the Pico W
- Autonomous exploring, then your own final project

## Key Results

After completing this course, you will:

- Drive two DC motors forwards, backwards, and at different speeds
- Read digital sensors and the timing signal from an ultrasonic sensor
- Work out PWM duty values for motors and servo angles by hand
- Control your robot over Bluetooth or Wi-Fi
- Write a loop that senses, decides and acts, and test each part on its own

## What you'll need

- Raspberry Pi Pico or Pico W, running MicroPython (see [How to install MicroPython](/learn/how_to_install_micropython/))
- L298N motor driver module
- 2x small DC gear motors with wheels, a caster wheel and a chassis
- 1x HC-SR04 ultrasonic sensor (the standard 5 V version)
- 1 or 2x TCRT5000 style IR line sensor modules with a digital output
- 1x SG90 servo
- Battery pack for the motors (see lesson 6 for the choices), jumper wires, a breadboard
- One 1k ohm and one 2k ohm resistor for the ultrasonic echo line
- Optional: HC-05 Bluetooth module, and a Pico W for Wi-Fi control

### Prerequisites

You should be comfortable saving and running a MicroPython program and using `Pin`. If not, start with [Learn MicroPython - The basics](/learn/micropython/) and [GPIO Mastery](/learn/micropython_gpio/). To understand motors in more depth, [Motors and motor drivers](/learn/motors_and_drivers/) goes further than we do here.

## How the course works

Each lesson opens with an objective, teaches one idea, and ends with **Try it Yourself** exercises, a **Common Issues** list and a short quiz. Wiring is given as tables, because a table of pin numbers is easier to check than a photo.

Here is the whole robot's pin plan. Every lesson uses these pins, so you can keep one wiring all the way through. The pins are our choice, not a rule, so if one is taken in your build, change the number in the code.

| Part | Signal | Pico GP pin | Physical pin |
|------|--------|-------------|--------------|
| L298N motor A (left) | IN1 | GP0 | 1 |
| L298N motor A (left) | IN2 | GP1 | 2 |
| L298N motor A (left) | ENA (PWM) | GP2 | 4 |
| L298N motor B (right) | IN3 | GP3 | 5 |
| L298N motor B (right) | IN4 | GP4 | 6 |
| L298N motor B (right) | ENB (PWM) | GP5 | 7 |
| IR sensor left | OUT | GP6 | 9 |
| IR sensor right | OUT | GP7 | 10 |
| HC-SR04 | TRIG | GP8 | 11 |
| HC-SR04 | ECHO (through a divider) | GP9 | 12 |
| SG90 servo | Signal | GP10 | 14 |
| HC-05 | Pico TX to HC-05 RXD | GP12 | 16 |
| HC-05 | Pico RX from HC-05 TXD | GP13 | 17 |
{:class="table table-single"}

Notice that the Bluetooth module sits on GP12 and GP13 rather than the more usual GP0 and GP1: those two are already driving a motor. GP12 and GP13 are a valid UART0 TX and RX pair on the Pico, so it works.

Code that you should save as a file on the Pico is named in the lesson, for example `robot.py`. Short snippets that are not whole programs are marked as such.

> **Teachers**: This course suits ages 11 and up with some Python, working in pairs or threes. Allow about 45 to 60 minutes per lesson, with lessons 6 and 11 needing a double session. Each group needs one full kit (robot parts, battery pack, a laptop). With no hardware, lessons 3, 5, 7 and 10 still work as reading and maths exercises, and the Pico code can be traced on paper. Batteries are the main safety point, so read the teacher notes in lessons 2 and 6 before the first session.

Let's get started with the simplest moving part of all, a single DC motor.

## Try it Yourself

1. Check your kit against the list above and tick each item off.
2. Draw the pin table on a sheet of paper and keep it beside you for the whole course.
3. Challenge: find GP12 and GP13 on a Pico pinout diagram and confirm they are labelled UART0 TX and RX.

## Check your understanding

{% quiz %}
? Which board does the course need for the Wi-Fi lesson?
- [ ] Any Raspberry Pi Pico
- [x] A Pico W (or Pico 2 W)
- [ ] An Arduino Uno
> Only the W models have the wireless chip, so only they can use the `network` module for Wi-Fi.

? Why is the HC-05 Bluetooth module wired to GP12 and GP13 in this course?
- [ ] Those are the only pins that can do UART
- [x] GP0 and GP1 are already used to drive the left motor
- [ ] The HC-05 only works on physical pin 16
> GP12 and GP13 are a valid UART0 pair, and keeping GP0 and GP1 for the motor means every part can stay wired up at once.

? The ECHO pin of the HC-SR04 goes to the Pico through two resistors. Which is in the kit list?
- [x] 1k ohm and 2k ohm
- [ ] 10 ohm and 20 ohm
- [ ] No resistors are needed
> The two resistors form a divider that brings the sensor's 5 V echo signal down to a level the Pico's 3.3 V pin can take.
{% endquiz %}
