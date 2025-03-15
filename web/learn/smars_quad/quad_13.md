---
layout: lesson
title: Wiring
author: Kevin McAleer
type: page
cover: /learn/smars_quad/assets/how_to_build_a_quad.jpg
date: 2023-03-04
previous: quad_12.html
description: Wiring the SMARS Quad robot
percent: 100
duration: 1
navigation:
- name: SMARS Quad
- content:
  - section: Overview
    content:
    - name: SMARS Quad robot Overview
      link: 00_intro.html
  - section: Print parts
    content:
    - name: Print the Parts
      link: quad_01.html
    - name: Servo Holders
      link: quad_02.html
    - name: Fit the servos
      link: quad_03.html
  - section: Assemble robot
    content:
    - name: Servo Frame
      link: quad_04.html
    - name: Servo Arms
      link: quad_05.html
    - name: Assemble arms
      link: quad_06.html
    - name: Calibrate Servos
      link: quad_07.html
    - name: Servo Horns
      link: quad_08.html
    - name: Print Feet
      link: quad_09.html
    - name: Assemble Feet
      link: quad_10.html
    - name: Check Rotation
      link: quad_11.html
    - name: Code
      link: quad_12.html
    - name: Wiring
      link: quad_13.html
---


![wiring](assets/wiring01.jpg){:class="img-fluid w-100"}

![wiring](assets/wiring02.jpg){:class="img-fluid w-100"}

---

## PCA9685 Pinout

PCA9685 Pin | Quad Limb
:----------:|-----------------
     0      | Front Left Leg
     1      | Front Left Foot
     2      | Back Left Leg
     3      | Back Left Foot
     4      | Back Right Leg
     5      | Back Right Foot
     6      | Front Right Leg
     7      | Front Right Foot
{:class="table table-striped "}

---

## PCA9685 to Pi Wiring

PCA9685 | Raspberry Pi
:------:|:-----------:
  GND   |   6 (GND)
  VCC   |   2 (VCC)
  SCL   |   5 (SDL)
  SDA   |   3 (SDA)
{:class="table table-striped "}

---
