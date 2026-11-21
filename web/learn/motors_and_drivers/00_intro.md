---
layout: lesson
title: Motors and motor drivers
author: Kevin McAleer
type: page
cover: /learn/motors_and_drivers/assets/banners/00_intro.jpg
date: 2026-11-21
next: 01_how_motors_work.html
description: Meet the course, the Scout drive base project, and what you need to get
  started with motors on the Raspberry Pi Pico.
percent: 5
duration: 4
date_updated: 2026-11-21
navigation:
- name: Motors and motor drivers
- content:
  - section: Introduction
    content:
    - name: Motors and motor drivers
      link: 00_intro.html
  - section: Motor basics
    content:
    - name: How motors work
      link: 01_how_motors_work.html
    - name: Why you need a motor driver
      link: 02_why_you_need_a_driver.html
    - name: Powering motors
      link: 03_powering_motors.html
  - section: DC motors
    content:
    - name: The H-bridge
      link: 04_the_h_bridge.html
    - name: Wiring a motor driver
      link: 05_wiring_a_motor_driver.html
    - name: Speed with PWM
      link: 06_speed_with_pwm.html
    - name: A motor class
      link: 07_a_motor_class.html
    - name: Driving two motors
      link: 08_driving_two_motors.html
    - name: Ramping and braking
      link: 09_ramping_and_braking.html
  - section: Servos
    content:
    - name: Servo basics
      link: 10_servo_basics.html
    - name: A servo class and calibration
      link: 11_a_servo_class_and_calibration.html
    - name: Continuous rotation servos
      link: 12_continuous_rotation_servos.html
  - section: Steppers and sensing
    content:
    - name: Stepper motors
      link: 13_stepper_motors.html
    - name: Measuring speed with encoders
      link: 14_measuring_speed_with_encoders.html
  - section: Project
    content:
    - name: Project - the Scout drive base
      link: 15_project_drive_base.html
    - name: Summary
      link: 16_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

Motors are what turn a pile of electronics into a robot. You have probably wired one up from a tutorial, copied some pin numbers, and watched a wheel spin. This course is about understanding *why* it worked, so that when a motor stutters, a Pico resets, or a driver gets hot, you know exactly where to look.

## Overview

We cover the three motors you will meet in almost every robot: the humble DC motor, the hobby servo, and the stepper. For each one you will learn how it works, how to power it safely, and how to drive it with MicroPython on a Raspberry Pi Pico.

Everything builds towards one project. **Scout** is a two-wheeled drive base with a servo-mounted "head", the perfect platform for a distance sensor later on. By the final lesson Scout drives a square, spins on the spot, and looks left and right on command.

## Course Content

- How DC motors, servos and steppers work, and when to choose each
- Why a Pico pin can never drive a motor directly, and what a driver does for you
- Powering motors: batteries, brownouts and the common ground
- The H-bridge, and wiring an L298N or a DRV8833 to the Pico
- Speed control with PWM, and a reusable `Motor` class
- Driving two motors, ramping and braking
- Servo pulse widths, a `Servo` class, calibration and continuous rotation servos
- Stepper motors with the 28BYJ-48 and ULN2003
- Measuring wheel speed with a simple encoder
- The Scout project: square, spin and head look-around

## Key Results

After completing this course, you will:

- Choose between a DC motor, a servo and a stepper for a given job
- Wire a motor driver and a separate motor supply without frying anything
- Control speed, direction and braking from MicroPython
- Write clean, reusable `Motor`, `Servo` and `DriveBase` classes
- Calibrate a drive base so a square really is a square
- Know what to check first when a motor misbehaves

## What you'll need

- A Raspberry Pi Pico with headers
- An L298N or DRV8833 motor driver board
- Two TT gear motors with wheels, on a chassis such as SMARS or a cheap 2WD kit
- An SG90 servo
- Optional: a 28BYJ-48 stepper motor with a ULN2003 driver board
- A 4xAA battery pack
- A breadboard and jumper wires
- Thonny or another way to put MicroPython files on the Pico

## How the course works

Each lesson introduces one idea, shows a short piece of working code, and ends with **Try it Yourself** exercises and a **Common Issues** list. Code blocks are complete, so you can copy one and run it.

From lesson 7 onwards we save reusable code as small files on the Pico (`motor.py`, `drive.py`, `ramp.py`, `servo.py`) and import them, exactly like you would on a real project. The file name is always the first line of the listing, as a comment.

We use the same pin numbers as the [MicroPython Robotics](/learn/micropython_robotics/) course, so the wiring will feel familiar:

| Pin | Used for |
|---|---|
| GP0, GP1, GP2 | Left motor: IN1, IN2, ENA (PWM) |
| GP3, GP4, GP5 | Right motor: IN3, IN4, ENB (PWM) |
| GP10 | Servo head |
| GP14, GP15 | Wheel encoders (lesson 14) |
| GP16 to GP19 | Stepper IN1 to IN4 (lesson 13) |
{:class="table table-single"}

Take it slowly, and do not worry if a wheel spins the wrong way the first time. Every robot builder has swapped a pair of wires and laughed about it.

## Before you start

This course assumes you have finished:

- [Electronics basics](/learn/electronics_basics/), for voltage, current, ground and breadboards
- [Learn MicroPython - The basics](/learn/micropython/), for variables, loops, functions and classes

Ready? Let's meet the motors.
