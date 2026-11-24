---
title: Motors and motor drivers
description: Meet the course, the Scout drive base project, and what you need to get started with motors on the Raspberry Pi Pico.
layout: lesson
type: page
cover: /learn/motors_and_drivers/assets/banners/00_intro.jpg
date_updated: 2026-11-24
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

If you use a DRV8833 instead of an L298N it has no enable pins, so GP2 and GP5 stay unused and the speed PWM goes on the IN pins. Lesson 5 shows both wirings.

> **Teachers**: this course suits learners aged about 11 and up who have done a first Pico course, and it works well in groups of two or three with one robot each. Allow 30 to 45 minutes per lesson, and two sessions for the Scout project in lesson 15. Per group you need a Pico, a motor driver, two TT motors on a chassis, a servo and a 4xAA battery pack (alkaline or NiMH is the safest choice for a classroom). The stepper in lesson 13 and the encoders in lesson 14 are optional. Lessons 1 to 4 and the code-only parts of 6, 8, 9 and 10 can be read and discussed with no hardware at all, and the pure-Python maths functions run in Thonny on a Pico with nothing attached. Keep batteries out of boards between sessions, and run robots on the floor, not on desks.

Take it slowly, and do not worry if a wheel spins the wrong way the first time. Every robot builder has swapped a pair of wires and laughed about it.

## Before you start

This course assumes you have finished:

- [Electronics basics](/learn/electronics_basics/), for voltage, current, ground and breadboards
- [Learn MicroPython - The basics](/learn/micropython/), for variables, loops, functions and classes

Ready? Let's meet the motors.
