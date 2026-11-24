---
layout: course_overview
title: Motors and motor drivers
author: Kevin McAleer
cover: /learn/motors_and_drivers/assets/cover.jpg
date: 2026-11-21
description: Learn how DC motors, servos and stepper motors work, why they need a
  motor driver, and how to control all three from a Raspberry Pi Pico with MicroPython.
  Along the way you will build Scout, a two-wheeled drive base with a servo-mounted
  head that drives a square, spins on the spot and looks left and right.
level: beginner
course_duration: 1h 38m
lesson_count: 17
first_lesson: 00_intro.html
quiz_total: 49
outcomes:
- Explain the difference between DC, servo and stepper motors and pick the right one
  for a job
- Wire an L298N or DRV8833 motor driver to a Pico with a safe, separate motor supply
  and a common ground
- Control motor speed and direction with PWM, including braking and soft starts
- Position a hobby servo precisely using pulse widths and calibrate its real range
- Step a 28BYJ-48 stepper motor through an exact number of degrees
- Build and tune a complete two-wheel drive base that drives a square and spins on
  the spot
prerequisites:
- Electronics basics
- Learn MicroPython - The basics
you_need:
- A Raspberry Pi Pico with headers
- An L298N or DRV8833 motor driver board
- Two TT gear motors with wheels, on a chassis such as SMARS or a cheap 2WD kit
- An SG90 servo
- Optional, a 28BYJ-48 stepper motor with a ULN2003 driver board
- A 4xAA battery pack
- A breadboard and jumper wires
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
