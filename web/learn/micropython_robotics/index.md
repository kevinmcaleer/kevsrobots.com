---
layout: course_overview
title: MicroPython Robotics Projects with the Raspberry Pi Pico
author: Kevin McAleer
cover: /learn/micropython_robotics/assets/cover.jpg
date: 2025-05-20
description: Learn how to build simple yet powerful robots using the Raspberry Pi
  Pico and MicroPython.  This hands-on course guides you through controlling motors,
  reading sensors, and combining hardware to create autonomous and remotely controlled
  robots.  Perfect for makers ready to go beyond GPIO and start building real-world
  robotics projects.
level: intermediate
course_duration: 1h 14m
lesson_count: 12
first_lesson: 00_intro.html
quiz_total: 36
video: 6YqzOQ7OZkQ
outcomes:
- Drive two DC motors forwards, backwards and at different speeds with an L298N driver
  and PWM
- Read an infrared line sensor and an HC-SR04 ultrasonic sensor, and turn the readings
  into decisions
- Aim an SG90 servo by working out the pulse width for any angle
- Control the robot from a phone over Bluetooth (HC-05) or Wi-Fi (Pico W)
- Combine motors and sensors into an autonomous robot that avoids obstacles and follows
  a line
prerequisites:
- Learn MicroPython - The basics
- Raspberry Pi Pico with MicroPython - GPIO Mastery
you_need:
- A Raspberry Pi Pico or Pico W (a Pico W for the Wi-Fi lesson) running MicroPython
- An L298N motor driver module and two small DC gear motors with wheels
- An HC-SR04 ultrasonic sensor, one or two TCRT5000 style IR line sensors and an SG90
  servo
- A battery pack for the motors, jumper wires and resistors (1k and 2k ohm)
- Optional, an HC-05 Bluetooth module
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
