---
layout: course_overview
title: Learn how to program SMARS with Arduino
author: Kevin McAleer
cover: /learn/smars_code/assets/program_smars.png
date: 2023-09-10
description: 'Program your SMARS robot in Arduino C++: drive it around, avoid obstacles
  with an ultrasonic sensor, sense its position, control it over Bluetooth, move servos
  and follow a line.'
level: beginner
course_duration: 1h 4m
lesson_count: 8
first_lesson: 00_intro.html
quiz_total: 21
video: XhArIe9UDoc
outcomes:
- Write, verify and upload an Arduino sketch to a SMARS robot
- Drive SMARS forwards, backwards and turning, and control it from the keyboard with
  the Serial Monitor
- Measure distance with an HC-SR04 ultrasonic sensor and use it to avoid obstacles
- Read pitch, roll and heading from an MPU-6050 module
- Control SMARS from a phone over Bluetooth, and sweep a servo
- Follow a line with a TCRT5000 sensor by tuning a threshold
prerequisites:
- Build a SMARS Robot, the first course in the series
- No programming experience needed
you_need:
- A built SMARS robot with an Arduino Uno and a Fundumoto motor shield
- A USB cable and a computer that can run the Arduino IDE
- Optional, one per later lesson, an HC-SR04 ultrasonic sensor, an MPU-6050 module,
  a Bluetooth module (HM-10 or HC-08 for iPhone), an SG90 servo and a TCRT5000 line
  sensor
navigation:
- name: Learn how to program SMARS with Arduino
- content:
  - section: Overview
    content:
    - name: Overview
      link: 00_intro.html
  - section: Movement
    content:
    - name: Lesson 01 - Movement
      link: 01_lesson_01.html
    - name: Lesson 02 - Turning
      link: 02_lesson_02.html
  - section: Sensing
    content:
    - name: Lesson 03 - Ultrasonic distance sensor
      link: 03_lesson_03.html
    - name: Lesson 04 - Positioning with the MPU-6050
      link: 04_lesson_04.html
  - section: Control
    content:
    - name: Lesson 05 - Bluetooth
      link: 05_lesson_05.html
    - name: Lesson 06 - How to control servos
      link: 06_lesson_06.html
    - name: Lesson 07 - Line Following
      link: 07_lesson_07.html
---
