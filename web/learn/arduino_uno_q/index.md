---
layout: course_overview
title: Arduino Uno Q - Two Brains, One Board
author: Kevin McAleer
cover: /learn/arduino_uno_q/assets/cover.jpg
date: 2026-11-21
description: Learn the Arduino Uno Q from first boot to a project that runs by itself.
  Split a job between its Linux computer and its microcontroller, make Python and
  an Arduino sketch talk over the Bridge, and build the Q Board, a number display
  you drive from Python.
level: beginner
course_duration: 1h 13m
lesson_count: 10
first_lesson: 00_intro.html
quiz_total: 27
video: 4HFb-FGQ8_w
outcomes:
- Explain what the Linux side and the microcontroller side of the Uno Q are each good
  at
- Set the board up in Arduino App Lab and work on it over Wi-Fi
- Draw pictures and animations on the LED matrix
- Read sensors and move a servo from an Arduino sketch
- Call sketch functions from Python, and Python functions from a sketch, over the
  Bridge
- Find and fix the common Bridge bugs, including the provide() deadlock
prerequisites:
- A little Python or a little Arduino C++. You don't need both
you_need:
- An Arduino Uno Q (2 GB or 4 GB)
- A USB-C data cable
- Arduino App Lab on a Windows, macOS or Linux computer
- A 10k potentiometer and three jumper wires (lessons 5 and 8)
- Optional, a Modulino Thermo with a Qwiic cable, and a small hobby servo (lesson
  5)
navigation:
- name: Arduino Uno Q - Two Brains, One Board
- content:
  - section: Meet the board
    content:
    - name: Introduction
      link: 00_intro.html
    - name: Two brains, one board
      link: 01_two_brains_one_board.html
    - name: First boot, App Lab and Wi-Fi
      link: 02_first_boot_app_lab_and_wifi.html
    - name: Your first App
      link: 03_your_first_app.html
  - section: The microcontroller side
    content:
    - name: Drawing on the LED matrix
      link: 04_drawing_on_the_led_matrix.html
    - name: Pins, sensors and servos
      link: 05_pins_sensors_and_servos.html
  - section: The Bridge
    content:
    - name: How the Bridge works
      link: 06_how_the_bridge_works.html
    - name: Python calls the sketch
      link: 07_python_calls_the_sketch.html
    - name: The sketch calls Python
      link: 08_the_sketch_calls_python.html
    - name: When the Bridge goes wrong
      link: 09_when_the_bridge_goes_wrong.html
---
