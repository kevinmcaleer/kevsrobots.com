---
layout: course_overview
title: Arduino Uno Q - Two Brains, One Board
author: Kevin McAleer
cover: /learn/arduino_uno_q/assets/cover.jpg
date: 2026-11-21
description: Learn the Arduino Uno Q from first boot to a project that runs by itself.
  Split a job between its Linux computer and its microcontroller, make Python and
  an Arduino sketch talk over the Bridge, and build the Q Board, a number display
  you drive from Python, a web page and a camera.
level: beginner
course_duration: 2h 16m
lesson_count: 17
first_lesson: 00_intro.html
quiz_total: 50
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
- Work on the Linux side from a terminal, and back up your Apps
- Give a project a web page, a camera with object detection, and make it start at
  power-on
prerequisites:
- A little Python or a little Arduino C++. You don't need both
you_need:
- An Arduino Uno Q (2 GB or 4 GB)
- A USB-C data cable
- Arduino App Lab on a Windows, macOS or Linux computer
- A 10k potentiometer and three jumper wires (lessons 5 and 8)
- Optional, a Modulino Thermo with a Qwiic cable, and a small hobby servo (lesson
  5)
- Optional, a USB webcam and a USB-C hub with power delivery (lessons 12 and 15)
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
  - section: The Linux side
    content:
    - name: Linux survival kit
      link: 10_linux_survival_kit.html
    - name: A web page for your project
      link: 11_a_web_page_for_your_project.html
    - name: Bricks, a camera and AI without the setup
      link: 12_bricks_camera_and_ai.html
    - name: Run at startup
      link: 13_run_at_startup.html
  - section: Showcase builds
    content:
    - name: An always-on AI agent
      link: 14_an_always_on_ai_agent.html
    - name: The chess camera
      link: 15_the_chess_camera.html
    - name: Summary
      link: 16_summary.html
---
