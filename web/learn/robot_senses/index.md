---
layout: course_overview
title: Give Your Robot Senses
author: Kevin McAleer
cover: /learn/robot_senses/assets/cover.jpg
date: 2026-11-24
description: 'Give one small tracked robot a sense at a time. Learn what each sensor
  measures, how to read it in MicroPython, and how to turn the readings into behaviour.
  Then move up to navigation: wall following, and the ideas behind mapping and SLAM.'
level: beginner
course_duration: 1h 21m
lesson_count: 17
first_lesson: 00_intro.html
quiz_total: 10
video: 1K1A1YGcOEE
outcomes:
- Explain the sense, think, act loop and say which part of a robot does each job
- Connect Modulino sensors over Qwiic and find them with an I2C scan
- Read a distance sensor and a movement sensor in MicroPython, and handle a missing
  reading safely
- Turn sensor readings into behaviour with thresholds, hysteresis and priorities
- Build a reactive robot that stops, backs off, pulls faces and reacts when picked
  up
- Describe how wall following works, and how mapping and SLAM go further
prerequisites:
- Robotics 101
- Build a SMARS Robot
you_need:
- A SMARS robot, or any small robot with two motors
- An UnexpectedMaker FeatherS2 Neo running MicroPython, or another board with I2C
  and Qwiic
- Arduino Modulino Distance, Buzzer, Motors, LED Matrix and Movement, with Qwiic cables
- A 9V battery for the motors, and a LiPo battery for the board
- A computer with a MicroPython editor such as Thonny or Snakie
- Optional, an Arduino Alvik and some cardboard walls for lessons 13 and 14
navigation:
- name: Give Your Robot Senses
- content:
  - section: Introduction
    content:
    - name: Introduction
      link: 00_intro.html
  - section: How robots sense
    content:
    - name: How Robots Sense
      link: 01_how_robots_sense.html
    - name: Meet Modulino SMARS
      link: 02_meet_modulino_smars.html
    - name: Qwiic and I2C
      link: 03_qwiic_and_i2c.html
    - name: Your First Connection
      link: 04_first_connection.html
  - section: One sense at a time
    content:
    - name: See with the Distance Module
      link: 05_see_distance.html
    - name: Feel Movement with the IMU
      link: 06_feel_movement.html
    - name: Speak with the Buzzer
      link: 07_speak_the_buzzer.html
    - name: Show with the LED Matrix
      link: 08_show_the_led_matrix.html
    - name: Move with the Motors
      link: 09_move_the_motors.html
  - section: From readings to behaviour
    content:
    - name: From Readings to Decisions
      link: 10_readings_to_decisions.html
    - name: Combining Senses
      link: 11_combining_senses.html
    - name: 'Project: The Reactive Robot'
      link: 12_project_reactive_robot.html
  - section: Navigation
    content:
    - name: Wall Following
      link: 13_wall_following.html
    - name: From Wall Following to Mapping and SLAM
      link: 14_mapping_and_slam.html
  - section: Wrapping up
    content:
    - name: Troubleshooting
      link: 15_troubleshooting.html
    - name: Summary
      link: 16_summary.html
---
