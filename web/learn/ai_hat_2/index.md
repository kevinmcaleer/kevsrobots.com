---
layout: course_overview
title: Raspberry Pi AI HAT+ 2 - Vision and Generative AI
author: Kevin McAleer
cover: /learn/ai_hat_2/assets/cover.jpg
date: 2026-11-24
description: Give a robot eyes and a brain with the Raspberry Pi AI HAT+ 2. Fit the
  HAT, run object detection and pose estimation from the camera, turn detections into
  robot behaviour, then run a language model and a vision-language model on the HAT
  itself.
level: intermediate
course_duration: 0h 47m
lesson_count: 14
first_lesson: 00_intro.html
quiz_total: 6
video: lT1NJfkuiU8
outcomes:
- Fit the AI HAT+ 2 to a Raspberry Pi 5 and check that it is working
- Run camera object detection and pose estimation with rpicam-apps
- Start a Hailo camera pipeline from Python and know where to find the detection data
- Turn detections into robot behaviour, such as a person-following decision
- Run a language model on the HAT with hailo-ollama and talk to it from a terminal,
  Python or a browser
- Choose between the AI Camera, the AI HAT+ and the AI HAT+ 2 for a project
prerequisites:
- Introduction to the Linux Command Line on Raspberry Pi OS
- Computer Vision on Raspberry Pi with CVZone
you_need:
- A Raspberry Pi 5
- A Raspberry Pi AI HAT+ 2
- A supported Raspberry Pi camera, such as Camera Module 3
- 64-bit Raspberry Pi OS (Trixie) on a microSD card, and a power supply for the Pi
  5
- A Raspberry Pi Active Cooler (recommended by the docs)
- A Phillips crosshead screwdriver
- Optional, a monitor and keyboard, or SSH access from another computer
- Optional, Docker, for the Open WebUI chat page (lesson 9)
navigation:
- name: Raspberry Pi AI HAT+ 2 - Vision and Generative AI
- content:
  - section: Meet the HAT
    content:
    - name: Introduction
      link: 00_intro.html
    - name: What is the AI HAT+ 2?
      link: 01_what_is_the_ai_hat_2.html
    - name: Fitting the HAT
      link: 02_fitting_the_hat.html
    - name: Software setup
      link: 03_software_setup.html
  - section: Eyes
    content:
    - name: Object detection with rpicam-apps
      link: 04_object_detection_with_rpicam_apps.html
    - name: Pose and segmentation
      link: 05_pose_and_segmentation.html
    - name: Reading detections from Python
      link: 06_reading_detections_in_python.html
    - name: Detections to robot behaviour
      link: 07_detections_to_robot_behaviour.html
  - section: A brain
    content:
    - name: A language model on the HAT
      link: 08_a_language_model_on_the_hat.html
    - name: A chat page in your browser
      link: 09_a_chat_page_in_your_browser.html
    - name: Asking the robot what it sees
      link: 10_asking_the_robot_what_it_sees.html
  - section: Choosing and fixing
    content:
    - name: Performance and choosing hardware
      link: 11_performance_and_choosing_hardware.html
    - name: Troubleshooting
      link: 12_troubleshooting.html
    - name: Summary
      link: 13_summary.html
---
