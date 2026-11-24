---
layout: lesson
title: Summary
author: Kevin McAleer
type: page
cover: /learn/ai_hat_2/assets/banners/13_summary.jpg
date: 2026-11-24
previous: 12_troubleshooting.html
description: What you have built, what to remember, and where to go next with the
  AI HAT+ 2
percent: 100
duration: 2
quiz_total: 6
date_updated: 2026-11-24
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


![Course Cover Image]({{page.cover}}){:class="cover"}

---

You gave a robot eyes and a brain. Let's recap what you can now do.

## What you learned

- The AI HAT+ 2 carries a Hailo-10H (40 TOPS INT4) and its own 8 GB of memory, and it is the one board in the family that runs LLMs and VLMs
- How to fit the HAT, install `hailo-h10-all`, and check it with `hailortcli`
- How to run object detection, pose estimation and segmentation with `rpicam-hello` and the Hailo post-process files
- Where to find the official Python routes to detection data, and how to start a pipeline from Python
- How to turn detections into a person-following decision in plain Python
- How to run `hailo-ollama`, talk to it from the terminal and Python, and chat in the browser with Open WebUI
- How to turn a vision-language description into a cautious decision
- How to choose between the AI Camera, the AI HAT+ and the AI HAT+ 2, and read a review's numbers sensibly

## Key points to remember

- Follow the official docs for commands, because the software moves quickly
- Keep your decision code separate from your detector, so you can swap either one
- Don't let a language model be the only thing between your robot and a wall
- Fit the heatsink and fan for heavy work

## Where to go next

- [Computer Vision on Raspberry Pi with CVZone](/learn/cvzone/), for more camera tricks
- [Building Object Detection Models with Raspberry Pi AI Camera](/learn/object_model/), to train your own detector
- [Reinforcement learning](/learn/reinforcement_learning/), to let a robot learn behaviour by trial and error
- [Learn ROS](/learn/learn_ros/), to wire your detections into a bigger robot system

Thank you for building along. Now go and give a robot some eyes, and see what it says about what it sees.
