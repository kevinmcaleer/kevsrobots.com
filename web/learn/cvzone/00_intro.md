---
layout: lesson
title: Computer Vision with CVZone
author: Kevin McAleer
type: page
cover: /learn/cvzone/assets/banners/00_intro.jpg
date: 2023-10-07
next: 01_install.html
description: Detect faces, hands and body poses in real time on a Raspberry Pi with
  Python, OpenCV and CVZone
percent: 16
duration: 62
quiz_total: 16
date_updated: 2026-11-24
redirect_from:
- /learn/cvzone/00_video.html
navigation:
- name: Computer Vision on Raspberry Pi with CVZone
- content:
  - section: Overview
    content:
    - name: Computer Vision with CVZone
      link: 00_intro.html
  - section: Getting Started
    content:
    - name: Installing CVZone
      link: 01_install.html
  - section: Face, Hand and Pose Detection
    content:
    - name: Face Detection
      link: 02_face.html
    - name: Hand Detection
      link: 03_hand.html
    - name: Pose Detection
      link: 04_posture.html
    - name: Annotating Your Video
      link: 05_advanced.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

## Watch the video

{% include youtubeplayer.html id="FrLHFZZIozk" %}

---

## Overview

`Computer vision` lets a computer make sense of what a camera sees. In this course you'll use `CVZone`, a friendly Python library built on top of OpenCV and Google's MediaPipe, to detect faces, track hands and estimate body poses in real time on a Raspberry Pi. Each project is only a few lines of code, which makes it a great way into computer vision for robots, games and smart gadgets.

---

## Course Content

1. **Installing CVZone** - set up a Python virtual environment with OpenCV, MediaPipe and CVZone, and connect a camera
2. **Face detection** - find faces in a live video feed
3. **Hand detection** - track hands and count raised fingers
4. **Pose detection** - find 33 body landmarks and measure angles between them
5. **Annotating your video** - add live text and feedback to the picture

---

## Key Results

After completing this course, you will be able to:

* Set up a Raspberry Pi for computer vision with Python
* Capture video from a USB webcam or a Raspberry Pi camera module
* Detect faces, hands and body poses with CVZone
* Use landmark positions to measure distances and angles
* Overlay live feedback on the video stream

---

## What you'll need

* A Raspberry Pi 4 or 5 running the 64-bit version of Raspberry Pi OS, with a desktop
* A USB webcam or a Raspberry Pi camera module
* A keyboard, mouse and screen, or remote desktop access (the examples open a video window)

The Raspberry Pi 5 gives the smoothest frame rates, but a Pi 4 works well too.

---

## Prerequisites

You should be comfortable writing and running simple Python programs. If you're new to Python, start with [Python for beginners](/learn/python/). You'll also type a few commands in the terminal, so [Introduction to the Linux Command Line](/learn/linux_intro/) helps if that is new to you.

> **Teachers**: This course suits secondary school and older learners, or a club, who can already write simple Python. Allow about an hour per lesson, so five sessions in all, with the install lesson taking longest. You need one Raspberry Pi 4 or 5, with a screen and camera, per learner or per pair. The installs and the one-off model downloads need a decent internet connection, and a school network that blocks Google's download servers will stop the detectors from starting, so test one Pi first. Point a camera at pupils only with the permission your school requires: these programs only find faces and bodies in a live picture and do not save or identify anyone, but agree where the camera points before you start. Without Pi hardware, the code reading, quizzes and exercises in each lesson can still be done on paper or on a laptop with a webcam.

---

## How the course works

Each lesson builds a short, complete program you can run straight away, then explains how it works. Most lessons end with a quick quiz, and your progress is saved in your browser and shown in the sidebar.

---
