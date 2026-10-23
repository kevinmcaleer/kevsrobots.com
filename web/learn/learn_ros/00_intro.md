---
layout: lesson
title: Introduction
type: page
description: Get started with ROS, the Robot Operating System and learn how to build
  robots using this industry standard framework
navigation:
- name: Learn ROS with me
- content:
  - section: Overview
    content:
    - name: Introduction
      link: 00_intro.html
    - name: What is ROS
      link: 01_what_is_ros.html
  - section: Setting up the Raspberry Pi 4 environment
    content:
    - name: Raspberry Pi Setup
      link: 02_pi_setup.html
    - name: Installing updates
      link: 03_install_updates.html
  - section: Setting Docker for ROS
    content:
    - name: Docker Install
      link: 04_docker_install.html
    - name: Get Cubie-1 files
      link: 05_get_cubie.html
    - name: Clone Docker Images
      link: 06_clone_images.html
  - section: Build the ROS2 Container
    content:
    - name: Build the ROS2 Container
      link: 07_build_container.html
  - section: Setting up VSCode
    content:
    - name: VSCode setup
      link: 08_vscode_setup.html
  - section: First ROS2 Program
    content:
    - name: First ROS2 Program
      link: 09_first_ros_program.html
    - name: Talker ROS2 Program
      link: 10_listener.html
  - section: ROS2 Python Packages
    content:
    - name: Create a ROS2 Python Package
      link: 11_create_py_package.html
    - name: Create a ROS2 Python Publisher
      link: 12_create_publisher.html
    - name: Create a ROS2 Python Subscriber
      link: 13_create_subscriber.html
  - section: Docker and X11
    content:
    - name: Getting Rviz2 and RQT to work in docker
      link: 14_docker_x11.html
  - section: RViz2
    content:
    - name: Rviz2
      link: 15_rviz.html
    - name: Rviz2 Displays
      link: 16_rviz_displays.html
    - name: Transformations with TF2
      link: 17_tf2.html
    - name: Laserscan data
      link: 18_laserscan.html
    - name: Summary and next steps
      link: 19_summary.html
---


![ROS background](assets/ros_background.jpg){:class="cover"}

## Watch the video

This course follows my "Learn ROS with me" video series. Part 1 shows the robot we're working towards.

{% include youtubeplayer.html id="_cTJhCRv858" %}

---

## Overview

This course will teach you all about the Robot Operating System, how to run ROS 2 on a Raspberry Pi, and how to write your own ROS programs in Python.

---

## Course Content

In this course you will learn:

* How to set up a Raspberry Pi for use with ROS
* How to install Docker on a Raspberry Pi
* Setting up the ROS environment within a Docker container
* ROS versions and distributions
* Creating a ROS workspace with `colcon`
* ROS command line tools such as `ros2 run` and `ros2 topic`
* Writing ROS 2 nodes in Python
* Creating a package with a publisher and a subscriber
* Visualising LiDAR laser scan data in RViz2, using TF2 transforms

---

## Key Results

By the end of this course you will be able to:

* Run ROS 2 in a Docker container on a Raspberry Pi
* Write, build and run your own ROS 2 Python nodes
* Explain what nodes, topics, publishers and subscribers are
* Display live data from a LiDAR sensor in RViz2

---

### About ROS

ROS (Robot Operating System) is an open-source, meta-operating system for your robot. It provides the tools and libraries that allow you to quickly create and deploy robotics applications. ROS provides a flexible framework for writing and executing code, as well as a wide variety of libraries and tools for navigation, manipulation, and perception. ROS is used by many leading robotics companies and research institutions worldwide.

---

### Don't be scared

 ROS can be very intimidating at first, but don't worry - it's actually quite manageable once you understand the basics. This course is a great starting point for unpacking and learning how ROS works.

---

### What you'll need

Item             | Description
:----------------|-----------------------------
A Raspberry Pi 4 or 5 | 4GB or 8GB (recommended)
A LiDAR sensor   | RPLidar A1 LiDAR sensor from Slamtec (only needed for the last section)
A computer with VS Code | To write code on the Pi remotely
{:class="table table-striped"}

ROS can be installed on Windows, macOS or Linux; however for robotics we often want to install this on a single board computer such as the **Raspberry Pi**. ROS supports the Raspberry Pi running Ubuntu, or you can run the standard 64-bit Raspberry Pi OS and run ROS inside a Docker container; this is what we will use in this course.

The 8GB version of the Raspberry Pi is recommended as ROS is quite memory hungry, and the Raspberry Pi cannot use a large swap file as it damages the SD card over time (due to the large number of writes to the flash storage).

---

## How the course works

Each lesson is a set of steps to follow on the Raspberry Pi or in VS Code, with commands to type shown like `this`. The four videos in the series appear in the lessons they match. There's a short quiz at the end of each section.

## Prerequisites

* Comfortable typing commands in a terminal; [Introduction to the Linux Command Line](/learn/linux_intro/) covers this
* Some Python; [Python for beginners](/learn/python/) covers everything you need
* It helps to have seen [Docker](/learn/docker/) before, but it isn't required
