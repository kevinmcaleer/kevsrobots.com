---
layout: lesson
title: Introduction
author: Kevin McAleer
type: page
next: 01_what_is_ros.html
description: Get started with ROS, the Robot Operating System and learn how to build
  robots using this industry standard framework
percent: 5
duration: 4
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
---


![Screenshot of the imager tool](assets/ros_background.jpg){:class="cover"}

## Overview

This course will teach you all about the Robot Operating System, how to use the ROS2 on a Raspberry Pi 4, and how to write your own ROS programs.

---

## What you'll learn

In this course you will learn:

* How to setup a Raspberry Pi for use with ROS
* How to install docker on a Raspberry Pi
* Setting up the ROS environment within a docker container
* ROS versions
* Creating a ROS workspace with Colcon
* ROS Commandline tools
    * colcon
    * ros2
    * ros nodes list
    * ros pkg list
    * qrt
* Creating a simple ROS program
* Creating a package
* Creating launch files
* Visualising nodes with qrt
* TL
* LIDAR
    * Laser Scan data
    * SLAM

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
A Raspberry Pi 4 | 4Gb or 8Gb (recommended)
A Lidar sensor   | A1 LiDAR sensor from Slamtec
{:class="table table-striped"}

ROS can be installed on Windows, macOS or Linux; however for robotics we often want to install this on a single board computer such as the **Raspberry Pi 4**. In fact ROS.org supports ROS on Raspberry Pi 4 running the Ubuntu OS, or the standard 64 bit version of the Raspberry Pi OS and running ROS within a Docker container; this is what we will use in this course.

The 8Gb version of the Raspberry Pi 4 is recommended as ROS is quite memory hungry, and the Raspberry Pi cannot use a large swap file as it damages the SD card over time (due to the large number of writes to the flash storage).

---
