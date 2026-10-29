---
layout: lesson
title: Summary and next steps
type: page
description: What you've learned about ROS 2 and where to go next
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


## What you've learned

Congratulations! You've gone from a blank SD card to live LiDAR data in RViz2. Along the way you:

* Set up a Raspberry Pi with 64-bit Raspberry Pi OS and Docker
* Built and ran ROS 2 Humble in a Docker container, and connected to it from VS Code
* Created a workspace with `colcon` and wrote ROS 2 nodes in Python
* Built a package with a publisher and a subscriber talking over a topic
* Forwarded X11 so RViz2 could run in the container
* Used TF2 to place your LiDAR scans in the world and displayed them in RViz2

## What's next

* **Launch files** start several nodes with one command. Try writing one that starts the RPLidar node and the static transform together.
* **SLAM** (Simultaneous Localisation and Mapping) uses LiDAR scans to build a map while the robot works out where it is in it. The [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) package is a great place to start.
* Watch [They All Gave Up on ROS and MicroPython, I Didn't](https://www.youtube.com/watch?v=vbLEdFrCbho) to see where I've taken ROS since.

{% include youtubeplayer.html id="vbLEdFrCbho" %}

## More courses

* [Docker](/learn/docker/): get more out of containers
* [MicroPython Robotics Projects with the Raspberry Pi Pico](/learn/micropython_robotics/)
