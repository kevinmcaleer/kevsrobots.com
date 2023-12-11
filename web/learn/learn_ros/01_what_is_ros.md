---
layout: lesson
title: What is ROS
author: Kevin McAleer
type: page
cover: /learn/learn_ros/assets/learn_ros.jpg
previous: 00_intro.html
next: 02_pi_setup.html
description: Learn about ROS, the different versions and what it does
percent: 10
duration: 2
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


## What is ROS?

Here is the definition from the creators of ROS - <https://ros.org/>

> *"The Robot Operating System (ROS) is a set of software libraries and tools that help you build robot applications. From drivers to state-of-the-art algorithms, and with powerful developer tools, ROS has what you need for your next robotics project. And it's all open source."*

---

## Which version should I use

There are two versions of ROS:

* **ROS** - The original version of ROS
* **ROS2** - The latest version of ROS - this is what we will be using, as the older version is depreciated.

---

## Releases & Distros

ROS2 has several releases, as of the writing of this the **Iron Irwini**, however this is only supported up to Nov 2024,  **Humble Hawksbill** is the release version with Long term support (LTS):

Distro              |     Release date |   End Of Life date
:-------------------|-----------------:|------------------:
**Iron Irwini**     | **May 23, 2023** |       **Nov 2024**
Humble Hawksbill    |   May 23rd, 2022 |           May 2027
Galactic Geochelone |   May 23rd, 2021 | December 9th, 2022
Foxy Fitzroy        |   June 5th, 2020 |           May 2023
{:class="table table-striped"}

---

## Programming languages

ROS2 Supports both `C++` and `Python` out of the box (and I'm sure there are other languages adapted to use it too), however for this course we will be focusing on `Python`.

---
