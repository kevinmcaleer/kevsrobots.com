---
layout: lesson
title: Clone Docker Images
author: Kevin McAleer
type: page
previous: 05_get_cubie.html
next: 07_build_container.html
description: Download the official docker images for ROS from the Open Source Robotics
  Foundation (OSRF)
percent: 49
duration: 1
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
---


![A list of Folders on the Raspberry Pi](assets/folders.jpg){:class="cover"}

## Open Source Robotics Foundation Official Docker files

The Open Source Robotics Foundation (OSRF) maintain a suite of official Docker container definitions. Lets grab these so that we can build our own container.

---

## Clone the official OSRF Docker Images

This step is optional, as I will provide a tested `dockerfile` and `docker-compose.yml`. However if you want to build a different version or release, these are the files you will need.

Download the official dockers images from the OSRF [github repository](https://www.github.com/osrf/docker_images):

* **Get official ROS docker images** - From the terminal, type:

```bash
git clone https://www.github.com/osrf/docker_images
```

---

## Summary

You should now have a number of extra folders on your Raspberry Pi:

![A list of Folders on the Raspberry Pi](assets/folders.jpg){:class="img-fluid w-50"}

* `docker-images` - contains the Official OSRF docker images
* `cubie-1` - contains the example files

---
