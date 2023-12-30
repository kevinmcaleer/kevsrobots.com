---
layout: lesson
title: Rviz2
author: Kevin McAleer
type: page
cover: /learn/learn_ros/assets/rviz2.jpg
date: 2023-01-07
previous: 14_docker_x11.html
next: 16_rviz_displays.html
description: Lets learn about RViz2, the ROS Visualisation tool
percent: 80
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


![RViz2 screenshot]({{page.cover}}){:class="cover"}

## What is RViz2?

[`RViz2`](/resources/glossary#rviz2) is a powerful, user-friendly visualization tool that allows you to explore and analyze data in 3D. It is capable of displaying a variety of objects including robots, cameras, geometry, point clouds, and more. RViz2 is an open source tool that is available for free and is compatible with the ROS (Robot Operating System) platform.

---

## How can we use RViz2?

We can use RViz2 to display the data from our LiDAR, and then take this further by adding in extra `nodes` such as the Mapping and Navigation nodes from the [SLAM](/resources/glossary#SLAM) Toolkit.

---

RViz2 is included in the full desktop version of ROS2, initially we only used the `core` version of ROS2. Its easy to change this, in the `dockerfile` you can change the line:

```docker
FROM ros:humble-ros-core-jammy
```

to the line:

```docker
FROM ros:humble-ros-desktop-jammy
```

---

## Rebuild Docker Container

We need to rebuild the docker-container so that it now has the settings.

* **Rebuild the container** - From the Raspberry Pi terminal, type:

```bash
docker-compose build --no-cache
docker-compose up -d
```

---

## Console into the container

Lets test out `RViz2`.

* **launch Bash in Container** - From the Raspberry Pi terminal, type:

```Bash
docker exec -it docker-full_ros2_1 bash
```

---

## Launch Rviz2

* **launch RViz2** - From the Raspberry Pi terminal, type:

```Bash
rviz2
```

RViz should now open within the Raspberry Pi OS - this is because the container is outputing its X11 graphics commands to the Host (the Raspberry Pi OS).

---
