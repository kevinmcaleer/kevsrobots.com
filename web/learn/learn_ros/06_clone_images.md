---
layout: lesson
title: Clone Docker Images
author: Kevin McAleer
type: page
cover: /learn/learn_ros/assets/banners/06_clone_images.jpg
date: 2023-01-07
previous: 05_get_cubie.html
next: 07_build_container.html
description: Download the official docker images for ROS from the Open Source Robotics
  Foundation (OSRF)
percent: 35
duration: 3
quiz_total: 58
date_updated: 2026-11-24
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
    - name: Nodes as Classes
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


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to explain where the official ROS Docker images come from, and optionally download their definitions.

## Where do ROS images come from?

The Open Source Robotics Foundation (OSRF) maintains the official ROS Docker images. They're published on Docker Hub as `ros`, and the tag says which distro and how much is installed. For example `ros:humble-ros-base` is ROS 2 Humble with the core packages, and `ros:humble-ros-core` is even smaller. The Cubie-1 `dockerfile` starts `FROM ros:humble-ros-core-jammy`, where `jammy` is the Ubuntu 22.04 release name, and then adds the build tools we need.

The recipes that build those images live in a public repository. Cloning them is **optional**: Cubie-1 already has a tested `dockerfile` and `docker-compose.yml`. But it's worth looking if you ever want a different distro or a different set of packages.

## Clone the official OSRF Docker images

* **Get the official ROS docker images** - from the terminal, in your home folder, type:

```bash
cd ~
git clone https://github.com/osrf/docker_images
```

The repository includes a folder per image. Read a `Dockerfile` or two and you'll see how the pieces are put together.

## Summary

You should now have extra folders in your home folder on the Raspberry Pi:

![A list of Folders on the Raspberry Pi](assets/folders.jpg){:class="img-fluid w-50"}

* `docker_images` - the official OSRF Docker definitions (the folder uses an underscore)
* `cubie-1` - the example files

```mermaid
flowchart LR
    A[OSRF docker_images<br/>recipes] --> B[Docker Hub: ros]
    B --> C[ros:humble-ros-core-jammy]
    C --> D[Cubie-1 dockerfile<br/>adds build tools + ros-base]
    D --> E[Your container]
```

---

## Try it Yourself

1. Run `ls ~/docker_images` to see what's inside.
2. Open the `dockerfile` in `~/cubie-1/docker` and find the `FROM` line. Which image does it start from?
3. Challenge: browse the `docker_images` folder and work out where the Humble images live.

## Common Issues

- **Problem**: `cd docker-images` says no such file or directory.
- **Solution**: The folder is `docker_images`, with an underscore.
- **Why**: Git names the folder after the repository.

- **Problem**: Which image tag should I use?
- **Solution**: For a robot, `humble-ros-base` is a good start; add `-desktop` tools only on machines with a screen.
- **Why**: The smaller the image, the less SD card space and RAM it uses.

## Check your understanding

{% quiz %}
? What does the `jammy` in `ros:humble-ros-core-jammy` stand for?
- [x] The Ubuntu 22.04 release the image is built on
- [ ] The name of the robot
- [ ] The ROS 1 version
> Ubuntu releases have code names, and Jammy is 22.04.

? Is cloning `docker_images` required to follow this course?
- [ ] Yes, the build fails without it
- [x] No, Cubie-1 already includes a tested dockerfile
> It's optional, useful if you want a different distro.

? What are the official ROS images published as on Docker Hub?
- [x] `ros`
- [ ] `ros-official`
- [ ] `cubie`
{% endquiz %}
