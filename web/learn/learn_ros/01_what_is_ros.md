---
layout: lesson
title: What is ROS
type: page
description: Learn about ROS, the different versions and what it does
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


## What is ROS?

Here is the definition from the creators of ROS - <https://ros.org/>

> *"The Robot Operating System (ROS) is a set of software libraries and tools that help you build robot applications. From drivers to state-of-the-art algorithms, and with powerful developer tools, ROS has what you need for your next robotics project. And it's all open source."*

---

## Which version should I use

There are two versions of ROS:

* **ROS 1** - The original version of ROS. Its last release, Noetic, reached end of life in May 2025.
* **ROS 2** - The current version of ROS, and what we will be using in this course.

---

## Releases & Distros

ROS 2 has a new release (called a *distro*) every May. The releases in even years are Long Term Support (LTS) releases with five years of updates. This course uses **Humble Hawksbill**, which is supported until May 2027 and is what the example Docker files are built on. If you're starting a new project, check the [list of ROS 2 distributions](https://docs.ros.org/en/rolling/Releases.html) for the newest LTS release; the code in this course works the same way.

Distro              |     Release date |   End Of Life date
:-------------------|-----------------:|------------------:
Jazzy Jalisco (LTS) |     May 23, 2024 |           May 2029
Iron Irwini         |     May 23, 2023 |      December 2024
**Humble Hawksbill (LTS)** | **May 23, 2022** | **May 2027**
Foxy Fitzroy        |     June 5, 2020 |           May 2023
{:class="table table-striped"}

---

## Programming languages

ROS2 Supports both `C++` and `Python` out of the box (and I'm sure there are other languages adapted to use it too), however for this course we will be focusing on `Python`.

---

## Check your understanding

{% quiz %}
? Which ROS 2 distro does this course use?
- [ ] Foxy Fitzroy
- [x] Humble Hawksbill
- [ ] Iron Irwini
> Humble is a Long Term Support release, supported until May 2027.

? Which two programming languages does ROS 2 support out of the box?
- [x] C++ and Python
- [ ] Java and JavaScript
- [ ] Rust and Go
{% endquiz %}
