---
layout: lesson
title: Installing updates
author: Kevin McAleer
type: page
previous: 02_pi_setup.html
next: 04_docker_install.html
description: Update and Upgrade the OS to the latest patch level
percent: 28
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


![Screenshot of the imager tool](assets/rpi_updates.jpg){:class="cover"}

## Install any updates for the Raspberry Pi

* **Connect to the Pi** - Once the Raspberry Pi 4 has booted up, either attach a keyboard, mouse and monitor, or remotely via ssh
* **Open a terminal** - and type:

```bash
sudo apt update
sudo apt upgrade
```

### Alternatively you can use the Raspberry Pi Update menu to install updates.

* **Click on the update icon** - next to the clock; it may not appear immediately after startup.

![Screenshot of the imager tool](assets/rpi_updates_menu.jpg){:class="img-fluid w-100"}

---
