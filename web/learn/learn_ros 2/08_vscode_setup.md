---
layout: lesson
title: VSCode setup
author: Kevin McAleer
type: page
previous: 07_build_container.html
description: Install the VSCode extensions so we can remotely connect to the Raspberry
  Pi and Docker container instances
percent: 100
duration: 1
navigation:
- name: Learn ROS with me
- content:
  - section: Overview
    content:
    - name: Introduction
      link: 00_intro.html
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
---


## VSCode Remote connection

Microsoft VSCode can connect to a remote computer, such as our Raspberry Pi, over SSH and then connect to running contains on that remote machine so that we can write, run and debug our ROS2 programs.

We will need to install some extensions first, then connect to our ROS2 container on the Raspberry Pi.

---

## Install Extensions

Lets install the two docker extensions on our main computer (not the Raspberry Pi), that we will use to write, run and debug code.

![VS Code Extentions](assets/vscode_extensions.png){:class="img-fluid w-50"}

* **Click on the Extensions Icon** - This will list installed and installable extensions
* **Type `docker` in the search bar** - This will show installable docker extensions
* **Click Install on the `Docker` extension** - This extension is published by Microsoft

## Remote connect

We can now connect to the Raspberry Pi 4 running our new docker container by using SSH.

* **Open Command Palette** - Hold `CTRL + SHIFT + P`


## Connect to the container