---
layout: lesson
title: Build the ROS2 Container
author: Kevin McAleer
type: page
previous: 06_clone_images.html
next: 08_vscode_setup.html
description: Use the official docker images to build our ROS2 container
percent: 84
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


## Dockerfile & Docker-Compose.yml

To make the process of building our ROS2 container simpler I have created a `dockerfile` and accompanying `docker-compose.yml` file. These two files help us build and then run the ROS2 docker container.

---

## Build the container

* **Change directory** - From the terminal, type:

```bash
cd cubie-1/docker
```

* **Docker build** - From the terminal, type:

```bash
docker build -t ros2 .
```

*Where `ros2` is the name of the docker image we have created*

![Build output on a terminal](assets/build.png){:class="img-fluid w-50"}

---

## Run the container

* **Run the container** - From the terminal, type:

```bash
docker-compose up -d
```

*The container will now be running, you can check to see if its running using the `docker ps` command.*

---
