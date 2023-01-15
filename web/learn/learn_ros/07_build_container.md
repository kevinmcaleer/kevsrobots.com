---
layout: lesson
title: Build the ROS2 Container
author: Kevin McAleer
type: page
previous: 06_clone_images.html
next: 08_vscode_setup.html
description: Use the official docker images to build our ROS2 container
percent: 56
duration: 3
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
      link: 10_talker.html
  - section: ROS2 Python Packages
    content:
    - name: Create a ROS2 Python Package
      link: 11_create_py_package.html
    - name: Create a ROS2 Python Publisher
      link: 12_create_publisher.html
    - name: Create a ROS2 Python Subscriber
      link: 13_create_subscriber.html
---


## Dockerfile & Docker-Compose.yml

To make the process of building our ROS2 container simpler I have created a `dockerfile` and accompanying `docker-compose.yml` file. These two files help us build and then run the ROS2 docker container.

> ### Docker-Compose.yml
>
> The `docker-compose.yml` file contains instructions on which docker image to use and also any parameters we want to include, such as host device (we'll want to use the lidar sensor which is found at `/dev/ttyUSB0` on the host file system), and also any volume or folder mappings.
>
> **Contents of the `docker-compose.yml`**
>
> ```docker
> version: "3.9"
> services:
>  ros2:
>    build: .
>    # restart: no
>    ports: 
>      - "3332:3332"
>    volumes:
>      - /home/kev/ros:/home/ros
>      - /home/kev/cubie-1:/ros2
>    devices:
>      - /dev/ttyUSB0:/dev/ttyUSB0
>    tty: true
> ```
>
> ---
>
> ### Folder mappings
>
> We can map local folders from our Raspberry Pi into our container so that any files we create inside or out of the container will show up.  Normally any changes made within the container are lost when we shut it down, however by mapping (called `binding` in Docker) the local host folder to the container we can ensure changes are retained.
>
> We will map two host folders:
>
> `/home/kev/ros` to `/home/ros` within the container
>
> `/home/kev/cubie-1` to `/ros2` within the container
>
{:class="bg-blue"}

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
