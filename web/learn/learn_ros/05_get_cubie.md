---
layout: lesson
title: Get Cubie-1 files
author: Kevin McAleer
type: page
cover: /learn/learn_ros/assets/banners/05_get_cubie.jpg
date: 2023-01-07
previous: 04_docker_install.html
next: 06_clone_images.html
description: Download the Cubie-1 example files
percent: 30
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

By the end of this lesson you will be able to download the Cubie-1 example files with `git` and say what the folders inside are for.

## Example files

Cubie-1 is the robot this series is building towards. Its repository holds the files we need to follow along: the `dockerfile` that describes our ROS 2 container, a `docker-compose.yml` that says how to run it, and a ready-made `docker-full` setup that includes RViz2 for the later lessons. Grabbing these now saves a lot of typing.

## Get Cubie-1 files

* **Clone the repository** - from the terminal, in your home folder, type:

```bash
cd ~
git clone https://github.com/kevinmcaleer/cubie-1
```

`git clone` copies the whole project, including its history, into a new `cubie-1` folder. Look inside:

```bash
ls cubie-1
```

You should see folders called `docker` and `docker-full`. The first builds the plain ROS 2 container we use for the next few lessons. The second also installs the desktop tools, including RViz2.

```mermaid
flowchart LR
    R[GitHub: cubie-1] -- git clone --> H[~/cubie-1 on the Pi]
    H --> A[docker/<br/>core ROS 2]
    H --> B[docker-full/<br/>ROS 2 + RViz2]
    H -. shared into container as .-> C[/ros2]
```

Later we'll map this folder into the container, so the files you write in the container are also stored safely on the Pi's SD card.

---

## Try it Yourself

1. Use `ls ~/cubie-1/docker` to list the files inside the `docker` folder. Which one is the `dockerfile`?
2. Use `cat` to read the `docker-compose.yml`. Can you guess what each line does?
3. Challenge: run `git log --oneline | head` inside `cubie-1`. How many changes has the project had?

## Common Issues

- **Problem**: `git: command not found`.
- **Solution**: Install it with `sudo apt install git -y`, then clone again.
- **Why**: Git isn't always included on a fresh install.

- **Problem**: `destination path 'cubie-1' already exists and is not an empty directory`.
- **Solution**: You've already cloned it. Use `cd cubie-1` and `git pull` to update it instead.
- **Why**: `git clone` refuses to overwrite an existing folder.

## Check your understanding

{% quiz %}
? What does `git clone` do?
- [ ] Deletes a repository
- [x] Copies a repository, with its history, to your machine
- [ ] Builds a Docker image
> It makes a local copy that you can read, edit and update.

? Which folder holds the version of the container that includes RViz2?
- [ ] `docker`
- [x] `docker-full`
- [ ] `assets`
> `docker-full` also installs the desktop tools, as the lesson says.

? Why clone Cubie-1 into the home folder?
- [x] Later commands refer to `~/cubie-1`
- [ ] Git only works in the home folder
- [ ] Docker requires it
{% endquiz %}
