---
layout: lesson
title: Laserscan data
author: Kevin McAleer
type: page
previous: 17_tf2.html
description: Lets Visualise our Laser scan data
percent: 100
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


## Visualise Laserscan

It's finally time to visualise the laser scan data.

* **Add Laser_scan topic display** - In rviz2, click the `add` button

* **Select Laser_scan** - Scroll down the list of displays types and select the `laser scan` and click `ok`

![Create Display](assets/rviz04.png){:class="img-fluid w-50 shadown-lg"}

---

* **Set the settings** - Click disclose triangle next to `LaserScan` to open it up

![Create Display](assets/rviz05.png){:class="img-fluid w-50 shadown-lg"}

* **Change topic to scan** - Change the topic to `/scan`

---

* **Change Global Options topic to World** - Change the Global Options, Fixed Frame topic to `world`

![Create Display](assets/rviz06.png){:class="img-fluid w-50 shadown-lg"}

---

🥳 you should now be able to see the Lidar Data in the middle panel

![Create Display](assets/rviz2.jpg){:class="img-fluid w-100 shadown-lg"}

---
