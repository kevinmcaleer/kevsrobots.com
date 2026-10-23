---
layout: lesson
title: Rviz2 Displays
type: page
description: Let's learn about RViz2 Displays and setting it up for Laser scans
cover: /learn/learn_ros/assets/rviz03.jpg
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


![RViz2 screenshot]({{page.cover}}){:class="cover"}

---

## RViz2 Screen Layout

RViz has 3 main panels by default, and you can add additional panels to suit your needs (it's very flexible).

![RViz2 Screen Layout](assets/rviz03.jpg){:class="img-fluid w-100 shadow-lg"}

In the screenshot above you can see the main areas of the screen:

1. The Displays panel (outlined in Red)
1. The Main Camera Display (outlined in Blue)
1. The Views panel (outlined in Green)

---

> ## RPLidar Laser scanner
>
> The SlamTec RPLidar A1 is an affordable laser scanner, ideal for use with robotics, and costs less than £99. We will use this in our tutorial.
> ![RPLidar factsheet](/assets/img/how_it_works/lidar01.jpg){:class="img-fluid w-100 shadow-lg"}
>

---

## Adding our LiDAR Laser Scan

To view our LiDAR data in Rviz2 we need to do a couple of actions:

* Download and compile the RPLidar_ros code
* Start the RPLidar node
* Start a transformation node to translate the `laser_scan` coordinates to the `World` coordinates
* Start `rviz2` and add the laser_scan display

---

## Download and compile the RPLidar_ros code

* **Download RPLidar code** - From the Raspberry Pi terminal, type:

```bash
cd ~/cubie-1
git clone https://github.com/babakhani/rplidar_ros2
```

> Slamtec now maintain their own ROS 2 driver, [sllidar_ros2](https://github.com/Slamtec/sllidar_ros2), which also supports their newer LiDARs. It's started with `ros2 launch sllidar_ros2 view_sllidar_a1_launch.py`.

> The `rplidar_ros2` folder needs to be inside the `cubie-1` folder, which appears as `/ros2` inside the container.

* **Compile the RPLidar code** - From the container, type:

```bash
cd /ros2/rplidar_ros2
colcon build
source install/setup.bash
```

---

## Start the RPLidar node

We can now start the Laser scanner node.

* **Start a new container terminal session** - from the Raspberry Pi terminal, type:

```bash
cd ~/cubie-1/docker-full
docker compose exec ros2 bash
```

---

* **Start the Laser Scanner** - from the container, type:

```bash
cd /ros2/rplidar_ros2
source install/setup.bash
ros2 run rplidar_ros rplidar_composition
```

The RPLidar will momentarily stop and then restart, and you will see some output on the terminal showing that the LiDAR is now up and running

---

## Check the scan data topic exists

We can check that the `/scan` topic exists by using the `ros2 topic list` command. Let's open a new terminal and check we can see it.

* **Start a new container terminal session** - from the Raspberry Pi terminal, type:

```bash
cd ~/cubie-1/docker-full
docker compose exec ros2 bash
```

* **Ros2 topic list** - from the container, type:

```bash
ros2 topic list
```

There should be three topics listed:

```bash
/parameter_events
/rosout
/scan
```

If the `/scan` topic is visible, this means the LiDAR is successfully sending data to the Ros topic named `/scan`.

---

## Start RViz2

Let's start RViz2.

* **Start RViz2** - from the container, type:

```bash
rviz2
```

We will configure RViz in the next section.

---
