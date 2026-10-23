---
layout: lesson
title: Summary and next steps
type: page
description: What you've learned about ROS 2 and where to go next
---

## What you've learned

Congratulations! You've gone from a blank SD card to live LiDAR data in RViz2. Along the way you:

* Set up a Raspberry Pi with 64-bit Raspberry Pi OS and Docker
* Built and ran ROS 2 Humble in a Docker container, and connected to it from VS Code
* Created a workspace with `colcon` and wrote ROS 2 nodes in Python
* Built a package with a publisher and a subscriber talking over a topic
* Forwarded X11 so RViz2 could run in the container
* Used TF2 to place your LiDAR scans in the world and displayed them in RViz2

## What's next

* **Launch files** start several nodes with one command. Try writing one that starts the RPLidar node and the static transform together.
* **SLAM** (Simultaneous Localisation and Mapping) uses LiDAR scans to build a map while the robot works out where it is in it. The [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) package is a great place to start.
* Watch [They All Gave Up on ROS and MicroPython, I Didn't](https://www.youtube.com/watch?v=vbLEdFrCbho) to see where I've taken ROS since.

{% include youtubeplayer.html id="vbLEdFrCbho" %}

## More courses

* [Docker](/learn/docker/): get more out of containers
* [MicroPython Robotics Projects with the Raspberry Pi Pico](/learn/micropython_robotics/)
