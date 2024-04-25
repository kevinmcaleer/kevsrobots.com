---
title: Guiding Light
description: >-
    Mapping with LiDAR using a Raspberry Pi Zero 2W, RPLidar, and Rviz
layout: showcase
date: 2024-04-25
author: Kevin McAleer
difficulty: beginner
cover: /assets/img/blog/lidar/cover.png
hero: /assets/img/blog/lidar/hero.png
mode: light
tags:
  - raspberry pi
  - file syncing
  - syncthing
groups:
  - robots
  - raspberrypi
videos:
  - yjc-bpJVaHo

---

## Hey Robot Makers!

Have you ever wondered about utilizing a LiDAR (Light Detection and Ranging) to facilitate your robot's understanding and mapping of the surrounding environment? Is integrating LiDAR with a Raspberry Pi Zero 2 on your radar? If so, I've got some good news for you- it's feasible! Today, we are commencing an exciting exploration into the universe of LiDAR mapping. So, come along with me, Kevin, as we delve into the world of robot building, coding, and of course, plenty of fun.

These are the show notes that accompany the [video](https://youtube.com/watch?v={{page.videos}}) above.

---

## Breaking Down LiDAR Mapping

The heartbeat of our conversation today is RPLidar, a LiDAR unit I have personally grown fond of. This gadget has found its home in my most recent robot project, one I can't wait to show off. So, what can we expect from today's session? The agenda includes a detailed look at how the RPLidar unit operates, the functionality of LiDAR, and installing Ubuntu on a Raspberry Pi zero. Additional topics include getting ROS (Robot Operating System) up and running, creating a swap file to augment memory, and taking the LiDAR out for a spin.

---

## Understanding the LiDAR Unit

Let's first tackle the LiDAR unit, specifically the RP LiDAR I'm currently using. Crafted by Slamtech, the RP LiDAR is a 360-degree distance sensor that costs around $89. It can rotate between 5 and 10 hertz, and guarantees a range of up to 8 meters. The unit can take between 2000 and 4000 samples per second and connects through UART or USB.

But the fundamental question remains: How does LiDAR function? Think of it as a rotating rangefinder. The device emits light that bounces off objects and returns to the sensor 40,000 times every second. The time taken for the light to reflect back to the sensor informs the LiDAR about the object's distance. 

[Read more about LiDAR](/resources/how_it_works/lidar).

---

## Setting Up Your Hardware

Now that you have a basic understanding of how LiDAR works let's move on to setting up your hardware. The first step calls for installing Ubuntu 20.04 on your Raspberry Pi Zero. Note that ROS isn't compatible with Raspberry Pi OS, so Ubuntu 20.04 is the preferred choice for this project.

One crucial detail to remember is that our Raspberry Pi Zero only has half a gig of RAM. Consequently, we will need to implement a swap file to supplement the memory. However, a word of caution: the swap file operation may lead to the degradation of your SD card due to extensive read/write operations. So, it's best to use high-quality, endurance SD cards to perform this operation.

The RPLidar simply plugs into the Raspberry Pi's USB port. The device is powered by the Raspberry Pi, so no additional power source is required.

---

## Installing ROS and RP LiDAR Software

With Ubuntu up and running and sufficient memory in place, we can now install ROS Noetic Ninjemys, our preferred version of ROS for this project. The step-by-step installation instructions can be accessed on the ROS website. To make our LiDAR operational, we'll also have to clone the ROS package for RP Lidar from GitHub. It's important to remember the name of the LiDAR unit's topic being broadcast on ROS, as it will be instrumental when visualizing data.

[Learn more about ROS and how to install it](/learn/learn_ros/).

---

## Visualizing Your Data with RViz

Once all is set and running, we can finally visualize our LiDAR data. This is where the ROS Visualizer, RViz, comes in handy. The setup includes creating a new ROS ecosystem, running the RP LiDAR software, and setting up the coordinate systems. Finally, by launching RViz, we can now view the LiDAR data in a 2D graphic.

---

## Closing Thoughts

Understanding and implementing LIDAR mapping using a Raspberry Pi Zero provides an insightful look into the world of robotics. It reinforces the limitless potential packed into these tiny components. If you want to further explore the concept, the next step would be experimenting with Hector SLAM, where location and mapping occur simultaneously. But that's a story for another day.

Stay curious, RoboMakers! Whether you're a beginner or an experienced tinkerer, the exciting realm of robotics awaits you. As always, feel free to ask questions or share your projects. Until next time, happy robot building!

---
