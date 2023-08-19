---
layout: how_it_works
title: SLAM
short_title: How it works - SLAM
short_description: Learn about Simultaneous Location and Mapping
date: 2023-01-14
author: Kevin McAleer
excerpt: 
cover: /assets/img/how_it_works/slam01.jpg
tags:
 - Robot
 - Tips
 - SLAM
 - ROS
 - LiDAR
 - Sonar
---

`SLAM` (`Simultaneous Localization and Mapping`) is an collection of algorithms used in robotics for navigation and mapping. It works by using [LIDAR](/resources/glossary#lidar), sonar and other sensor data to construct a 3D map of the environment and then using this map to localize the robot within it.

LIDAR (Light Detection and Ranging) is a sensing technology that uses lasers to measure distances to nearby objects by timing how long it takes for the laser to return after being emitted. 

The LIDAR data is used to construct a 3D point cloud of the environment which is then used to build an occupancy grid map. The occupancy grid map is then used to localize the robot and navigate it through the environment. Additionally, SLAM algorithms can use additional sensory data such as inertial measurements and camera images to improve the accuracy and reliability of the mapping and localization process.

The SLAM algorithm starts by generating an initial map of the environment and then uses the data from the sensors to refine the map. SLAM algorithms can also localize the robot in the environment by tracking its motion and comparing it to the map. Slam algorithms are a powerful tool for navigation and can be used in many applications such as self-driving cars, robotics, and augmented reality.

## SLAM Process

[![SLAM Process diagram](/assets/img/how_it_works/slam02.jpg){:class="img-fluid w-100"}](/assets/img/how_it_works/slam02.jpg)

### Pose esimation

`Pose estimation` is a process of estimating the position and orientation of an object in a 3D space. It uses a combination of computer vision and machine learning techniques to determine the 3D position of an object from an image or video.

Pose estimation can be used to recognize objects and estimate their poses in a scene, allowing for applications such as augmented reality, robotics, and virtual reality. 

The process typically involves using algorithms to detect features in the image or video, such as keypoints or edges, and then using machine learning techniques to identify the object and estimate its pose. It can also be used to estimate the pose of a person in a video, allowing for applications such as gesture recognition and tracking.

---

### Feature matching

`Feature matching` is a key component of SLAM. It essentially involves matching features between images taken from different locations and orientations to create a map. Feature matching involves extracting features from an image and then finding the same features in other images. This is done by comparing features such as intensity, color, shape, and texture. Once the features are matched, the pose or location of the camera can be estimated. By combining this information over time, the SLAM algorithm can build a map of the environment.

Optical Computer mice also use this method to track the motion of the mouse.

---

### Loop closure

`Loop closure` in SLAM is the process of recognizing when a robot has returned to a previously visited location. This enables the robot to more accurately map its environment and improve its navigation capabilities. By recognizing a previously visited area, the robot can more accurately understand the layout of the environment and accurately determine its location.

This process can prevent *drift*, where sensors such as IMU and odemetry's small inaccuracies can build up over time and cause the pose estimation to incorrect position the robot, and it appears to drift around on the map.

---

### bundle adjustment

`Bundle adjustment` in SLAM is a process of refining the estimated camera poses and point locations of a scene by minimising the reprojection errors of the estimated 3D points onto the observed 2D image points. This is done by adjusting the camera poses and 3D points in a least squares sense. The goal is to optimise the estimates of the camera poses and 3D points to obtain the best-fit solution. This is an iterative process that is repeated until the reprojection errors are minimised.

---
