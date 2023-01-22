---
layout: lesson
title: Rviz2
type: page
description: Lets learn about RViz2, the ROS Visualisation tool
cover: assets/rviz2.jpg
---

![RViz2 screenshot]({{page.cover}}){:class="cover"}

## What is RViz2?

[`RViz2`](/resources/glossary#rviz2) is a powerful, user-friendly visualization tool that allows you to explore and analyze data in 3D. It is capable of displaying a variety of objects including robots, cameras, geometry, point clouds, and more. RViz2 is an open source tool that is available for free and is compatible with the ROS (Robot Operating System) platform.

---

## How can we use RViz2?

We can use RViz2 to display the data from our LiDAR, and then take this further by adding in extra `nodes` such as the Mapping and Navigation nodes from the [SLAM](/resources/glossary#SLAM) Toolkit.

---

RViz2 is included in the full desktop version of ROS2, initially we only used the `core` version of ROS2. Its easy to change this, in the `dockerfile` you can change the line:

```docker
FROM ros:humble-ros-core-jammy
```

to the line:

```docker
FROM ros:humble-ros-desktop-jammy
```

---

## Rebuild Docker Container

We need to rebuild the docker-container so that it now has the settings.

* **Rebuild the container** - From the Raspberry Pi terminal, type:

```bash
docker-compose build --no-cache
docker-compose up -d
```

---

## Console into the container

Lets test out `RViz2`.

* **launch Bash in Container** - From the Raspberry Pi terminal, type:

```Bash
docker exec -it docker-full_ros2_1 bash
```

---

## Launch Rviz2

* **launch RViz2** - From the Raspberry Pi terminal, type:

```Bash
rviz2
```

RViz should now open within the Raspberry Pi OS - this is because the container is outputing its X11 graphics commands to the Host (the Raspberry Pi OS).

---
