---
layout: lesson
title: Rviz2
type: page
description: Let's learn about RViz2, the ROS Visualisation tool
cover: /learn/learn_ros/assets/rviz2.jpg
---

![RViz2 screenshot]({{page.cover}}){:class="cover"}

## Watch the video

{% include youtubeplayer.html id="ev9Mt_gg81g" %}

---

## What is RViz2?

[`RViz2`](/resources/glossary#rviz2) is a powerful, user-friendly visualization tool that allows you to explore and analyze data in 3D. It is capable of displaying a variety of objects including robots, cameras, geometry, point clouds, and more. RViz2 is an open source tool that is available for free and is compatible with the ROS (Robot Operating System) platform.

---

## How can we use RViz2?

We can use RViz2 to display the data from our LiDAR, and then take this further by adding in extra `nodes` such as the Mapping and Navigation nodes from the [SLAM](/resources/glossary#SLAM) Toolkit.

---

RViz2 is included in the full desktop version of ROS2; so far we've only used the `core` version. The example files include a second Docker setup, `docker-full`, whose `dockerfile` also installs the `ros-humble-desktop` packages, and whose `docker-compose.yml` includes the `DISPLAY` and `network_mode: host` settings from the last lesson.

---

## Build the full container

* **Build and start the container** - From the Raspberry Pi terminal, type:

```bash
cd ~/cubie-1/docker-full
docker compose build
docker compose up -d
```

> If the build stops with an error about `ros-humble-desktop=0.10.0-1*` not being found, edit the `dockerfile`, remove `=0.10.0-1*` from that line so the latest version is installed, and build again.

---

## Console into the container

Let's test out `RViz2`.

* **launch Bash in Container** - From the Raspberry Pi terminal, type:

```Bash
docker compose exec ros2 bash
```

---

## Launch Rviz2

* **launch RViz2** - From the container terminal, type:

```Bash
rviz2
```

RViz should now open within the Raspberry Pi OS - this is because the container is outputting its X11 graphics commands to the Host (the Raspberry Pi OS).

---
