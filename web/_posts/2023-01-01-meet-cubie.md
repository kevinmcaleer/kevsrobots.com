---
layout: blog
title: Meet Cubie-1
short_title: Meet Cubie-1
short_description: A ROS Robot
date: 2023-01-01
author: Kevin McAleer
excerpt: Build a robot to learn ROS with
cover: /assets/img/blog/cubie-1/cubie07.jpg
tags:
 - Raspberry Pi 4
 - ROS
 - Robot
---

## Contents

{:toc}
* toc

---

## YouTube Video

Click the thumbnails below to watch the show all about this build.

{% include youtubeplayer.html id="_cTJhCRv858" %}

---

## Overview

Meet Cubie-1; the [Raspberry Pi](/glossary#raspberrypi) 4 robot designed to help you learn [ROS](/glossary#ros). Cubie is a 3d printable robot, with a [LiDAR](/glossary#lidar) sensor and 16mp autofocus Arducam camera module.

Cubie-1 was designed to support the [Learn ROS with me](https://youtube.com/playlist?list=PLU9tksFlQRircAdEplrH9NMm4WtSA8yzi) series.

---

## Bill of Materials

Item            | Description             | Qty |  Price
----------------|-------------------------|----:|------:
Lidar           | Slamtec RPLidar A1      |   1 | £90.00
Motors          | N20 Motors              |   4 | £10.00
Explorer Hat    | Pimoroni Explorer Hat   |   1 | £13.20
Arducam         | 16mp autofocus Arducam  |   1 | £30.00
Battery         | 5000mAh LiPo battery    |   1 | £18.00
Wheels          | Moonbuggy Wheels (pair) |   2 |  £4.50
Raspberry Pi 4  | Raspberry Pi 4 8Gb      |   1 | £85.50
M3 nuts & Bolts | Assortment              |   1 |  £9.99
{:class="table table-striped"}

---

## 3d Design

Cubie is made up of several 3d printable parts:

* Base
* Pillar
* Shelf
* Top
* Motor Holders
* Pi Holder
* Camera holder
* Front Panel
* Side Panel
* Top Side Panel
* Back Top Panel

### Gallery

[![](/assets/img/blog/cubie-1/01base.png){:class="img-fluid w-25"}](/assets/img/blog/cubie-1/01base.png)
[![](/assets/img/blog/cubie-1/02pillar.png){:class="img-fluid w-25"}](/assets/img/blog/cubie-1/02pillar.png)
[![](/assets/img/blog/cubie-1/03piholder.png){:class="img-fluid w-25"}](/assets/img/blog/cubie-1/03piholder.png)
[![](/assets/img/blog/cubie-1/04raspberrypi.png){:class="img-fluid w-25"}](/assets/img/blog/cubie-1/04raspberrypi.png)
[![](/assets/img/blog/cubie-1/05shelf.png){:class="img-fluid w-25"}](/assets/img/blog/cubie-1/05shelf.png)
[![](/assets/img/blog/cubie-1/06lidar.png){:class="img-fluid w-25"}](/assets/img/blog/cubie-1/06lidar.png)
[![](/assets/img/blog/cubie-1/07top.png){:class="img-fluid w-25"}](/assets/img/blog/cubie-1/07top.png)
[![](/assets/img/blog/cubie-1/08camera.png){:class="img-fluid w-25"}](/assets/img/blog/cubie-1/08camera.png)
[![](/assets/img/blog/cubie-1/09camera_holder.png){:class="img-fluid w-25"}](/assets/img/blog/cubie-1/09camera_holder.png)
[![](/assets/img/blog/cubie-1/10frontplates.png){:class="img-fluid w-25"}](/assets/img/blog/cubie-1/10frontplates.png)
[![](/assets/img/blog/cubie-1/11sidepanels.png){:class="img-fluid w-25"}](/assets/img/blog/cubie-1/11sidepanels.png)
[![](/assets/img/blog/cubie-1/12final.png){:class="img-fluid w-25"}](/assets/img/blog/cubie-1/12final.png)

---

## Electronics

* This project uses a Raspberry Pi 4, which is powered by the USB powerbank. 
* The motors are connected to the Explorer Hat; the two left motors connect to `Motor 1` and the two right motors to `Motor 2`.
* The Arducam connects to the CSI camera module connector on the Raspberry Pi.
* The Lidar connects to the Raspberry Pi via the USB Port. (a later update may change this to connect directly via 2 GPIO pins).

---

## ROS2 Python code

A later update will provide the code.

---

## STL files

* [`base.stl`](/assets/stl/cubie-1/base.stl)
* [`pillar.stl`](/assets/stl/cubie-1/pillar.stl)
* [`shelf.stl`](/assets/stl/cubie-1/shelf.stl)
* [`top.stl`](/assets/stl/cubie-1/top.stl)
* [`motor_holders.stl`](/assets/stl/cubie-1/motor_holder.stl)
* [`pi_holder.stl`](/assets/stl/cubie-1/pi_holder.stl)
* [`camera_holder.stl`](/assets/stl/cubie-1/camera_holder.stl)
* [`front_plate.stl`](/assets/stl/cubie-1/front_plate.stl)
* [`side_panel.stl`](/assets/stl/cubie-1/side_panel.stl)
* `top_side_panel.stl` - coming soon
* `back_top_panel.stl` - coming soon