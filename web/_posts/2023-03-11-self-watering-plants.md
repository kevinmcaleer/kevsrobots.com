---
layout: project
title: Self Watering Plants
description: Build a self watering plant system
difficulty: Beginner
short_title: Self Watering Plants
short_description: Build a self watering plant system
date: 2023-03-11
author: Kevin McAleer
excerpt: Create a self watering plant system using the Pimoroni Enviro Grow
cover: /assets/img/blog/halapeno9000/halapeno9000.jpg
tags: 
 - raspberry_pi_pico
 - micropython
 - pimoroni
 - enviro
 - enviro_grow
 - plants
 - 3d_printing
groups:
 - garden
 - pico
 - micropython
videos:
 - fD2qmNgkV0A
---

## Bill of Materials

Items         | Description                                                            | Qty |  Price
--------------|------------------------------------------------------------------------|:---:|------:
Enviro Grow   | [Pimoroni Enviro Grow](https://collabs.shop/gwdrum)                    |  1  | £39.99
or            |                                                                        |     |       
Accessory Kit | Pimoroni Enviro Grow kit accessory kit                                 |  1  |  49.50
or            |                                                                        |     |       
Grow Hat Mini | [Pimoroni Grow Hat Mini for Raspberry Pi](https://collabs.shop/kwfxqa) |  1  | £27.50
M2 Screws     | M2 Screws                                                              |  4  |  £2.00
{:class="table table-striped"}

---

## 3d design

---

I've designed some cute Geometric plant pots with integrated drainage holes, feed-tube input an sensor holder.

This is the second interation of the design, the first version caused the self watering system to mis-read the sensors as the sensor was not deep enough into the soil to take a proper reading, and instead kept watering the plant until the water supply ran out.

![Plant Pot](/assets/img/blog/halapeno9000/growholder01.jpg){:class="img-fluid w-50"}
![Enviro Grow Holder](/assets/img/blog/halapeno9000/growholder02.jpg){:class="img-fluid w-50"}

The Grow Hat Mini is mounted on top of a Raspberry Pi Zero W, which is intern mounted on a [Pi Stand](/blog/pistands.html).

---

## Assembly

The Grow Hat Mini, or Enviro grow connects to the 3 soil moisture sensors, and also to the three water pumps, and each water pump connects to a plant pot (ensure that the sensor and pump are connected to the same plant - A, B & C).

---

## Download the STL files and print today

* [`plantpot.stl`](/assets/stl/halapeno9000/plantpot.stl) - Large Plantpot
* [`grow holder.stl`](/assets/stl/halapeno9000/grow_holder.stl) - Enviro Grow Holder
* [`connector.stl`](/assets/stl/halapeno9000/connector.stl) - Connector (connects the feed tube to the plantpot)

---
