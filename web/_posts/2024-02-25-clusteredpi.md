---
title: ClusteredPi
description: >-
    Learn how to build a cluster of Raspberry Pi's to learn more about distributed computing and to experiment with Docker Swarm.
layout: showcase
date: 2024-02-25
author: Kevin McAleer
difficulty: beginner
excerpt: >-
    In December 2021 I decided to build a cluster of Raspberry Pi's to learn more about distributed computing and to experiment with Docker Swarm.
cover: /assets/img/blog/clusteredpi/cover.jpg
hero: /assets/img/blog/clusteredpi/hero.png
mode: light
tags:
  - cluster
  - raspberry pi
  - docker
groups:
  - raspberrypi
videos:
  - 0wB2i8DFehw
---

In December 2021 I decided to build a cluster of Raspberry Pi's to learn more about distributed computing and to experiment with Docker Swarm. I had a few Raspberry Pi Zero's lying around, so I decided to put them to good use.

I started by installing the latest version of Raspberry Pi OS on each of the Pi Zero's, and then I installed Docker on each of them. Once I had Docker installed, I was able to create a Docker Swarm cluster and deploy a simple web application to it - the [clustered-pi website](https://www.clustered-pi.com) itself <https://www.clustered-pi.com>.

---

The Clustered-Pi Logo
{:class="caption"}

![Clustered-Pi](/assets/img/blog/clusteredpi/clusteredpi01.png){:class="img-fluid w-25 rounded-3 shadow-lg"}

---

I based the 3D printed design of the cluster on the original Cray-1 Super Computer.

The Clustered-Pi Design based on the Cray-1 Super Computer
{:class="caption"}

![Clustered-Pi](/assets/img/blog/clusteredpi/clusteredpi03.jpg){:class="img-fluid w-25 rounded-3 shadow-lg"}

The Clustered-Pi Design
{:class="caption"}

![Clustered-Pi](/assets/img/blog/clusteredpi/clusteredpi02.jpg){:class="img-fluid w-25 rounded-3 shadow-lg"}

---

One of the challeneges with this design is that the Raspberry Pi Zero's don't have a network port, so I had to use the built-in WiFi to connect them to the network, this usually fine but with so much radio being broadcast in such close proxmity its not ideal.

The next version of Clustered-Pi was built using Raspberry Pi 4's, which have a built-in network port, so I was able to connect them to the network using a network cable.

The Raspberry Pi 4 Clustered-Pi went on to host a number of web applications, including a [blog](https://www.clustered-pi.com/blog/), KevsRobots website <https://www.kevsrobots.com> which includes hosting a python based search engine <https://search.kevsrobots.com/search?query=python> and a few other web applications, [read more about them here](https://www.clustered-pi.com/blog/software-overview.html).

---

## Rack Mounted Clustered-Pi

I sourced the rack mount from a local supplier (<https://www.thepihut.co.uk>), and it was a perfect fit for the Raspberry Pi 4's. The Pi's were powered through a 5V 20A power supply, and the network switch beneath the rack was powered by a 12V 5A power supply.

---

## Bill of Materials

|Item | Description            | Qty | Approx Price | Total |
|:--:|:---------------|:---:|------------:|-----:|
| ![A Raspberry Pi 4](/assets/img/blog/clusteredpi/raspberry-pi-4.png){:style="height:32px"}| [Raspberry Pi 4](https://amzn.to/30SzZw0) |  4  | £58          | £232   |
|![An SD Card](/assets/img/blog/clusteredpi/sd_card.png){:style="height:32px"}| [64sdGb SD Card](https://amzn.to/33VboYH) | 4 | £15 | £60 |
| ![A network switch](/assets/img/blog/clusteredpi/tp_link_8port_hub.png){:style="height:32px"}| [Network Hub](https://amzn.to/3poQp94) | 1 | £18 | £18|
|![some network cables](https://www.clustered-pi.com/assets/img/network_cables.jpg){:style="height:32px"}| [CAT6 Cables](https://amzn.to/3FsnSFc) | 1 | £13 | £13 |
|![a usb cable](/assets/img/blog/clusteredpi/usb_cable.png){:style="height:32px"}| [USB A to USB C Charging Cables](https://amzn.to/3FsvjfA) | 4 |£6  |£24|
| ![usb power supply](/assets/img/blog/clusteredpi/usb_charger.png){:style="height:32px"} | [USB Charger - 3A per port](https://amzn.to/3JehNOY) | 1  | £15  |£15 |
| ![a raspberry pi cluster chassis](/assets/img/blog/clusteredpi/cluster_chassis.png){:style="height:32px"} | [Raspberry Pi 4 Chassis](https://thepihut.com/products/metal-cluster-rack-case-kit-for-raspberry-pi-4?variant=41230817591491&currency=GBP&utm_medium=product_sync&utm_source=google&utm_content=sag_organic&utm_campaign=sag_organic&gclid=Cj0KCQiA2ZCOBhDiARIsAMRfv9IT60o3yFnRcOAhtCzc-M35Q0qapcmBIeexjIhR8EqQ5qvckcffw68aAsajEALw_wcB) | 1 | £42 | £42|
| | | |**Grand Total** |**£404**|
{:class="table "}

---

## Raspberry Pi 4

Clustered-Pi consists of 4 [Raspberry Pi 4](https://www.raspberrypi.org) computers. Each one has slightly different sizes of memory, which is more to do with when they released these than a design choice.

![Raspberry Pi 4](/assets/img/blog/clusteredpi/raspberry-pi-4.png){:class="img-fluid w-25"}
Raspberry Pi 4, 8Gb

---

### SD Cards

I recommend getting a branded SD card, from a reputable supplier; in my experience unbranded SD Cards only last a couple of months under reasonable use, whereas the branded ones, particularly those designed for heavy write demands last years.

![sd cards](/assets/img/blog/clusteredpi/sd_card.png){:class="img-fluid w-25"}
64Gb SanDisk Extreme SD Card

## Disk Imaging

I used the official [Raspberry Pi Imager](https://www.raspberrypi.com/software/)

---

## Configuration

Each node has a `hostname` that is unique:

* `node01`
* `node02`
* `node03`
* `node04`

You set the hostname when first booting up the Raspberry Pi, or later using the `raspi-config` command, from the terminal.

---

## Network hub & Cables

Lets look how to wire this all up.

---

### Network Hub

I purchased a [cheap 8 port network hub](https://amzn.to/3poQp94) (a TP Link 8 port Gigabit Desktop Switch). This gives some room for expansion later on, and doens't take up too much space on the desk.

It also just happens to be the perfect size to sit underneath the Raspberry Pis.

---

### Network Cables

I also bought bunch of [colourful patch CAT6 cables](https://amzn.to/3FsnSFc) to connect the hub to each Raspberry Pi. The cables are only 25cm long, so they fit neatly into the pi's and hub.

---

|                                                                                                                   |                                                                                                      |
|:-----------------------------------------------------------------------------------------------------------------:|:----------------------------------------------------------------------------------------------------:|
| [![TP Link 8 Port Hub](/assets/img/blog/clusteredpi/tp_link_8port_hub.png){:class="img-fluid"}](https://www.clustered-pi.com/assets/img/tp_link_8port_hub.jpg) | [![Cables](https://www.clustered-pi.com/assets/img/network_cables.jpg){:class="img-fluid w-50"}](https://www.clustered-pi.com/assets/img/network_cables.jpg) |
|                                                    Network Hub                                                    |                                                Cables                                                |
{:class="my-4"}

The network hub is connected to my router, so I can connect to them from my laptop, and more importantly, they are connected to the Internet so they can download Docker images.

---

## Chassis & Power

Lets get our Pi's Racked and stacked.

## Cluster Chassis

I originally had the Raspberry Pi's in their cases, just sat on top of the network hub, however this was shortly improved by purchasing a purpose built chassis. Not only does it make the Raspberry Pi's more structurally stable, it provides additional cooling via 2 large fans behind the Pis.

|                                                                                             |                                                                                    |
|:-------------------------------------------------------------------------------------------:|:----------------------------------------------------------------------------------:|
| [![No Chassis](https://www.clustered-pi.com/assets/img/no_chassis.jpg){:class="img-fluid"}](https://www.clustered-pi.com/assets/img/no_chassis.jpg) | [![Chassis](https://www.clustered-pi.com/assets/img/chassis.jpg){:class="img-fluid"}](https://www.clustered-pi.com/assets/img/chassis.jpg) |
|                                         No Chassis                                          |                                    With Chassis                                    |
{:class="mb-4"}

## Power

For Power, I bought some [short 25cm USBA to USB C charger cables](https://amzn.to/3FsvjfA), and a [4 port USB charger plug](https://amzn.to/3JehNOY) that can deliver 3.1Amps per port.

---

## UPS

This Cluster will be running a live website, accessible via the internet, so to ensure its always available I have powered it via an uninterruptable power supply (UPS), I used the [Cyberpower BR1200ELCD BRIC](https://amzn.to/3qo9fMy)

![Chassis](/assets/img/blog/clusteredpi/ups.png){:class="img-fluid w-25"}

Cyberpower UPS: 1200VA/720W, with 6 UK Outlets

---

## Pi 5 upgrades

The Clustered-Pi 5 Upgrades
{:class="caption"}

![Clustered-Pi](/assets/img/blog/clusteredpi/clusteredpi04.jpg){:class="img-fluid w-50 rounded-3 shadow-lg"}

In October 2023 the Raspberry Pi 5 was released, and this meant I could finally upgrade the Pi 4 cluster with more power, and more reliable and larger storage, via the NVMe SSD's.

I managed to source another rack enclosure, however these are now discontinued.

Each Pi 5 is powered by the official 27W power supply, each also with a 1TB NVMe SSD, and a 1TB NVMe Base from Pimoroni (if you buy one of these, remember to include the `kevin` coupon code - I'll thank you later).

Read more about the upgrades [here](https://www.clustered-pi.com/blog/pi5-upgrades.html)

---
