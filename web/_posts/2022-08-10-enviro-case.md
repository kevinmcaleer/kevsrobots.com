---
layout: blog
title: Pimoroni Enviro Indoor Stevenson Screen case
short_title: Stevenson Screen
short_description: For Pimoroni Enviro Indoor
date: 2022-08-10
author: Kevin McAleer
excerpt: 3D Print your own Stevenson Screen for the Pimoroni Enviro Indoor sensor
cover: /assets/img/stevenson_screen.jpg
subtitle: 3d Printable Stevenson Screen for Pimoroni Enviro Indoor
description: 3D Print your own Stevenson Screen for the Pimoroni Enviro Indoor sensor
tags:
 - Pimoroni
 - Enviro
 - 3D Printing
groups:
 - 3D Printing
 - raspberrypi
 - Weather
 - garden
 - stands and cases
---

## Table of Contents

{:toc}
* toc

---

## Enviro Sensors

![Pimoroni Indoor sensor](/assets/img/pimoroni_enviro_indoor.webp){:class="img-fluid w-25"}

Have you seen the new [Pimoroni](https://shop.pimoroni.com/products/enviro-indoor?variant=40055644684371) Enviro range of sensors?
There are is a line up of great sensors, including:

* [Enviro Indoor](https://shop.pimoroni.com/products/enviro-indoor?variant=40055644684371) - for measuring indoor temperature, humidity, pressure and Volitile Organic Compounds (VOC). It also has a light and colour sensor.
* [Enviro Urban](https://shop.pimoroni.com/products/enviro-urban?variant=40056508219475) - for measuring outdoor temperature, humidity, pressure and particulate matter
* [Enviro Grow](https://shop.pimoroni.com/products/enviro-grow?variant=40055904305235) - for measuring plant health; moisure level, humidity, temperature, pressure and even has 3 water pump connectors for auto watering your plants
* [Enviro Weather](https://shop.pimoroni.com/products/enviro-weather-board-only?variant=40056047173715) - for measuring weather, including Rain level, Windspeed and direction, temperature, humidity, pressure and light levels
* [Enviro Camera](https://shop.pimoroni.com/products/enviro-camera?variant=40056602067027) - 2 megapixel camera for capturing images of wildlife

## What makes the Enviro a standout product

The Enviro has a special feature up it sleeve - it can run on a battery for over 6 months due to insanely great battery management. This means you can plug in a battery pack, place it somewhere convienent and forget about it!

## Local Storage, MQTT and even InfluxDB connectivity

The Enviro takes data readings every 5 minutes, and after the 5th reading it will connect to the local Wi-Fi and upload the readings to a number of end points, including:

* MQTT (this is the endpoint I've choosen)
* InfluxDB
* Local storage
* Adafruit I/O
* Custom HTTP endpoint

## 3D Printed Enclosure (STL Files)

I've designed a cute enclosure for the Enviro Indoor, which is big enough to hold both the Enviro and a battery Pack.
The design is based on the larger [Stevenson Screen](https://shop.pimoroni.com/products/weatherproof-cover-for-outdoor-sensors?variant=40047884468307), which is pretty cheap at £13 plus shipping.

![Enviro 3d design render](/assets/img/stevenson_3d.png){:class="img-fluid w-75"}

You can download the STL files here:

* [`Enclosure.stl`](/assets/stl/stevenson_screen/stevenson_screen.stl)
* [`Bottom.stl`](/assets/stl/stevenson_screen/bottom.stl)

The `Enclosure.stl` is the main body, and the `Bottom.stl` is the bottom section that can be screwed in with two 2M screws/bolts.

These are completely free to download - if you want to support my work (which is always free), [you can buy me a coffee](https://www.buymeacoffee.com/kevinmcaleer)

Enjoy!
-Kevin