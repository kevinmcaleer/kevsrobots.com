---
layout: blog
title: Rover the Mecanum Robot
date: 2022-04-30
author: Kevin McAleer
excerpt: Build your own Mecanum Robot using this guide
cover: /assets/img/blog/rover/rover.jpg
tags:
 - Rover
 - Motor 2040
 - Mecanum
 
---

## Table of Contents

{:toc}
* toc

---

## YouTube Video
{% include youtubeplayer.html id="JsYGHDRF-VQ" %}

## Overview - Rover
Meet Rover - the Mecanum marvel. Rover is a simple robot, one you can 3d print yourself using the STL files below. Rover has mecanum wheels - these wheels have small spindles at a 45 degree angle to the direction the wheel is pointing. This means if four of these wheels are used in unison the robot will move sideways.

## Bill of Materials

Item | Description | Cost
-----|---|--
Mecanum wheels | Make your robot or buggy go every which way with Mecanum omniwheels (pack of 4)| £24
HC-SR04 | Detect objects in front of the robot using Ultrasound | £5
4x 50:1 Micro Metal GearMotors | Provide fast and accurate movement with these little motors | £5.10
4x Standoff | These provide the controller board with room to breath above the chasis, and make it more accessible (pack of 4)  | £4.50 
{:class="table table-striped"}


![Mecanum Wheel](/assets/img/blog/rover/mecanum_wheel.png){:class="img-fluid w-50"}

## Print your own - (the STL files)
Rover is made up of three 3d printable files:
- [Chassis](/assets/stl/chassis_v2.stl)
- [Range Finder Holder](/assets/stl/range_finder_holder.stl)
- [Motor Holders](/assets/stl/rover_wheel_holder_v9.stl)

## Mecanum wheels
You can buy mecanum wheels online from companies such as [Pimoroni](https://shop.pimoroni.com/products/mecanum-wheels-pack-of-4?variant=31590632030291) at a price of around £24 (excluding shipping).

## Other Electronics
Rover uses four N20 Motors, 150RPM motors (the 6v variety) should work fine, however a better option is the N20 Motors with built in Encoders - this enables ultra precise movement and positioning. You'll need a controller board that can read the values from the encoders to count how many revolutions each motor has made. Encoders are simply a wheel that attaches to the end of the motor shaft, and has a hole or mark that can be read by a sensor, often an infra-red led and infra-red sensor pair. The sensor detects the hole (or some kind of mark) every time the wheel rotates 360 degrees. The rotation data can then be read by the microcontroller to count each revolution and feed this into the algorithm that is driving the motors.

## Reading the Range Finder
The range finder uses 4 pins (5V, GND, Echo and Trigger).  