---
title: PicoSMARS 2
layout: blog
short_title: PicoSMARS 2
short_description: A Raspberry Pi Pico W Powered SMARS Robot
description: A Raspberry Pi Pico W Powered SMARS Robot
date: 2023-10-05
author: Kevin McAleer
excerpt: 
cover: /assets/img/blog/picosmars/picosmars.jpg
tags:
 - robotics
 - SMARS
 - pico
 - micropython
groups:
 - pico
 - robots
 - 3dprinting
 - smars
 - micropython
---

`PicoSMARS 2` is a [SMARS](/smars) based robot that, unlike the original SMARS is powered by a Raspberry Pi Pico W.

This enables the SMARS to be controlled via WiFi, Bluetooth and run more sophisticated programs. The Pico enables us to write programs in MicroPython as well as in C++, so unlike the Arduino based SMARS it has some extra language options.

{:toc}
* toc

---

## Design

PicoSMARS is based on the 3D Printables SMARS chassis, with a slight modification to the rear right side to accomodate the [Kitronik Robotics board](https://kitronik.co.uk/products/5329-kitronik-compact-robotics-board-for-raspberry-pi-pico?_pos=2&_sid=62e580e2a&_ss=r) on/off switch.

[![PicoSMARS 2](/assets/img/blog/picosmars/picosmars.jpg){:class="img-fluid w-75"}](/assets/img/blog/picosmars/picosmars.jpg)

---

## Kitronik Robotics Board

The motor driver and Raspberry Pi Pico interface is provided by the Kitronik Robotics Board (£14.25 from Kitronik Ltd). Power to the board can be provided by a 9v battery (the board supports 3V - 10.8V), which fits neatly into the SMARS Chassis.

The Kitronics Robotics Board also has connectors for 4 DC motors or 2 stepper motors, and 8 servos (which are controlled via an onboard 16 port I2C driver).

An On/Off switch also enables you to save battery power when its time to turn the robot off, which was missing from the original SMARS design.

---

## Bill of Materials

Item              | Description                                       | Qty | Unit Price |   Cost
------------------|---------------------------------------------------|:---:|-----------:|------:
Pico / Pico W     | Either a Raspberry Pi Pico or Pico W              |  1  |      £6.30 |  £6.30
Ultrasonic Sensor | HC-SR04+ Ultrasonic range finder (the 3v version) |  1  |      £1.85 |  £1.85
9v battery        | Alkaline 9V battery                               |  1  |      £5.95 |  £5.95
2x Motors         | N20 style DC motors, 150RPM 6V                    |  1  |      £6.00 | £12.00
3x DuPont cables  | Pack of  (female to female)                       |  1  |      £2.00 |  £2.00
{:class="table table-striped"}

---

## 3D Printable STL Files

Pico SMARS Consists of several 3D printed parts:

* [`chassis.stl`](/assets/stl/picosmars2/pico_smars2_chassis.stl) - The main robot body
* [`motor_holder.stl`](/assets/stl/smars/motor_holder.stl) - Holds the motors in place
* [`range_finder_holder.stl`](/assets/stl/smars/range_finder_holder.stl) - Holds the Range Finder
* [`range_finder_cover.stl`](/assets/stl/smars/range_finder_cover.stl) - Covers the Range Finder
* [`powered_smars_wheels.stl`](/assets/stl/smars/powered_wheel.stl) - 2x the wheels that connect to the motors
* [`unpowered_wheels.stl`](/assets/stl/smars/unpowered_wheel.stl) - 2x the wheels that spin freely
* [`mechanical_tracks.stl`](/assets/stl/smars/tracks.stl) - 32x mechacnical track pieces. These connect together with pieces of 3D printed filament

---

## MicroPython code

The code for this project is available [at this GitHub Repository](https://www.github.com/kevinmcaleer/picosmars2).

You can also find the original PicoSMARS code here <https://www.github.com/kevinmcaleer/picosmars>

This project is a work in progress, so I'll return to this section at a later time to update it in more detail.

---

## Gallery

Here are some photos of PicoSMARS 2

[![PicoSMARS 1](/assets/img/blog/picosmars/picosmars01.jpg){:class="img-fluid w-25"}](/assets/img/blog/picosmars/picosmars01.jpg)
[![PicoSMARS 2](/assets/img/blog/picosmars/picosmars02.jpg){:class="img-fluid w-25"}](/assets/img/blog/picosmars/picosmars02.jpg)
[![PicoSMARS 2](/assets/img/blog/picosmars/picosmars03.jpg){:class="img-fluid w-25"}](/assets/img/blog/picosmars/picosmars03.jpg)

---
