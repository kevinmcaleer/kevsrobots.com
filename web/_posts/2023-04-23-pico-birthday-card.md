---
layout: project
title: Pico Birthday Card
description: "Raspberry Pi Powered Birthday Card"
short_title: Pico Birthday Card
short_description: "Raspberry Pi Powered Birthday Card"
difficulty: Intermediate
date: 2023-04-23
author: Kevin McAleer
excerpt: >- 
   How to Make an Ultimate DIY Birthday Card with Raspberry Pi Pico
   
cover: /assets/img/blog/picard/picard.jpg
tags: 
 - Raspberry Pi Pico
 - Birthday Card
 - MicroPython
 - 3D Printing
groups:
 - weird
 - seasonal
 - micropython
videos:
 - ylZYOJ2zx8c
repo:
 - https://github.com/kevinmcaleer/picobirthdaycard
---

## Overview

Are you tired of giving boring, generic birthday cards? Want to give a personalized and interactive card that will impress your loved ones? Look no further! In this tutorial, we'll show you how to create a unique and special birthday card with the power of Raspberry Pi Pico.

Raspberry Pi Pico is a microcontroller board that offers a simple and low-cost way to interact with the physical world. With its easy-to-learn programming language and some extra components, Raspberry Pi Pico is perfect for creating fun and creative DIY projects. In this tutorial, we'll show you how to use Raspberry Pi Pico to build an ultimate DIY birthday card.

I've not heard of any using a Raspberry Pi Pico to power a birthday card before, so maybe I'm the first to do this!

---

## Gallery

Here are a couple of images of the construction process.

[![Card example](/assets/img/blog/picard/card01.jpg){:class="img-fluid w-25"}](/assets/img/blog/picard/card01.jpg)
[![Card example](/assets/img/blog/picard/card02.jpg){:class="img-fluid w-25"}](/assets/img/blog/picard/card02.jpg)
[![Card example](/assets/img/blog/picard/card03.jpg){:class="img-fluid w-25"}](/assets/img/blog/picard/card03.jpg)
[![Card example](/assets/img/blog/picard/card04.jpg){:class="img-fluid w-25"}](/assets/img/blog/picard/card04.jpg)
[![Card example](/assets/img/blog/picard/card05.jpg){:class="img-fluid w-25"}](/assets/img/blog/picard/card05.jpg)
[![Card example](/assets/img/blog/picard/card06.jpg){:class="img-fluid w-25"}](/assets/img/blog/picard/card06.jpg)

---

## Bill of Materials

Item         | Description                                                    | Qty | Price
-------------|----------------------------------------------------------------|:---:|-----:
Pico         | Raspberry Pi Pico                                              |  1  | £3.90
Touch Sensor | TTP223 Capacitive Touch                                        |  1  | £1.00
5x5 Display  | [5x5 RGB Matrix Breakout](collabs.shop/vdxbfh)                 |  1  |  9.90
Buzzer       | Piezo Transducer (Passive Buzzer)                              |  1  | £0.90
Lipo Battery | Galleon 400mAh Hard Case LiPo Battery                          |  1  | £7.50
Lipo Charger | [LiPo Amigo (LiPo/LiIon Battery Charger)](collabs.shop/eewgsf) |  1  | £8.40
{:class="table table-striped"}

---

## About the project

In this tutorial, we will guide you through the process of building an ultimate DIY birthday card using Raspberry Pi Pico. In the accompanying video you can see the card's exciting features, including a touch sensor, a 5x5 RGB LED display, and a speaker that plays the Happy Birthday tune.

Before starting the project, Kevin introduces the necessary materials and components needed for the card. 

Next, Kevin leads us through the assembly process. He shows us how to connect the Raspberry Pi Pico to the touch sensor, LED display, and speaker. He explains how to place all components in the enclosure, and gives us tips on how to secure everything in place.

---

## Wiring

![Wiring Diagram](/assets/img/blog/picard/picard-wiring.jpg){:class="img-fluid w-100"}

Wiring the Pico to the components may look complicated, however it is actually straight forward.

Component Pin      | Pico GPIO
-------------------|----------
Touch Sensor       | GPIO 00
Touch Sensor GND   | GND
Touch Sensor VCC   | 3.3v
5x5 Display - SDA  | GPIO 02
5x5 Display - SCL  | GPIO 03
5x5 Display - VCC  | 3.3v
5x5 Display - GND  | GND
Buzzer + or signal | GPIO 09
Buzzer - or GND    | GND
{:class="table table-striped"}

---

## Assembly

Print out [3d printbale cart insert, below](#3d-printed-enclosure). The parts will fit into the card insert and can be attached to the card with either hot-glue or blutac.

You can design a foldable card using the online graphics creation tool, [Canva](www.canva.com). Its free to use, and is very simple to create amazing designs.

---

## MicroPython code

After completing the assembly process, it's time to code. Kevin walks us through the coding process using MicroPython, a programming language perfect for microcontrollers. He explains the different sections of the code, and how to adjust the settings to personalize the card.

There are a couple of programs:

* `buzzer_demo.py` - plays the happy birthday tune
* `cake.py` - Shows the cake image on the 5x5 display
* `font.py` - This file contains all jonny five font thats specifically designed for the 5x5 display
* `touch.py` - displays touched or not touched using the touch sensor
* `scroll.py` - displays scrolling text on the 5x5 display
* `happy_birthday.py` - bring all the programs above together so that when you touch the sensor it shows the cake, plays the music, and then displays the message

---

## 3d printed enclosure

You can download the 3d printable enclosure here:

* [`card-insert.stl`](/assets/stl/picard/card-insert.stl) - Card Insert

---

## Summary

This tutorial provides a fun and engaging way to create an ultimate DIY birthday card using Raspberry Pi Pico. With the easy-to-follow instructions, you can impress your loved ones with a personalized and interactive card that they will cherish.

---

So, what are you waiting for? Grab your Raspberry Pi Pico, follow the steps in the video, and create an ultimate DIY birthday card that your loved ones will never forget!

---
