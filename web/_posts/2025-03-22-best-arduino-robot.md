---
title: "The Best Arduino Robot for Beginners"
description: "The SMARS robot is a simple, 3D printed robot that can be controlled with an Arduino, Raspberry Pi, or any other microcontroller. It's a great way to learn about robotics, electronics, and programming."
excerpt: >-
    "The SMARS robot is a simple, 3D printed robot that can be controlled with an Arduino, Raspberry Pi, or any other microcontroller. It's a great way to learn about robotics, electronics, and programming."
layout: showcase
date: 2025-03-23
author: Kevin McAleer
difficulty: beginner
cover: /assets/img/blog/best_arduino/cover.jpg
hero:  /assets/img/blog/best_arduino/hero.png
mode: light
videos:
  - Ylc84kAKSxI
tags:
 - Arduino
 - robotics
 - micropython
groups:
 - robots
 - micropython
 - pico
 - arduino
code:
 - https://www.github.com/kevinmcaleer/smars_gamepad
---

## The Best Arduino Robot for Beginners

My first 3D printed robot was the SMARS robot. This robot took me from blinking LEDs on an Arduino to building advanced AI robots, computer vision, ROS robots and more. It's a great robot for beginners and advanced users alike.

---

## What is the SMARS Robot?

The SMARS robot is a simple, 3D printed robot that can be controlled with an Arduino, Raspberry Pi, or any other microcontroller. It's a great way to learn about robotics, electronics, and programming.

You can learn all about SMARS here:

<div class="row row-cols-3">

{% include card.html cardtitle="SMARS" link="/learn/learning_pathways/smars.html" img="/assets/img/learn/learn_smars.png" col=3 tl="2" dl="3" type="course" description="Learn how to build SMARS robots, starting with the 3D Printing the model, Designing SMARS and Programming SMARS"  noborder=true bg_color="bg-card-blue"%}
</div>

---

## GamePad Controller

I built a GamePad controller for controlling robots using a Raspberry Pi Pico WH. This allows you to control the robot and customise the controls to suit your needs.

To buy the GamePad controller, visit the store here: {% include store/pcbs.html %}

There are also instructions on how to build the GamePad controller here:

<div class="row row-cols-3">
 {% include card.html cardtitle="GamePad Controller" link="/blog/gamepad2.html" img="/assets/img/blog/gamepad2/cover.jpg" col=3 tl="1" dl="2" %}
</div>

---

## Why is the SMARS Robot the Best Arduino Robot for Beginners?

SMARS is the best Arduino robot for the following reasons:

1. **It's easy to build**: The SMARS robot is 3D printed, so you can build it yourself. This is a great way to learn about robotics and electronics.
1. **It's affordable**: The SMARS robot is affordable to build. You can 3D print the parts yourself, and the electronics are cheap and easy to find.
1. **It's easy to program**: The SMARS robot can be controlled with an Arduino, Raspberry Pi, or any other microcontroller. This makes it easy to program and customise.
1. **It's versatile**: The SMARS robot can be customised in many ways. You can add sensors, cameras, and other components to make it do whatever you want.
1. **It's fun**: The SMARS robot is a fun project to work on. You can learn a lot about robotics, electronics, and programming while building and customising your robot.

---

## Personal Journey with SMARS and Robotics

I started building robots once I had run through the usual introductory projects with Arduino. In December 2017 I bought a cheap (< $100) 3D Printer, an inet A8 clone, and started printing parts for the SMARS robot. My first robot was quite basic and used a cheap kit from eBay.

{% include gallery.html titles="My First Printer, My First Robot" images="/assets/img/blog/best_arduino/first_printer.jpg,/assets/img/blog/best_arduino/first_robot.jpg" noborder=true %}

I chose SMARS because there wasn't many other options available at the time, and SMARS is quite unique in thats its designed to be built without needing any screws; it just slots together.

SMARS has lots of mods available, and I've built a most of them. I've also designed a few of my own mods. I loved this robot design so much I created the [SMARS Fan Website](https://www.smarsfan.com) to share my mods and designs with others, as well as it being a place to bring the designs, code and how-tos together in once place.

---

## Using the GamePad with SMARS

In this project I use a GamePad controller to control the SMARS robot. The GamePad controller is built using a Raspberry Pi Pico WH, and it communicates with the robot using a Bluetooth (BLE) connection.

The gamePad is running the standard `test_gamepad.py` code from the GamePad Code repository <https://www.github.com/kevinmcaleer/gamepad>.

The code for SMARS can be found in the link below:

---
