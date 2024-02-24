---
title: SunFounder GalaxyRVR Review
description: 
layout: project
date: 2024-02-24
author: Kevin McAleer
difficulty: beginner
excerpt: >-

cover: /assets/img/blog/galaxyrvr/galaxyrvr_review.jpg
tags:
  - robot
  - arduino
groups:
  - arduino
  - robot
videos:
  - 6XGtNjmqBts
---

This weekend I've been ~~playing~~ reviewing with the **SunFounder** `GalaxyRVR`, a versatile and customizable robot that can be programmed to perform a wide range of tasks, from simple movements to complex behaviors. 

The GalaxyRVR is an excellent choice for anyone who is interested in learning about robotics and programming, and it is suitable for both beginners and experienced enthusiasts.

> ## Fusion to the rescue!
>
> When assembling the robot I found one of the wheel connectors (that connects the wheel to the TT motor) had the wrong type of hole in it, so I quickly designed a replacement part in Fusion 360 and 3D printed it.
>
> ![Galaxy RVR](/assets/img/blog/galaxyrvr/3dpart.jpg){:class="img-fluid w-100 rounded-3 shadow-lg"}

---

## SunFounder GalaxyRVR Review

You can find my full review of the SunFounder GalaxyRVR here:

<div class="row">
<div class="col-lg-6 col-12">
{% include card.html cardtitle="SunFounder GalaxyRVR Review" img="/assets/img/reviews/galaxyrvr_review.jpg" link="/reviews/galaxyrvr" %}
</div></div>

---

## Features & Functionality

I most impressed with the simple communication between the ESP32 Camera modeule and the Arduino Uno R3 microcontroller. 

The size of the robot is impressive for the price point too, and the fact that it can be powered by a solar panel is a nice touch.

---

## Problems with the Wifi connection

When testing the robot I found the Wifi connection to be troublesome. I could see that it aquired an IP address, which proves the hardware is working, but I couldn't connect to it using the app. I'm going to spend some time looking into this and see if I can find a solution. Hopefully this can be resolved because it's a great robot otherwise.

---
