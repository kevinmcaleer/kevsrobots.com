---
layout: blog
title: BurgerBot V2 - Quick on the Draw
short_title: BurgerBot V2
short_description: BurgerBot gets an upgrade
date: 2022-09-22
author: Kevin McAleer
excerpt: Add a Sharpie to Burgerbot and learn how to draw with a robot!
cover: /assets/img/blog/burgerbot/burgerbot_v2.jpg
tags:
 - Raspberry Pi Pico W
 - Pico
 - Pico W
 - Robot
 - MicroPython
 - Logo
 - 3D Printing
 
---

## New BurgerBot Course

Learn how to build your own BurgerBot with the new course:

[![BurgerBot Course](/learn/burgerbot/assets/burgerbot.jpg){:class="img-fluid w-50 rounded-3"}](/learn/burgerbot/)

---

## Table of Contents

{:toc}
* toc

---

## Videos

There are a couple of videos covering the features, demo and build process for BurgerBot, as well as the original BurgerBot and why I created it.

{% include youtubeplayer.html id="5G6psAuTYT4" %}
{% include youtubeplayer.html id="CYP3oL3Vs9o" %}
{% include youtubeplayer.html id="awQ97DOyOcM" %}

---

## What's new, in version 2

There are a bunch of new features in this release:

<div class="row row-cols-sm-2 row-cols-md-4 g-4">
{% include card.html cardtitle="New Pen Mechanism"  link="#quick-on-the-draw" %}
{% include card.html cardtitle="Battery holder" link="#battery-holder" %}
{% include card.html cardtitle="Pico holder" link="#pico-holder" %}
{% include card.html cardtitle="Access Point mode" link="#access-point-mode" %}
{% include card.html cardtitle="Logo Commands" link="#logo" %}
{% include card.html cardtitle="New look" link="#new-look" %}

</div>

---

## Quick on the Draw

BurgerBot now has the ability to draw, with the addition of a new mechanism.

{% include video_player.html video="/assets/img/blog/burgerbot/pen_mechanism.mov" %}

The Pen mechanism is designed to comfortably hold a Sharpie pen. There is small hole in the pen holder to enable an M2 screw to be inserted and tightend up against the pen to provide a secure grip. Just make sure you put the lid back on the Sharpie after use so it doesn't dry up.

[![Pen Wireframe](/assets/img/blog/burgerbot/pen.jpg){:class="img-fluid w-50"}](/assets/img/blog/burgerbot/pen.jpg)

The pen is at the very center of the robot, between the wheels which makes drawing shapes and patterns very easy as its the origin point of the robot.

---

## Battery Holder

We love the [Galleon Battery](https://shop.pimoroni.com/products/galleon-400mah-battery) from Pimoroni; it's a hardcase, 400mAh LiPo battery, with a convienient JST-PH connector.

The battery is now located in the rear support section, back to back with the Raspberry Pi Pico W.

[![Battery Holder CAD image](/assets/img/blog/burgerbot/battery_wireframe.png){:class="img-fluid w-25"}](/assets/img/blog/burgerbot/battery_wireframe.png)
[![Battery photo](/assets/img/blog/burgerbot/battery.jpg){:class="img-fluid w-25 rounded"}](/assets/img/blog/burgerbot/battery.jpg)

---

## Pico holder

The rear support also has mounting points for the Pico W, with holes for 4 M2 screws. There is also a small channel for the Battery wire to route from the other side.

[![Pico Holder](/assets/img/blog/burgerbot/pico_holder.png){:class="img-fluid w-75 rounded"}](/assets/img/blog/burgerbot/pico_holder.png)

Moving the Pico W from the top of the board to inside makes room for the new Pen mechanism.

Here is a photo of the Pico W, Pico Lipo Shim and Pico Motor Pack assembly, with the Galleon battery JST-PH connector visible.

[![Pico Holder](/assets/img/blog/burgerbot/pico_shim_motor.jpg){:class="img-fluid w-75 rounded"}](/assets/img/blog/burgerbot/pico_shim_motor.jpg)

---

## Access Point mode

You can now connect to BurgerBot using Wi-Fi via access point mode. This means you can remotely control the robot using a mobile phone, tablet or laptop simply by connecting to the Wi-Fi hot spot provided by BurgerBot.

When you connect to the Wi-Fi hot spot your browser will be redirected to the interally hosted web page, where you can control the robot.

---

## New Look

The front of BurgerBot now has an inset providing a fresh, clean new look. The wheels are also of the new, slimmer design, making for a more compact robot.

---

## Logo

One of the very first programmable robots for education used a language called Logo to issue movement commands. Logo also had a PenUp() and PenDown() command to raise and lower a pen. Taking inspration from this, BurgerBot will also support some Logo commands [^1]

[^1]: Coming soon

---

## Parts

Below is a list of parts with links - if you do some research you may be able to find the parts much cheaper.

Part                                                                                                     | Description                         | Qty           |      Price
---------------------------------------------------------------------------------------------------------|-------------------------------------|---------------|----------:
[Ultrasonic Range Finder](https://shop.pimoroni.com/products/ultrasonic-distance-sensor)                 | Measures distance using sound waves | 1             |      £6.90
[Motor SHIM for Pico](https://shop.pimoroni.com/products/motor-shim-for-pico)                            | drives the motors                   | 1             |      £9.60
[LiPo SHIM for Pico ](https://shop.pimoroni.com/products/pico-lipo-shim)                                 | powers the robot via a LiPo battery | 1             |       £7.5
[Micro Metal Motors](https://shop.pimoroni.com/products/micro-metal-gearmotor-with-motor-connector-shim) | moves the robot around              | 2             |      £5.46
[Moon Buggy Tyres](https://shop.pimoroni.com/products/moon-buggy-wheels-pair)                            | provide grip and traction           | 1             |      £4.50
[Galleon Battery](https://shop.pimoroni.com/products/galleon-400mah-battery)                             | 400mah hardcase LiPo Battery        | 1             |      £7.50
[SG90 Servo](https://kunkune.co.uk/shop/on-sale/micro-servo-motor-sg90/)                                 | moves the pen up and down           | 1             |      £1.99
M2 & M3 Nuts and bolts                                                                                   | For securing the parts together     | 14x M3, 6x M2 |    Various
                                                                                                         |                                     | **Total**     | **£48.81**
{:class="table table-striped"}

---

## STL Files

If you want to print out your own Burgerbot V2, download the files below:

* [Bottom Section](/assets/stl/burgerbot_v2/bottom.stl) - The Robot base
* [Motor Holders](/assets/stl/burgerbot_v2/motor_holder.stl) - The Motor holders; print 2 of these
* [Range Finder Holder](/assets/stl/burgerbot_v2/range_finder.stl) - The Range Finder holder
* [Battery Holder](/assets/stl/burgerbot_v2/battery_holder.stl) - The rear support Pico W and Battery Holder
* [Top](/assets/stl/burgerbot_v2/top.stl) - The Top section
* [Servo Holder](/assets/stl/burgerbot_v2/servo_holder.stl) - The Servo holder
* [Cog Wheel](/assets/stl/burgerbot_v2/cog_wheel.stl) - The pinion wheel that attached to the servo
* [Pen Holder](/assets/stl/burgerbot_v2/pen_holder.stl) - The Pen holder; holds the Sharpie.

---

## Notes