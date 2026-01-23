---
layout: project
title: BurgerBot - a simple 3D Printable Robot
short_title: BurgerBot
short_description: A simple 3D Printable Robot
description: A simple 3D Printable Robot
difficulty: Beginner
date: 2022-08-24
author: Kevin McAleer
excerpt: Build your own 2 motor, Pico W-based, 3d printable robot
cover: /assets/img/blog/burgerbot/burgerbot.jpg
tags:
 - pico_w
 - pico
 - robot
 - micropython
groups:
 - raspberrypi
 - pico
 - micropython
 - robots
 - 3dprinting 
videos:
 - awQ97DOyOcM
repo:
 - https://www.github.com/kevinmcaleer/burgerbot
---

### BurgerBot

BurgerBot, as the name suggests, is a Burger-shaped robot. BurgerBot is quick and easy to print and assemble, with a low build cost. The Raspberry Pi Pico and Pico W board power the robot making it easy to program with MicroPython.

It uses two Micro Metal Motor which connect to the [Motor SHIM for Pico](https://shop.pimoroni.com/products/motor-shim-for-pico) from Pimoroni, and a Galleon Battery, which connects to the [LiPo SHIM for Pico](https://shop.pimoroni.com/products/pico-lipo-shim).

BurgerBot has an ultrasonic rangefinder mounted at the front; however, this needs to be the 3.3v variant (a later post will cover the wiring).

BurgerBot also features two moon-buggy wheels that attach directly to the motors. In addition, there are two little 'nubs' underneath to stop the robot from falling over backwards or forwards.

The top section has four mounting holes to secure the Pico using four M2 screws.

There are also a couple of `lego`[^1] compatible studs on the top section.

[^1]: Lego is a registered trade mark of Lego group.

---

### Parts

Part                                                                                                     | Description                         | Qty | Price
---------------------------------------------------------------------------------------------------------|-------------------------------------|-----|-----:
[Ultrasonic Range Finder](https://shop.pimoroni.com/products/ultrasonic-distance-sensor)                 | Measures distance using sound waves | 1   | £6.90
[Motor SHIM for Pico](https://shop.pimoroni.com/products/motor-shim-for-pico)                            | drives the motors                   | 1   | £9.60
[LiPo SHIM for Pico](https://shop.pimoroni.com/products/pico-lipo-shim)                                 | powers the robot via a LiPo battery | 1   |  £7.5
[Micro Metal Motors](https://shop.pimoroni.com/products/micro-metal-gearmotor-with-motor-connector-shim) | moves the robot around              | 2   | £5.46
[Moon Buggy Tyres](https://shop.pimoroni.com/products/moon-buggy-wheels-pair)                            | provide grip and traction           | 2   | £4.50
[Galleon Battery](https://shop.pimoroni.com/products/galleon-400mah-battery)                             | 400mah hardcase LiPo Battery        | 1   | £7.50
{:class="table table-striped"}

---

### Assembly

![BurgerBot Assembly Graphic](/assets/img/blog/burgerbot/assembly.png){:class="img-fluid w-50"}

---

### STL Files

Here are all the parts you need to print your own burgerbot:

* [Base](/assets/stl/burgerbot/base.stl)
* [Motor Holder](/assets/stl/burgerbot/motor_holder.stl) - print 2 of these
* [Top Section](/assets/stl/burgerbot/top_section.stl)
* [Rear Support](/assets/stl/burgerbot/support.stl)
* [Range finder](/assets/stl/burgerbot/rangefinder.stl)
