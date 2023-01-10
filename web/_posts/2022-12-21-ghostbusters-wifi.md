---
layout: blog
title: Ghostbusters Wifi Scanner
short_title: Ghostbusters Wifi Scanner
short_description: Who you gonna call?
date: 2022-12-21
author: Kevin McAleer
excerpt: 3d Print a Ghostbusters PKE style Wifi Scanner
cover: /assets/img/blog/ghostbusters/ghost02.jpg
tags:
 - Raspberry Pi Pico
 - Ghostbusters
 - Wifi
 - Scanner
 - Movie Prop
---

## YouTube Video

Click the thumbnails below to watch the show all about this build.

{% include youtubeplayer.html id="XTUOUlSnVpc" %}

## Overview

Build a fun Ghostbusters PKE style Wifi scanner using a Raspberry Pi Pico, a couple of electronics and some 3d printed parts. The scanner will scan and display a list of all the local wifi hotspots, and change the position of the arms depending on the signal strength.

[![Picture of wifi scan](/assets/img/blog/ghostbusters/ghost01.jpg){:class="img-fluid w-25"}](/assets/img/blog/ghostbusters/ghost01.jpg)
[![Picture of wifi scan](/assets/img/blog/ghostbusters/ghost03.jpg){:class="img-fluid w-25"}](/assets/img/blog/ghostbusters/ghost03.jpg)
[![Prototype with mechanism showing](/assets/img/blog/ghostbusters/proto01.jpg){:class="img-fluid w-25"}](/assets/img/blog/ghostbusters/proto01.jpg)

---

## Bill of Materials

You will also need the following electronics:

Item                   | Description                                                                                                       | Qty |  Price
-----------------------|-------------------------------------------------------------------------------------------------------------------|----:|------:
Display Pack 2.0       | [Pimoroni Display Pack for Pico](https://shop.pimoroni.com/products/pico-display-pack-2-0?variant=39374122582099) |   1 | £18.90
Pico W                 | [Raspberry Pi PicoPico W](https://shop.pimoroni.com/products/raspberry-pi-pico-w?variant=40059369619539)          |   1 |  £6.30
DS-929MG Digital Servo | [DS-959MG Servo](https://shop.pimoroni.com/products/ds-929mg-digital-servo?variant=1015994157)                    |   1 |  £9.60
Pico Proto             | [Pico Proto board](https://shop.pimoroni.com/products/pico-proto?variant=32369530110035)                          |   1 |  £2.10
4x M2 screws           | 2M 8mm screws                                                                                                     |   4 |  £1.00
4x 2M 12mm screws      | 4x 2M 12mm screws                                                                                                 |   4 |  £1.00
1x M2.5 screw and nut  | M3 nut and bolt                                                                                                   |   1 |  £0.25
3 male Dupont cables   | To connect the servo to the protoboard                                                                            |   1 |  £0.30
Galleon Battery        | [Pimoroni 400mAh LiPo Battery](https://shop.pimoroni.com/products/galleon-400mah-battery?variant=40061068673107)  |   1 |  £7.50
LiPo Amigo Pro         | [Amigo Pro LiPo battery charger](https://shop.pimoroni.com/products/lipo-amigo?variant=39779302539347)            |   1 |  £8.40
JST-PH cable           | To connect the Pico Proto board to the LiPo Amigo Pro                                                             |   1 |  £0.50
{:class="table table-striped"}

---

## 3d Design

The Scanner is made up of several components:

* Back
* Left Arm
* Right Arm
* Middle section
* Back Layer
* Handle
* Top

[![Back](/assets/img/blog/ghostbusters/3d_01.png){:class="img-fluid w-25"}](/assets/img/blog/ghostbusters/3d_01.png)
[![Servo](/assets/img/blog/ghostbusters/3d_02.png){:class="img-fluid w-25"}](/assets/img/blog/ghostbusters/3d_02.png)
[![Battery](/assets/img/blog/ghostbusters/3d_03.png){:class="img-fluid w-25"}](/assets/img/blog/ghostbusters/3d_03.png)
[![Middle section](/assets/img/blog/ghostbusters/3d_04.png){:class="img-fluid w-25"}](/assets/img/blog/ghostbusters/3d_04.png)
[![Handle](/assets/img/blog/ghostbusters/3d_05.png){:class="img-fluid w-25"}](/assets/img/blog/ghostbusters/3d_05.png)
[![Arms](/assets/img/blog/ghostbusters/3d_06.png){:class="img-fluid w-25"}](/assets/img/blog/ghostbusters/3d_06.png)
[![Top](/assets/img/blog/ghostbusters/3d_07.png){:class="img-fluid w-25"}](/assets/img/blog/ghostbusters/3d_07.png)
[![Pico](/assets/img/blog/ghostbusters/3d_08.png){:class="img-fluid w-25"}](/assets/img/blog/ghostbusters/3d_08.png)
[![Screen](/assets/img/blog/ghostbusters/3d_09.png){:class="img-fluid w-25"}](/assets/img/blog/ghostbusters/3d_09.png)

---

## Construction

[![build 1](/assets/img/blog/ghostbusters/build01.jpg){:class="img-fluid w-25"}](/assets/img/blog/ghostbusters/build01.jpg)
[![build 2](/assets/img/blog/ghostbusters/build02.jpg){:class="img-fluid w-25"}](/assets/img/blog/ghostbusters/build02.jpg)
[![build 3](/assets/img/blog/ghostbusters/build03.jpg){:class="img-fluid w-25"}](/assets/img/blog/ghostbusters/build03.jpg)
[![build 4](/assets/img/blog/ghostbusters/build04.jpg){:class="img-fluid w-25"}](/assets/img/blog/ghostbusters/build04.jpg)
[![build 5](/assets/img/blog/ghostbusters/build05.jpg){:class="img-fluid w-25"}](/assets/img/blog/ghostbusters/build05.jpg)
[![build 6](/assets/img/blog/ghostbusters/build06.jpg){:class="img-fluid w-25"}](/assets/img/blog/ghostbusters/build06.jpg)
[![build 7](/assets/img/blog/ghostbusters/build07.jpg){:class="img-fluid w-25"}](/assets/img/blog/ghostbusters/build07.jpg)

1. Push the `servo` into the hole on the `base`
1. Place the `LiPo Amigo pro`, the `Galleon battery` onto the `back`, connecting up the `battery` to the `LiPo Amigo Pro`
1. There are mounting screws for the `LiPo amigo Pro`, though it can just float around in the compartment
1. Place the `middle section` on top of the `base`
1. Place the `left arm` and `right arm` on the underside of the `Top` section
1. Place the `middle layer` on top of the `left arm` and `right arm`
1. Push the `servo` cable through the hole in the `top` section and `middle layer`
1. Place the `Pico W` on to the `top` section with the pins facing up
1. Solder the wires onto the `Pico Proto board` as described above and push this onto the `Pico W`
1. Push the `Display pack` on top of the `Pico Proto board`
1. Connect the `Servo` to the `3 pin dupont cable`
1. Connect the `JST-PH` power cable from the `Pico Proto board` to the `device` connector of the `LiPo Amigo Pro`

---

## Electronics

[![Wiring](/assets/img/blog/ghostbusters/wiring.jpg){:class="img-fluid w-25"}](/assets/img/blog/ghostbusters/wiring.jpg)
[![Wiring Diagram](/assets/img/blog/ghostbusters/wiring_diagram.jpg){:class="img-fluid w-50"}](/assets/img/blog/ghostbusters/wiring_diagram.jpg)

Although this picture of the wiring looks a little complicated, its actually pretty straight forward:

* The `VCC` and `GND` from the `device` connector the LiPo Amigo Pro connects to the VBus and GND of the Pico
* The `VCC` and `GND` from the `device` connector the LiPo Amigo Pro connects to the Servos 5V and GND
* The Pico `GPIO Pin 0` connects to the `Servo signal pin`
* The `Display pack` pushes on top of the `Pico W` with the `Pico Proto board` sandwiched inbetween
* The `Pico Proto board` `VCC` and `GND` connect to the `JST-PH` connector
* The `JST-PH` connector from the `Pico Proto board` connects to the `LiPo Amigo Pro`

---

## MicroPython code

The files you need to copy to the pico can be found here: <https://github.com/kevinmcaleer/ghostbusters_wifi>. The files you need to upload to the Pico W are:

* `arms.py`
* `background.jpg`
* `gui.py`
* `list.jpg`
* `scanning.jpg`
* `splash.jpg`

---

## User Interface

[![ghost logo](/assets/img/blog/ghostbusters/ghost.jpg){:class="img-fluid w-50"}](/assets/img/blog/ghostbusters/ghost.jpg)

The user interface is made up of a couple of simple screens:

![User Interface](/assets/img/blog/ghostbusters/gui01.jpg){:class="img-fluid w-100"}

![Files to run](/assets/img/blog/ghostbusters/gui02.jpg){:class="img-fluid w-100"}

![Files to run](/assets/img/blog/ghostbusters/splash.jpg){:class=""}
![Files to run](/assets/img/blog/ghostbusters/scanning.jpg){:class=""}
![Files to run](/assets/img/blog/ghostbusters/background.jpg){:class=""}

Use the `Y` and `X` buttons on the Pico Display Pack 2.0 to move the list selection `up` and `down`.
Press the `A` button to reset the program.

---

## STL files

You can download the STLs for 3d printing here:

* [`top.stl`](/assets/stl/ghostbusters/top.stl) - Top Section
* [`back.stl`](/assets/stl/ghostbusters//back.stl) - Back
* [`middle_section.stl`](/assets/stl/ghostbusters/middle_section.stl) - Middle Section
* [`back_layer.stl`](/assets/stl/ghostbusters/back_layer.stl) - Back Layer
* [`handle.stl`](/assets/stl/ghostbusters/handle.stl) - Handle
* [`left_arm.stl`](/assets/stl/ghostbusters/left_arm.stl) - Left Arm
* [`right_arm.stl`](/assets/stl/ghostbusters/right_arm.stl) - Right Arm

The arms are best printed in white PLA+, the rest of the parts can be printed in black PLA+.

---
