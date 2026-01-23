---
title: "KevsArcade"
description: >-
    Build your own miniature Arcade with Raspberry Pi's and a laser cutter
excerpt: >-
    KevsArcade is a project to create my own Arcade (yes, with multiple machines) using Raspberry Pi and a laser cutter
layout: showcase
date: 2024-11-01
author: Kevin McAleer
difficulty: beginner
cover: /assets/img/blog/kevsarcade/cover.jpg
hero:  /assets/img/blog/kevsarcade/hero.png
mode: light
tags:
 - laser_cutting
 - raspberry_pi
 - arcade
 - retro
groups:
 - lasercutting
 - raspberrypi
 - games
 - retro
videos:
 - 1O9-4kNRSrw
code:
 - https://www.github.com/kevinmcaleer/kevsarcade
---

## KevsArcade - a Raspberry Pi powered arcade cabinet

As a kid growing up in the 80s, I always loved going to arcades; the nearest arcades were in Manchesterm which was about an hours bus ride away. I remember seeing Teenage Mutant Ninja Turtles, Operation Wolf, Outrun and Pacman for the first time. I was hooked.

I always wanted to build my own arcade cabinet, but never had the space or the money to do it. Now, with the advent of the Raspberry Pi, I can build my own arcade cabinet, and so can you.

KevsArcade is a project to create my own Arcade (yes, with multiple machines) using Raspberry Pi and a laser cutter. The wood I'm using is 2mm basswood and one cabinet only requires 2 sheets of 300x300mm stock.

---

## Bill of Materials

Item              | Description                      | Item Cost | Qty |                Total
------------------|----------------------------------|----------:|:---:|--------------------:
Raspberry Pi 5    | Raspberry Pi 5 2Gb.              |       £38 |  1  |                  £38
2mm Basswood      | A 300x300 sheet of 2mm basswood. |        £1 |  3  |                   £3
Arcade Buttons    | 2x arcade buttons.               |     £1.50 |  1  |                   £3
Joystick          | 1 KY-023 joysticks               |        £1 |  1  |                   £1
Display           | Hyperpixel 4.0 non-touch         |       £50 |  1  |                  £45
Stand offs        | 7x 10mm stand offs               |        £8 |  1  |                   £8
Screws            | 7x 6mm M2.5 screws               |        £8 |  1  |                   £8
Raspberry Pi Pico | Raspberry Pi Pico                |        £5 |  1  |                   £5
                  |                                  |           |     | **Total Cost: £111**
{:class="table table-striped"}

---

## Assembly

The cabinet is designed to be cut from 3x sheets of 2mm basswood. The front control panel is designed to hold 2 arcade buttons and a joystick. The Hyperpixel 4.0 display is mounted to the inside of the cabinet on top of the Raspberry Pi 5 (you can switch this out of a raspberry pi 4 or earlier if you prefer).

The overall design is to use a tab and slot method to hold the cabinet together, with each panel being glued to the sides. Its easiest to add each peice to the left side panel, glueing as you go, and then finally marry up the right side panel (which can be a bit fiddlely at first).

![Joystick](/assets/img/blog/kevsarcade/joystick.jpg){:class="img-fluid w-100 rounded-3"}

---

## Software

The software I've chosen is RetroPie, the controls are mapped to the joystick and buttons.

---

## Custom joystick and button control panel

As the HyperPixel 4.0 display takes up all the Raspberry Pi GPIO pins, I decided to use a simple Raspberry Pi Pico running CircuitPython 9.2.0, as this can present as a HID (Human Interface Device) to the Raspberry Pi, and can be used to read the joystick and buttons. 

The code is available on my [GitHub](https://www.github.com/kevinmcaleer/kevsarcade).

Simply flash CircuitPython to the Pico, and copy the `code.py`, `boot.py` and `hid_gamepad.py` file to the Pico. The Pico will present as a USB keyboard to the Raspberry Pi. We can configure RetroPie to use this as the default controller.

---

## DXF Files

File                                                            | Description                                | Qty to cut
----------------------------------------------------------------|--------------------------------------------|:---------:
[`base.dxf`](/assets/dxf/kevsarcade/base.dxf)                   | Base of the cabinet                        |     1
[`carosel.dxf`](/assets/dxf/kevsarcade/carosel.dxf)             | Carosel for the joystick                   |     1
[`control_plate.dxf`](/assets/dxf/kevsarcade/control_plate.dxf) | Control plate for the buttons and joystick |     1
[`front_plate.dxf`](/assets/dxf/kevsarcade/front_plate.dxf)     | Front plate of the cabinet                 |     1
[`front.dxf`](/assets/dxf/kevsarcade/front.dxf)                 | Front of the cabinet                       |     1
[`left_side.dxf`](/assets/dxf/kevsarcade/left_side.dxf)         | Left side of the cabinet                   |     2
[`pi_holder.dxf`](/assets/dxf/kevsarcade/pi_holder.dxf)         | Raspberry Pi holder                        |     1
[`right_side.dxf`](/assets/dxf/kevsarcade/right_side.dxf)       | Right side of the cabinet                  |     2
[`top.dxf`](/assets/dxf/kevsarcade/top.dxf)                     | Top of the cabinet                         |     1
[`speakers.dxf`](/assets/dxf/kevsarcade/speakers.dxf)           | Speaker holder                             |     1
[`top.dxf`](/assets/dxf/kevsarcade/top.dxf)                     | Top of the cabinet                         |     1
[`screen.dxf`](/assets/dxf/kevsarcade/screen.dxf)               | Screen surround                            |     1
{:class="table table-striped"}

---

## Wiring up the USB HID controller

Connect the two arcade buttons to the Pico:

- `A` goes to `GPIO-00`
- `B` goes to `GPIO-01`
- `VRX` goes to `GPIO-26`
- `VRY` goes to `GPIO-27`
- `SW` goes to `GPIO-02`

![Wiring diagram](/assets/img/blog/kevsarcade/wiring.jpg){:class="img-fluid w-100 rounded-3"}

---

## RetroPie

RetroPie is a great piece of software, and I've been using it for years. I've got a few Raspberry Pi's running it, and I've always wanted to build my own arcade cabinet. I've got a few more ideas for this project, so stay tuned.

### Setting up RetroPie

I've created a specific tutorial on setting up RetroPie on the Raspberry Pi, you can find it [here](https://www.kevsrobots.com/blog/retropie2.html).

![Setting up retropie](/assets/img/blog/kevsarcade/setup_retropie01.jpg){:class="img-fluid w-100 rounded-3"}

![Setting up retropie](/assets/img/blog/kevsarcade/setup_retropie02.jpg){:class="img-fluid w-100 rounded-3"}

![Setting up retropie](/assets/img/blog/kevsarcade/setup_retropie03.jpg){:class="img-fluid w-100 rounded-3"}

{% include card.html cardtitle="Ultimate Guide to Setting Up RetroPie on a Raspberry Pi 4" link="/blog/retropie2.html" img="/assets/img/blog/retropie/retropie01.jpg" class="w-50" %}

---
