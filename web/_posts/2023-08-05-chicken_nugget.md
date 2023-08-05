---
layout: blog
title: Chicken Nugget of Doom
description: "Can a Chicken Nugget run Doom?"
short_title: Chicken Nugget of Doom
short_description: "Can a Chicken Nugget run Doom?"
date: 2023-08-05
author: Kevin McAleer
excerpt: 
cover: /assets/img/blog/chicken_nugget/cover.jpg
tags: 
 - Raspberry Pi
 - McDonalds
 - Doom
 - Pimoroni
 - Chicken Nugget
 - Hack
---

## Contents

{:toc}
* toc

---

## Video

{% include youtubeplayer.html id="4DH9DLzvx5Y" %}

---

## Can a Chicken Nugget run Doom?

To celebrate 40 years of the Chicken Nugget, McDonald's commissioned this crazy Chicken Nugget Tetris game. It was only available in China, so I got one of these on eBay, shipped over to the UK, and I wanted to have a quick play of it before I started to hack this because this can do so much more.

Inside is a custom PCB, a couple of buttons, and an on off switch, and a tiny little LCD screen. I decided to hack this with a Raspberry Pi Zero 2W to create... The Chicken Nugget of Doom.

![eBay](/assets/img/blog/chicken_nugget/ebay.jpg){:class="img-fluid w-75"}

---

## Bill of Materials

Item           | Description               | Qty |   Cost
---------------|---------------------------|:---:|------:
Pi Zero 2W     | Raspberry Pi Zero 2W      |  1  | £17.10
Display        | Pimoroni Display HAT Mini |  1  | £18.90
Happy Meal Toy | Chicken Nugget Tetris Toy |  1  | £15.00
{:class="table table-striped"}

---

## Disassembly

1. The back of the Nugget has 4 triangular shaped screws (called Tri-angle screws!), remove these - I used a iFixit set that included this type of screw driver head

1. Unscrew the PCB

1. Cut the cable to the speaker and Battery box

1. Remove the screen, but keep the rectangular orange side parts of the screen

1. Cut away the column supports near the screen so that the Display HAT can sit flush in the inside of the case

---

## Assembly

![Parts on a workbench](/assets/img/blog/chicken_nugget/parts.jpg){:class="img-fluid w-75"}

1. Place the Display HAT onto the Raspberry Pi Zero 2W's 40 pin header

1. Place the assembled Pi screen down into the nugget

1. Use the orange screen supports to mask the gap between the display and the case

1. I used Blu-tack to hold everything place, hot glue works fine too

1. Also put the buttons back into the case, for this version of the project I didn't connect these up, but it is possible to connect up some tact switches and wire them to the Pi's GPIO

---

## Setting up the Nugget for Doom

To get the display to work as an alternative to the HDMI display, takes a few extra steps:

1. Install the 32-bit version of Raspberry Pi OS

1. Follow [this tutorial](https://jackgovier.co.uk/2022/01/20/setting-up-pimoroni-display-hat-mini-with-raspberry-pi-zero-2-w/) to get the display to show the Raspberry Pi OS desktop

---

## Installing Doom

Next,  lets install Doom

1. Install Doom - from the terminal type:

    ```bash
    sudo apt install chocolate-doom
    ```

---

![Doom on a Chicken Nugget](/assets/img/blog/chicken_nugget/doom.jpg){:class="img-fluid w-50"}

You can now enjoy Doom on a Chicken Nugget!

---
