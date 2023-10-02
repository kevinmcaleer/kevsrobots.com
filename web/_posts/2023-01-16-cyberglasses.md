---
layout: blog
title: Cyberglasses
short_title: Cyberglasses
short_description: Wearable Tech
description: RGB LED & servo powered Cyberglasses
date: 2023-01-16
author: Kevin McAleer
excerpt: Learn how to 3D Print, and build your own Raspberry Pi Pico W powered Cyberglasses
cover: /assets/img/blog/cyberglasses/cyberglasses04.png
tags:
 - Wearable
 - Tips
 - 3D Printing
 - Cyberglasses
groups:
 - wearable
 - pico
 - 3dprinting
---

## Contents

{:toc}
* toc

---

### Video

Click here to watch the video:

{% include youtubeplayer.html id="jcGF-C6t1LI" %}

---

## CyberGlasses

I've been deeply inspired by Jorvon Moss [@Odd_Jayy](https://www.twitter.com/@Odd_Jayy) and his amazing Glasses, and I wanted to give this a go myself.

---

### Design

I've always loved Casey Neistat's Rayban sunglasses, so I decided to use that as the inspriation for my own design. I wanted to make them 3d printable, and mostly flat (because it prints much quicker).

The glasses will initially feature a single SG90 [servo](/resources/glossary#servo), connected to a 3d printed part that will hold an Adafruit [Neopixel](/resources/glossary#neopixel) ring light. The neopixel ring will be connected to a Raspberry Pi Pico, which means I can run all kinds of cool code on it to make the servo move and the RGB LEDs light up in different patterns.

In the inital version I have not included a battery or power connectivity; I'll look into introduce this in a v2 of the glasses.

---

### Bill of Materials

Item                 | Description                                                                                                           | Qty |         Price
---------------------|-----------------------------------------------------------------------------------------------------------------------|:---:|-------------:
SG90 Servo           | Any cheap SG90 servo will do                                                                                          |  1  |         £3.00
Pico/Pico W          | [Raspberry Pi Pico / Pico W](https://shop.pimoroni.com/products/raspberry-pi-pico)                                    |  1  | £3.60 / £6.30
Neopixel Ring        | [Adafruit 12 pixel Ring](https://shop.pimoroni.com/products/adafruit-neopixel-ring-24-x-rgb-led-w-integrated-drivers) |  1  |         £7.20
3d printing filament | PLA+ 0.5kg Yellow or White                                                                                            |  1  |        £14.99
M2 screws            | 4x M2 screws, to hold the pico in place                                                                               |  4  |         £1.00
3 Wires              | Red, Black and Blue copper wire , 30cm                                                                                              |  3  |         £1.00
{:class="table table-striped"}

---

### Construction

This is simple enough to construct:

1. **Print out the parts** - download and print out the STL files
1. **Solder wires** - Solder 3 wires to the Neopixel ring; VCC, GND and Data to the Pico
1. **Glue the Neopixel ring to the ring holder part** - Superglue works fine
1. **Hinges** - Use a piece of 3D printing filament for the hinges
1. **Servo** - screw the servo into the frame, remove the 3 pin connector and attach the wires as per the wiring diagram below
1. **Neopixel ring** - attach the Neopixel holder to the servo horn, screw in the little screw to secure
1. **Pico** - screw the Pico into the Arm using 4x M2 screws

---

### MicroPython code

The code for this project is available here: <https://github.com/kevinmcaleer/cyber_glasses>

---

### Gallery

![Cyberglasses](/assets/img/blog/cyberglasses/cyberglasses01.png){:class="img-fluid w-50"}

![Cyberglasses](/assets/img/blog/cyberglasses/cyberglasses02.jpg){:class="img-fluid w-50"}

![Cyberglasses](/assets/img/blog/cyberglasses/cyberglasses03.jpg){:class="img-fluid w-50"}

![Cyberglasses](/assets/img/blog/cyberglasses/cyberglasses05.jpg){:class="img-fluid w-50"}

![Cyberglasses](/assets/img/blog/cyberglasses/cyberglasses06.jpg){:class="img-fluid w-50"}

---

### STL Files

Here are the 3d printable STL files:

* [Glasses Body](/assets/stl/cyberglasses/glasses.stl)
* [Right Arm](/assets/stl/cyberglasses/rightarm.stl)
* [Left Arm](/assets/stl/cyberglasses/leftarm.stl)
* [NeoPixel holder](/assets/stl/cyberglasses/neopixel_holder.stl)

---
