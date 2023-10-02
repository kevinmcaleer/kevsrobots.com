---
layout: blog
title: Worlds most advanced Bauble
short_title: Worlds most advanced Bauble
short_description: Fun with Cheerlights
description: Fun with Cheerlights
date: 2022-11-06
author: Kevin McAleer
excerpt: Build an internet controllable Christmas Bauble
cover: /assets/img/blog/cheerlights/cheerlights.jpg
tags:
 - Raspberry Pi Pico
 - 3D Printing
 - Christmas
 - Cheerlights
groups:
 - christmas
 - 3D Printing
 - pico
---

## Contents

{:toc}
* toc

---

# YouTube Video
Click the thumbnails below to watch the show all about this build.

{% include youtubeplayer.html id="UlY00Het18M" %}
{% include youtubeplayer.html id="ij1zMzfsl74" %}
{% include youtubeplayer.html id="R9eihHOKTUg" %}
{% include youtubeplayer.html id="I8WZ7ZEsHjM" %}

---

## Overview

In this tutorial we'll look at:

* Cheerlights - The Inspiration
* Wheatley - 3d design
* 3d Design - The printable parts
* Electronics - Wiring up
* MicroPython - Code

---

## Bill of Materials

Item                             | Description               | Qty |  Price
---------------------------------|---------------------------|:---:|------:
[Pimoroni Plasma Stick 2040 W ](https://shop.pimoroni.com/products/plasma-stick-2040-w)    | Raspberry Pi Pico, aboard |  1  | £12.00
[Adafruit Jewel 7x Neopixel model](https://www.adafruit.com/product/2226) | 7 RGB unit                |  1  |  $5.95
{:class="table table-striped"}


---

## Cheerlights

### What is it?

* CheerLights is an “Internet of Things” project created by Hans Scharler in 2011 
* Allows people’s lights all across the world to synchronize to one color set by Twitter
* This is a way to connect physical things with social networking experiences

You can change the colour of all cheerlights globally by Tweeting (from Twitter):

`#cheerlights blue`

Checkout Hans' amazing Cheerlights website here: <https://www.cheerlights.com>

---

## Wheatley - 3d design
Wheatley - the Portal 2 Character.

![](/assets/img/blog/cheerlights/slide.004.jpeg){:class="img-fluid w-100"}

Portal & Portal 2 are amongst my favourite games, I played portal which was bundled as part of the Orange Box Xbox 360 game a couple of years ago.

Wheatley is an AI character from Portal 2, and is described as an ***Intelligence Dampening Sphere***.

He’s round, so I’ve chosen to make a model based on this design, and it was also a reason to design something fun in Fusion 360.

---

### Wheatley 3d design

![](/assets/img/blog/cheerlights/slide.005.jpeg){:class="img-fluid w-100"}

The design is made up of 4x 3d printable parts:

* Two main halves (left and right)
* Eye piece
* Top holder loop

It was quite a complex design to model, involving the creation of a number of smaller and larger sphere, which are then use to subtrack material away from each other.

The design is also round - just like a Christmas Bauble!
It also about the same size a regular Christmas Tree Bauble.

> Grab the [STL files here](#stl-files)

---

### Left Half

![](/assets/img/blog/cheerlights/slide.006.jpeg){:class="img-fluid w-100"}

As you can see in the slide above, the left half has a hollow center, where the electronics are housed.

It also has two recessed sections that enable the eye piece and the top loop to be locked into place.

The tolerances are such that using the eye piece between the two halves provides sufficient friction and purchase to keep both halves together.

If the fit is different on your 3dprinted model, you can also use 2x M2 screws to secure the two halves together.

---

### Loop

The loop is simply for hanging the bauble from a tree (like a regular decoration).

![](/assets/img/blog/cheerlights/slide.007.jpeg){:class="img-fluid w-100"}

It features:

* Simple loop
* Attach some thread/cord to the loop
* Is secured by the small lip that extrudes all the way round
* Allows the USB cable to push into the center

---

### Eye
Diffuses the LED lights.

![](/assets/img/blog/cheerlights/slide.008.jpeg){:class="img-fluid w-100"}

The eye features:

* A simple, coin like, circle
* Attach the RGB LED using Hot-Glue / Blutac to the back of the eye
* Is secured be a small slot within each half
* Print in white, use an infill pattern to make the light defuse in interesting ways (I used hexagonal pattern)

---

## Electronics
![](/assets/img/blog/cheerlights/slide.011.jpeg){:class="img-fluid w-100"}

The electronics for this project are super simple:

* The Pimoroni [Plasma Stick 2040 W](https://shop.pimoroni.com/products/plasma-stick-2040-w)
  * Has a Raspberry Pi Pico W aboard
  * Has a reset button
  * QW/ST connector
  * 3 screw terminal for connecting the LED Strips
* [Adafruit Neopixel Jewel](https://www.adafruit.com/product/2226) (or compatible), watch out for RGB or RGBW, and RGB order
  

---

### Wiring up Electronics

![](/assets/img/blog/cheerlights/slide.012.jpeg){:class="img-fluid w-100"}

* Solder 3 wires to the back of the RGB led module:
    * **5v** - red wire
    * **GND** - black wire
    * **Data** in - blue wire

---

## MicroPython code

Download the project files from: <https://www.github.com/kevinmcaleer/christmas-cheer>

The main MicroPython code you will need is:

* `rgb.py` - the RGB helper library
* `pattern.py` - contains the code for `cheerlight_bauble.py`
* `cheerlight_bauble.py` - the main webserver code
* `index.html` - the web page template
* `header.html` - the web page header template
* `footer.html` - the web page footer template

There are also a couple of stand-alone demo programs:
* `colour.py` - a simple solid colour test
* `spin.py` - spins colours round the display
* `glow.py` - glow colours on the display

---

## STL files
Grab the 3d printable STL files here:

* [`left.stl`](/assets/stl/cheerlights/left.stl) - Left half
* [`right.stl`](/assets/stl/cheerlights/right.stl) - Right half
* [`eye.stl`](/assets/stl/cheerlights/eye.stl) - Eye piece
* [`loop.stl`](/assets/stl/cheerlights/loop.stl) - Top Loop