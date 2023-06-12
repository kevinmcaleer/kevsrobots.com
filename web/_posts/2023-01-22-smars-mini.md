---
layout: blog
title: SMARS Mini
short_title: SMARS Mini
short_description: Big Personality, Tiny Robot
date: 2023-01-22
author: Kevin McAleer
excerpt: Learn how to print your own SMARS Mini, download the parts, and get the electronics today
cover: /assets/img/blog/smars_mini/mini07.png
tags:
 - SMARS
 - Mini
 - Tiny Robot
 - 3D Printing
---

## Contents

{:toc}
* toc

---

## Videos

Head over to YouTube and start watching a SMARS Mini Video Overview and Tutorial today.

{% include youtubeplayer.html id="yiC5hiVpheg" %}

{% include youtubeplayer.html id="gJUsHlIcHdk" %}

---

## What is SMARS Mini

SMARS Mini is smaller version of the original SMARS Robot. It is 1/10 the size of the original, barely wide enough to house the two `N20 Motors`.

![SMARS Mini 3d render](/assets/img/blog/smars_mini/smarsmini.jpg){:class="img-fluid w-100 shadow-lg"}

![SMARS Mini 3d render](/assets/img/blog/smars_mini/pack04.jpg){:class="img-fluid w-100"}

---

## Tiny Electronics

SMARS Mini is so small it needs to have special, tiny electronics, to fit in the chassis. These include the `VL53L0X` Time of Flight sensor, that uses a tiny laser to sense objecst up to two meters away. These are just as cheap as the original sensor but much, much smaller.

![SMARS Mini 3d render](/assets/img/blog/smars_mini/pack01.jpg){:class="img-fluid w-100"}

![SMARS Mini 3d render](/assets/img/blog/smars_mini/pack02.jpg){:class="img-fluid w-100"}

![SMARS Mini 3d render](/assets/img/blog/smars_mini/pack03.jpg){:class="img-fluid w-100"}

---

## Download the STLS and print today

![SMARS Mini 3d render](/assets/img/blog/smars_mini/pack05.jpg){:class="img-fluid w-100"}

Download the files and start printing out your own SMARS Mini today.

SMARS Mini is available from Thingiverse, where you can grab all the STLs for the following parts:

* [`chassis.stl`](/assets/stl/smars_mini/chassis.stl) - The main robot body
* [`wheel.stl`](/assets/stl/smars_mini/wheel.stl) - The wheels, you'll need 4x of these
* [`face.stl`](/assets/stl/smars_mini/face.stl) - The ToF range finder holder (it looks like a face!)
* [`motor_driver.stl`](/assets/stl/smars_mini/motor_driver.stl) - The motor driver board holder

---

## SMARS Mini Electronics

### Tiny Electronics for a large amounts of fun

SMARS Mini is so small it needs to have special, tiny electronics, to fit in the chassis.

Item                     | Description       | Qty |  Price
-------------------------|-------------------|:---:|------:
Time Of Flight           | VL53L0X           |  1  | £19.50
L298N Motor Driver Board | L298N             |  1  | £12.90
Pimoroni Tiny2040        | Pimoroni Tiny2040 |  1  |  £8.40
N20 Motors               | N20 Motors        |  2  |  £5.10
{:class="table table-striped"}

---

## VL53L0X - Time of Flight sensor

These include the VL53L0X Time of Flight sensor, that uses a tiny laser to sense objecst up to two meters away. These are just as cheap as the original sensor but much, much smaller.

---

## L298N - Motor driver board

The Mini also uses a small L298N Motor driver board to control the N20 servos.

---

## Pimoroni Tiny2040

The brains of the SMARS Mini is a Pimoroni Tiny2040, which uses the same RP2040 chip as the Raspberry Pi Pico. Infact this chip has more flash storage than the Pico - coming in at 8Mb of flash storage.

---

## N20 Motors

SMARS Mini uses the same N20 motors found in the original SMARS

---

## Construction

[![](/assets/img/blog/smars_mini/mini00.png){:class="img-fluid w-25 shadow-lg"}](/assets/img/blog/smars_mini/mini00.png)

[![](/assets/img/blog/smars_mini/mini01.png){:class="img-fluid w-25 shadow-lg"}](/assets/img/blog/smars_mini/mini01.png)

[![](/assets/img/blog/smars_mini/mini02.png){:class="img-fluid w-25 shadow-lg"}](/assets/img/blog/smars_mini/mini02.png)

[![](/assets/img/blog/smars_mini/mini03.png){:class="img-fluid w-25 shadow-lg"}](/assets/img/blog/smars_mini/mini03.png)

[![](/assets/img/blog/smars_mini/mini04.png){:class="img-fluid w-25 shadow-lg"}](/assets/img/blog/smars_mini/mini04.png)

[![](/assets/img/blog/smars_mini/mini05.png){:class="img-fluid w-25 shadow-lg"}](/assets/img/blog/smars_mini/mini05.png)

[![](/assets/img/blog/smars_mini/mini06.png){:class="img-fluid w-25 shadow-lg"}](/assets/img/blog/smars_mini/mini06.png)

[![](/assets/img/blog/smars_mini/mini07.png){:class="img-fluid w-25 shadow-lg"}](/assets/img/blog/smars_mini/mini07.png)

## MicroPython Code

SMARS Mini uses the same code as the PicoSMARS robot:

<https://github.com/kevinmcaleer/picosmars>

---
