---
title: Pico W Toothbrush
description: Raspberry Pi Pico powered Toothbrush
layout: blog
cover: /assets/img/blog/picotoothbrush/picotoothbrush.png
short_title: Pico W Toothbrush
short_description: Raspberry Pi Pico powered Toothbrush
date: 2023-11-26
author: Kevin McAleer
excerpt: >-
    A 3D Printed Raspberry Pi Pico powered toothbrush you can build and program yourself
tags:
 - pico
 - robots
 - micropython
 - toothbrush
groups:
 - weird
 - 3dprinting
 - pico
---

{:toc}
* toc

---

## Pico Bluetoothbrush

The idea of this project is to create a Raspberry Pi Pico powered toothbrush. The Pico will be powered by a 3.7v LiPo battery, and will be connected to a motor driver board, which will drive a small DC motor. The motor will be connected to a small toothbrush head, and will be used to brush teeth.

There is a small display to show the time remaining, and a button to start the brushing cycle. The toothbush will contain a wireless charger, and in an internal lipo battery. The Pico W will run MicroPython and report the brushing time to a local MQTT server. It can also report data via bluetooth.

---

## The screen

The screen is 128x32 pixels, black and white OLED. It is connected to the Pico via I2C. The screen is used to display the time remaining, and the bluetooth status. The screen model is SSD1306.

Shows battery level, bruth time and bluetooth status.

---

## Code

The (work in progress) microPython code is available on github <https://www.github.com/kevinmcaleer/ssd1306_pico>

---

## BOM

Part               | Description             | Qty | Price
-------------------|-------------------------|:---:|-----:
Pico W             | Raspberry Pi Pico W     |  1  | £3.60
Screen             | 128x32 OLED             |  1  | £2.00
Button             | Push Button             |  1  | £0.10
Motor              | N20 100RPM DC Motor     |  1  | £5.00
Motor driver board | mx1508                  |  1  | £1.00
Battery            | 3.7v LiPo 400mAh        |  1  | £5.22
Charger            | Pimoroni LiPo Amigo Pro |  1  | £5.88
{:class="table table-striped"}

---

## Gallery

[![Pico W Toothbrush](/assets/img/blog/picotoothbrush/pico01.png){:class="img-fluid w-50"}](/assets/img/blog/picotoothbrush/pico01.png)

[![Pico W Toothbrush](/assets/img/blog/picotoothbrush/pico02.png){:class="img-fluid w-50"}](/assets/img/blog/picotoothbrush/pico02.png)

[![Pico W Toothbrush](/assets/img/blog/picotoothbrush/pico03.png){:class="img-fluid w-50"}](/assets/img/blog/picotoothbrush/pico03.png)

[![Pico W Toothbrush](/assets/img/blog/picotoothbrush/pico04.png){:class="img-fluid w-50"}](/assets/img/blog/picotoothbrush/pico04.png)

[![Pico W Toothbrush](/assets/img/blog/picotoothbrush/pico05.png){:class="img-fluid w-50"}](/assets/img/blog/picotoothbrush/pico05.png)

---
