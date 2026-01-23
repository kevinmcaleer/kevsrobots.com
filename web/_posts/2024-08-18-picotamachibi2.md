---
title: Picotamachibi 2
description: >-
    Build a Raspberry Pi Pico 2 powered Virtual Pet with a 1.3" OLED display and some MicroPython
excerpt: >-
layout: showcase
date: 2024-08-18
date_updated: 2024-11-22
author: Kevin McAleer
difficulty: beginner
cover: /assets/img/blog/picotamachibi2/picotamachibi2.jpg
hero:  /assets/img/blog/picotamachibi2/hero.png
mode: light
tags: 
 - raspberry_pi
 - pico
groups:
    - raspberrypi
    - pico
    - microPython
    - pets
    - pcb
    - electronics
videos:
 - c6D1JRDddkE
 - btG3Pd8ZlBw
 - taWCovWiTJ0
code:
 - https://github.com/kevinmcaleer/picotamachibi
---

## Version 2.0

The `Picotamachibi 2` is a ***Raspberry Pi Pico 2*** powered Virtual Pet with a 1.3" OLED display. It's a fun project that combines hardware and software to create a digital pet that you can interact with. 

The project uses a Pico 2, a 1.3" OLED display, and some MicroPython code to create a virtual pet that you can feed, play with, and take care of.

![PCB](/assets/img/blog/picotamachibi2/pico2.jpg){:class="img-fluid w-100 card-shadow card-hover rounded-3"}

This is a follow up of the [Picotamachibi 1](/learn/projects/picotamachibi) project, where we built a similar virtual pet using a Raspberry Pi Pico.

The Picotamachibi 2 takes things a step further by using the new Pico 2 board, which has more memory and a faster processor, allowing for more complex interactions and animations.

This project also features a custom PCB that makes it easy to connect the Pico 2 to the OLED display and other components. The PCB includes a breakout for the Pico 2, a breakout for the OLED display, and a breakouts for the 3 control buttons.

---

## Bill of Materials

Item         | Description                                      |  Qty  |  Price
-------------|--------------------------------------------------|:-----:|------:
Pico         | Raspberry Pi Pico 2                              |   1   |  £5.00
3x Buttons   | 3x [Tact](/resources/how_it_works/tact) switches |   3   |  £0.50
Solrde       | solder                                           |   1   |  £0.50
OLED Display | SSD1306 128x64 OLED Display                      |   1   | £16.00
Custom PCB   | Picotamachibi Custom PCB                         |   1   |  £4.00
             |                                                  | Total | £25.00
{:class="table table-striped"}

> Custom PCBs
>
> The Custom PCBs will be available for sale soon. Please check back for updates.

---

## Wiring

The wiring is pretty simple, though you may have to keep the wires short to fit everything in the case.

![Wiring](/assets/img/blog/picotamachibi2/wiring.jpg){:class="img-fluid w-100"}

Pico GPIO  | Component
-----------|----------
`GPIO00`   | SDA
`GPIO01`   | SCL
`3.3v out` | VCC
`GND`      | GND
`GPIO04`   | Button A
`GPIO03`   | Button B
`GPIO02`   | Button X
{:class="table table-striped"}

---

## Assembly

![Assembly](/assets/img/blog/picotamachibi2/pcb.jpg){:class="img-fluid w-100 card-shadow card-hover rounded-3"}

The assembly of the Picotamachibi 2 is straightforward. You will need to solder the Pico 2 to the custom PCB, solder the OLED display to the PCB, and connect the buttons to the PCB.

![Assembly](/assets/img/blog/picotamachibi2/picotamachibi2.jpg){:class="img-fluid w-100 card-shadow card-hover rounded-3"}

![Assembly](/assets/img/blog/picotamachibi2/pcb_layout.jpg){:class="img-fluid w-100 card-shadow card-hover rounded-3"}

---

## Interface

The Picotamachibi 2 has a simple interface that allows you to interact with your virtual pet. The OLED display shows the pet's status, such as its hunger level, happiness level, and health level.

There are 3 buttons to navigate the Toolbar at the top of the screen.

![Interface](/assets/img/blog/picotamachibi2/interface.jpg){:class="img-fluid w-100 card-shadow card-hover rounded-3"}

Button   | Function
---------|-----------------------------------------
Button A | Moves the Toolbar selection to the right
Button B | Selects the current Toolbar item
Button X | Cancels the current selection
{:class="table table-striped"}

---

## Toolbar Icons

Icon      | Description
----------|--------------------------------------------------------------------
Food      | reduces hunger and makes Baby happy
Lightbulb | makes baby sleep, increases energy, increases happiness
Game      | not implemented yet, increases health and happiness, reduces energy
First aid | increases health
Toilet    | cleans any poop onscreen (which decreases happiness and health)
Heart     | check on the status of Energy, Happiness and Health
Call      | Flashes when Baby wants attention
{:class="table table-striped"}

![Toolbar](/assets/img/blog/picotamachibi2/toolbar.jpg){:class="img-fluid w-100 card-shadow card-hover rounded-3"}

---
