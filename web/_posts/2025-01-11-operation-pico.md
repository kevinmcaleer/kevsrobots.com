---
title: "Operation Pico"
description: >-
    Recreate the classic Operation game using a Raspberry Pi Pico and some croc clips
excerpt: >-
    Build a Raspberry Pi Pico version of the classic Operation game
layout: showcase
date: 2025-01-11
author: Kevin McAleer
difficulty: beginner
cover: /assets/img/blog/operation_pico/cover.jpg
hero:  /assets/img/blog/operation_pico/hero.png
mode: light
tags:
 - raspberry_pi_pico
 - pico
 - micropython
 - kids
 - electronics
groups:
 - raspberry pi
 - micropython
 - kids
videos:
 - Gjz6LPaaRmY
code:
 - https://www.github.com/kevinmcaleer/operation_pico
---

`Operation Picb` is a fun project that I built using a Raspberry Pi Pico (the Pimoroni pico Jumbo), laser cut pieces, a large LED and some croc clips. The project is a recreation of the classic Operation game, where you have to remove objects from a patient's body without touching the sides of the cavity. The game is a fun way to learn about electronics and programming, and it's a great project for kids and beginners.

---

## Bill of Materials

Item                | Description                  | Quantity |  Price |  Total
--------------------|------------------------------|:--------:|-------:|------:
Pimoroni Pico Jumbo | RP2350 based Microcontroller |    1     | £15.90 | £15.90
LED                 | Large LED                    |    1     |  £2.00 |  £2.00
Croc Clips          | Crocodile Clips              |    2     |  £0.50 |  £1.00
Laser Cut Pieces    | Basswood pieces for the game |    1     |  £5.00 |  £5.00
Monk Makes speaker  | Speaker for sound effects    |    1     |  £9.67 |  £9.67
Tin Foil            | For the cavity               |    1     |  £1.00 |  £1.00
Total               |                              |          |        | £34.57
{:class="table table-striped"}

---

## Pimoroni Pico Jumbo

When the Pico 2 was launched Pimoroni released a large version called the Pico Jumbo. One of the unique features of the Pico Jumbo is the crocodile and banana connector compatible pads. This makes it easy to connect croc clips to the Pico Jumbo and use it in projects like this one.

---

## Monk Makes Speaker

The original Operation game simply had a buzzer that was connected to the battery and made a circuit whenever the tweezers touched the sides of the cavity. I wanted to add a bit more fun to the game so I added a Monk Makes speaker to the Pico Jumbo and played a sound effect whenever the tweezers touched the sides of the cavity.

The Monk Makes speaker module also has crocodile clip connectors, so it was easy to connect it to the Pico Jumbo and play sound effects in the game - no soldering required!

---

## Pins used

The Pico Jumbo has a lot of pins, so I used the following pins for the game:

- GP0 - LED
- GND - LED GND
- GP1 - Tweezer 1
- GND - Tweezer 2
- GP15 - Monk Makes speaker signal
- 3V3 - Monk Makes speaker power
- GND - Monk Makes speaker ground

---

## Code

I used MicroPython to program the Pico Jumbo. The code is simple and uses the `machine` module to control the pins and play sound effects. The game has a main loop that checks if the tweezers are touching the sides of the cavity and plays a sound effect if they are.

Make sure you copy the `music.py` and `tunes.py` files to the Pico Jumbo as well as the `operation.py` file. To automatically run the game when the Pico Jumbo is powered on, rename the `operation.py` file to `main.py`.

<script src="https://gist.github.com/kevinmcaleer/f823715d9e11a92dba77dd8d927147e4.js"></script>

---

## Laser cut DXF files

To create the game pieces, I designed the pieces in Fusion 360 and exported them as DXF files. I then used Lightburn to cut these on my 5w laser cutter. The pieces are made from basswood and are easy to assemble and paint

Here are the DXF files for the game pieces:

- [Table](/assets/dxf/operation_pico/table.dxf) - The table that the Patient sits on
- [Belly](/assets/dxf/operation_pico/belly.dxf) - The patient's belly
- [Hair](/assets/dxf/operation_pico/Hair.dxf) - The patient's hair
- [Nose](/assets/dxf/operation_pico/nose.dxf) - The patient's nose
- [Nose aligher](/assets/dxf/operation_pico/nose_aligner.dxf) - For 5mm LEDs nose
- [Body](/assets/dxf/operation_pico/body.dxf) - The patient's body

---

## Gallery

{% include gallery.html images="/assets/img/blog/operation_pico/build01.jpg, /assets/img/blog/operation_pico/build02.jpg, /assets/img/blog/operation_pico/build03.jpg, /assets/img/blog/operation_pico/build04.jpg, /assets/img/blog/operation_pico/build05.jpg, /assets/img/blog/operation_pico/build06.jpg, /assets/img/blog/operation_pico/build07.jpg " titles='Laser Cutting, Ready for painting, Paint job done, Face close up, Belly, Table, Finished model' %}