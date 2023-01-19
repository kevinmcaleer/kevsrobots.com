---
layout: blog
title: Shift Registers
short_title: How it works - Shift Registers
short_description: Learn about how Shift Registers works
date: 2023-01-18
author: Kevin McAleer
excerpt: 
cover: /assets/img/how_it_works/shift_registers.png
tags:
 - Robot
 - Tips
 - Shift Registers
 - SN74HC595
---

`Shift registers` are a type of sequential logic circuit used to store and transfer data. They are often used to reduce the number of [GPIO](/resources/glossary#gpio) pins needed from a Microcontroller, such as the Arduino or Raspberry Pi Pico to control another device, such as a keyboard or keypad.

They are typically composed of a number of flip-flops, each of which is capable of storing a single bit of data. 
The data is transferred in and out of the register through a series of clock pulses. Each pulse shifts the data one bit at a time, either to the left or right, depending on the type of register used.

Shift registers are commonly used in communication and audio/video systems to store and transmit data in a serial format. They can also be used to convert parallel data into serial data and vice versa.

## SN74HC595

This `SN74HC595` is common shift register that takes a serial input (SPI) of 1 byte (8 bits) and then output those digital bits onto 8 pins. They can be chained together, for example, putting three in a row with the serial output of one plugged into the serial input of another to make 3 x 8 = 24 digital outputs.

---

## Video

In this video I show you how to use shift registers to reduce the number of pins required to control several seven segment displays.

{% include youtubeplayer.html id="iSTlP4Lbibs" %}

---
