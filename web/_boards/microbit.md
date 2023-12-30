---
title: Micro:Bit
description: >-
  The BBC micro:bit is a pocket-sized computer that brings digital creativity and computational learning to your fingertips.
subtitle: A great pocket-sized computer for beginners
layout: board
manufacturer: Micro:Bit Educational Foundation
cover: /assets/img/boards/microbit.png
date: 2023-06-19
author: Kevin McAleer
excerpt: >-
  An innovative, user-friendly platform that educates and empowers students and enthusiasts alike to understand technology and develop core skills in digital technology, design and innovation
tags:
  - microbit
  - microcontroller
---

## Overview

The BBC micro:bit is a pocket-sized computer that brings digital creativity and computational learning to your fingertips. It's an innovative, user-friendly platform that educates and empowers students and enthusiasts alike to understand technology and develop core skills in digital technology, design and innovation. With the ability to code, customize and control your micro:bit, you can learn and create anything from basic games to advanced robotics.

---

## About the Manufacturer

The BBC micro:bit is a product of the British Broadcasting Corporation (BBC) in the UK. Designed and released in 2016, the board is a part of the corporation's Make It Digital campaign, aimed at inspiring young people to enter the world of digital technology. The project involves several tech partners, including Microsoft, the Python Software Foundation, Samsung, and many more.

---

## Target Audience

Primarily aimed at children and students aged 8 to 14, the BBC micro:bit is a valuable teaching tool for schools worldwide. However, it's not only for young learners. Hobbyists, educators, and even professional developers can take advantage of this versatile board to test out their ideas and prototype before scaling up.

---

## Onboard Sensors and Features

![Features](/assets/img/boards/features.jpg){:class="img-fluid w-100"}

The micro:bit is compact, but it packs in a host of sensors and features. The board comes with 25 programmable LEDs for simple outputs, two programmable buttons, an accelerometer, a compass, Bluetooth connectivity, and the ability to interact with other micro:bits and devices.

The micro:bit V2, released in 2020, also includes a microphone for sound detection and a speaker for audio output, as well as a capacitive touch sensor and a power saving mode, extending its versatility.

---

## Specification

The BBC micro:bit has an ARM Cortex-M0 processor, 16KB RAM, and 256KB Flash. It runs on a 3V coin cell or 2 x AAA batteries and has 5 I/O rings for connecting extra sensors or devices using alligator clips or 4mm banana plugs. The latest version, micro:bit V2, also features an upgraded processor, the Nordic nRF52833, providing users with a performance boost.

---

## Programming Languages

The micro:bit can be programmed using multiple languages, including the beginner-friendly Microsoft MakeCode, which offers a drag-and-drop visual interface, and MicroPython for more advanced users. This versatility means users can progress from a visual programming language to a more traditional scripting language as their skills develop.

---

## Fun Projects

The possibilities with the BBC micro:bit are vast. Beginners can start with simple projects such as creating a "Rock, Paper, Scissors" game using the LEDs and buttons, or a basic compass using the onboard magnetic sensor.

More advanced users might experiment with the micro:bit's Bluetooth capabilities, creating a remote-controlled robot, or a simple weather station by integrating external sensors. The addition of a microphone and speaker in the micro:bit V2 opens up new possibilities, like sound-activated projects or even a DIY smart speaker.

---


## Micro:Bit v2

The micro:bit can be used to sense, meaure and log values from all its onboard sensors:

* light
* temperature
* sound
* movement
* magnetism

Micro:bit also has:

* 2 user buttons
* 25 x red LEDs
* A Radio
* Bluetooth
* 5 large GPIO Pins for use with crocodile clips
* Micro-b USB conncetor
* JST-PH Battery connector

---

## GPIO Pinouts

One of the most valuable features of the BBC micro:bit is its General-Purpose Input/Output (GPIO) pins. These are a set of programmable pins that allow the micro:bit to interact with the outside world, expanding its capabilities by connecting with sensors, motors, and other components. 

The micro:bit has 25 pins in total: 23 GPIO pins, one 3V power pin, and one ground pin. Among the 23 GPIO pins, three are large, easy-to-use pins marked as 0, 1, and 2, and the rest are part of the edge connector and require a special connector to use.

Here's a brief rundown of the GPIO pinouts:

* **Pin 0, 1, and 2**: These pins are multi-functional. They can be used as digital inputs/outputs and can read analog sensors as well. Moreover, pins 0 and 1 are touch-sensitive and can act as capacitive touch sensors. Pin 0 can also be used to read the onboard temperature sensor.

* **Pin 3V**: This pin can supply 3V power to external devices.

* **Pin GND**: The ground pin, necessary to complete any electric circuit.

* **Pin 3 to Pin 16 and Pin 19 to Pin 20**: These pins are accessible through the edge connector. They provide digital input/output, with some providing additional functionality like PWM (Pulse Width Modulation) output, or serving as I2C or SPI bus pins.

* **Pin 17 and Pin 18**: Reserved for internal use (connected to the LED matrix).

* **Pin RESET**: This pin resets the micro:bit when connected to GND.

Please note that using some pins might interfere with the onboard sensors, so it's crucial to review your project needs and the micro:bit's specifications carefully before assigning tasks to the pins.

Connecting your micro:bit to other devices with the GPIO pins can make your projects more interactive and exciting. Whether it's controlling a servo motor, reading data from a temperature and humidity sensor, or playing music through a buzzer, the micro:bit's GPIO pins are the gateway to an expanded world of digital making.

---

In conclusion, the BBC micro:bit is a powerful yet accessible platform, perfect for those looking to dive into the world of digital creativity and computational thinking. Whether you're a student, a teacher, or a hobbyist, the micro:bit is an excellent tool to start your journey in programming and robotics.

---
