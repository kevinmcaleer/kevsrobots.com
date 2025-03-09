---
title: "Chicken Nugget Piano"
description: "Learn how to use touch sensors with Raspberry Pi Pico to make a chicken nugget piano"
excerpt: >-
   Making touch sensors is easier than you'd think.
layout: showcase
date: 2025-03-08
author: Kevin McAleer
difficulty: beginner
cover: /assets/img/blog/chicken_nugget_piano/cover.jpg
hero:  /assets/img/blog/chicken_nugget_piano/hero.png
mode: light
videos:
  - MWBl0E1Z8Ps
tags:
 - Raspberry Pi Pico
 - touch
 - circitpython
 - micropython
groups:
 - pico
 - micropython
 - music
 - weird
code:
 - https://www.github.com/kevinmcaleer/PicoTouch
---

Setting up touch sensors with Raspberry Pi Pico is easier than you'd think. In this project, we will use the touch sensors to make a chicken nugget piano.

---

## 1 pin Capacitive Touch Sensors

Capacitive touch sensors work by detecting the change in capacitance when a conductive object (such as a human finger) comes in contact with the sensor. The human body acts as a [capacitor](/resources/glossary#capacitor), and when it comes in contact with the sensor, the capacitance of the sensor changes. This change in capacitance is detected by the sensor, and the touch is registered.

It only requires a single pin to read the touch sensor, this means we can have 26 touch sensors on a single Raspberry Pi Pico.

---

## How Capactive Touch works?

We can detect the touch by measuring the time it takes for the GPIO pin to discharge. When the touch sensor pad is not touched, the GPIO pin will discharge slowly. When the touch sensor pad is touched, the GPIO pin will discharge quickly. By measuring the time it takes for the GPIO pin to discharge, we can detect the touch.

To filter our the noise we can use a `threshold` value, if the time it takes for the GPIO pin to discharge is less than the threshold value, we can assume that the touch sensor pad has been touched.

{% include gallery.html images="/assets/img/blog/chicken_nugget_piano/how_it_works03.jpg,/assets/img/blog/chicken_nugget_piano/threshold.jpg,/assets/img/blog/chicken_nugget_piano/how_it_works01.jpg,/assets/img/blog/chicken_nugget_piano/how_it_works02.jpg" cols=1 %}

---

## Making a Piano

{% include gallery.html images="/assets/img/blog/chicken_nugget_piano/piano01.jpg,/assets/img/blog/chicken_nugget_piano/piano02.jpg" titles="Cardboard foil piano, Close up" cols=2 %}

---

The easiest way to start is to cut out some aluminium foil and stick it to some cardboard - I laser-cut some wood using a simple 1 octave piano layout. I then stuck the foil to the back of the wood and connected it to the Raspberry Pi Pico using crocodile clips, and jumper wires. The 1M Ohm resistor is connected to the GPIO pin and a GND pin.

I plugged the Pico into a breadboard to make the wiring easier, there is just enough room to put a jumper wire and 1M Ohm resistor in the same row on the breadboard. I then connect the crocodile clips to the jumper wire and also to the foil on the piano.

Download the [Touch Piano PDF](/assets/pdf/touch_piano/touch_piano.pdf) here.

---

## Nugget Piano

{% include gallery.html images="/assets/img/blog/chicken_nugget_piano/nugget01.jpg,/assets/img/blog/chicken_nugget_piano/nugget02.jpg,/assets/img/blog/chicken_nugget_piano/nugget03.jpg,/assets/img/blog/chicken_nugget_piano/nugget04.jpg" titles="Nugget, Nugget Piano, Overhead, Chicken Nugget Piano" cols=2 %}

---

## Setting up the Pico

The easiest way to use touch in your projects is to use CircuitPython as this has a mature touch library. You can also use MicroPython, but you will need to write your own touch library (which I have [here]({{page.code}}) if you're interested, make sure to upload it to the pico to use it in your projects).

To install CircuitPython, simply load up [Thonny IDE](https://thonny.org/) hold the boot button down when attaching the Pico and then click on the 'Install CircuitPython' button at the bottom of the screen. This will install the latest version of CircuitPython on your Pico.

{% include gallery.html images="/assets/img/blog/chicken_nugget_piano/circuit.png, /assets/img/blog/chicken_nugget_piano/circuit01.jpg" titles="Wiring" cols=2 %}

---

You will also need to drag the touch and midi libraries from the [Adafruit CircuitPython touch library](https://circuitpython.org/libraries) onto the Pico as well, in the `lib` folder.

---

## Midi

`Midi` is a protocol that allows electronic musical instruments, computers, and other equipment to communicate with each other. It is used to send information about the pitch, velocity, and duration of a note, as well as other information such as the instrument being used.

We can use the `Midi` library in our project to send the note information to a computer or other device that can play the note. This allows us to create a simple piano using the touch sensors and the Raspberry Pi Pico.

I connected the pico using a USB cable to my computer and used garage band to play and hear the notes.

---
