---
title: "10 Projects for your Raspberry Pi Pico"
description: >-
    Here is a collection of projects that you can build with your Raspberry Pi Pico 
excerpt: >-
    If you've just got a new Raspberry Pi Pico and you're looking for some inspiration, then you've come to the right place. Here is a collection of projects that you can build with your Raspberry Pi Pico.
layout: showcase
date: 2024-11-22
author: Kevin McAleer
difficulty: beginner
cover: /assets/img/blog/picoprojects/cover.jpg
hero:  /assets/img/blog/picoprojects/hero.png
mode: light
tags:
 - raspberry_pi
 - micropython
 - raspberry_pi_pico
groups:
 - raspberrypi
 - micropython
# videos:
#  - 

---

## Introduction

The `Raspberry Pi Pico` was introduced in January 2021 and has since become a popular choice for hobbyists and makers. With its powerful RP2040, and later RP2350, microcontrollers and support for MicroPython, the Pico is a versatile platform for building a wide range of projects.

Meet the family:

{% include boards_mini.html family="pico" %}

---

## 1. **LED Blink**

The classic "Hello, World!" of microcontroller projects, the LED blink program is a simple way to get started with the Raspberry Pi Pico. By toggling an LED on and off at regular intervals, you can verify that your Pico is working correctly and familiarize yourself with the basics of programming.

<div class="row row-cols-4">

{% include card.html cardtitle="Blink the onboard LED" description="Blink the onboard LED" link="https://projects.raspberrypi.org/en/projects/getting-started-with-the-pico/5" img="https://projects-static.raspberrypi.org/projects/getting-started-with-the-pico/b3f5d25071fc53deef4a41b31eb1e1fdd1fd75d1/en/images/banner.png" tl="2" dl="3" %}

</div>

---

## 2. **Temperature Sensor**

Did you know the Pico has a built-in temperature sensor? By reading the temperature value from the sensor, you can create a simple thermometer project that displays the current temperature on an the console.

<div class="row row-cols-4">

{% include card.html cardtitle="Using the Raspberry Pi Pico's Built-in Temperature Sensor" description="Explore the Raspberry Pi Pico's built-in temperature sensor and learn how to use it with MicroPython." link="/learn/pico_temp_sensor/" img="/learn/pico_temp_sensor/assets/cover.jpg"  tl="2" dl="3" %}

</div>

---

## 3. **Weather Robot**

<div class="row row-cols-4">

I designed WeatherBot to be a fun robot that can show a tempareture reading in an innovate and unusual way; it uses a servo to point to a value on a dial, on its stomach. It even holds the temperature sensor in its hand, with the wires going to the back of the robot where the microcontroller is housed.

{% include card.html cardtitle="WeatherBot" description="A cute robot that can show you the temperature" link="/blog/weatherbot" img="/assets/img/blog/weatherbot/weatherbot.png"  tl="2" dl="3" %}

</div>

---

## 4. **BurgerBot**

Learn how to build and program Burgerbot, a robot that can move around and avoid objects using a range finder.

<div class="row row-cols-4 g-3">

{% include card.html cardtitle="BurgerBot Course" description="Learn how to build and program Burgerbot, a robot that can move around and avoid objects using a range finder." link="/learn/burgerbot/" img="/learn/burgerbot/assets/burgerbot.jpg" tl="2" dl="3" %}

{% include card.html cardtitle="BurgerBot v1" description="Build a BurgerBot" link="/blog/burgerbot/" img="/assets/img/blog/burgerbot/burgerbot.jpg" tl="2" dl="3" %}

{% include card.html cardtitle="BurgerBot v2 - Upgrades" description="BurgerBot can draw" link="/blog/burgerbot_v2" img="/assets/img/blog/burgerbot/burgerbot_v2.jpg" tl="2" dl="3" %}

{% include card.html cardtitle="BurgerBot Bluetooth" description="Control BurgerBot using Bluetooth" link="/blog/bluetooth-remote.html" img="/assets/img/blog/ble/ble.jpg" tl="2" dl="3" %}

</div>

---

## 5. LED Displays

The 8x8 LED Matrix I2C display is useful for scrolling text, and displaying fun icons and simple animations.

<div class="row row-cols-4 g-3">

{% include card.html cardtitle="A Cute 8x8 LED Robot" description="Facebot is a cute desktop robot that features a small 8x8 LED Display matrix. It is powered by a Raspberry Pi Pico W, which means we can control this robot using Wifi and/or Bluetooth." link="/blog/facebot" img="/assets/img/blog/facebot/facebot.png" tl="2" dl="3" %}

</div>

---

## 6. Virtual Pet

The Pico is a powerful little board; we can harness that power to create a fun virtual pet that you can interact with.

<div class="row row-cols-4 g-3">

{% include card.html cardtitle="Picotamachibi" description="Create a virtual pet using the Raspberry Pi Pico" link="/blog/picotamachibi" img="/assets/img/blog/picotamachibi/picotamachibi.jpg" tl="2" dl="3" %}

{% include card.html cardtitle="Picotamachibi 2" description="Use a Pico 2 to run the code faster!" link="/blog/picotamachibi2" img="/assets/img/blog/picotamachibi2/picotamachibi2.jpg" tl="2" dl="3" %}

</div>

---

## 7. GhostBox

The GhostBox is a fun project that uses a Raspberry Pi Pico to create a spooky Halloween ghost detector. The project uses a Pico, a Pimoroni display and some fun lights.

<div class="row row-cols-4 g-3">

{% include card.html cardtitle="GhostBox" description="Make your own GhostBox - using a Raspberry Pi Pico" link="/blog/ghostbox" img="/assets/img/blog/ghostbox/cover.jpg" tl="2" dl="3" %}

</div>

---

## 8. Wifi Scanner

The Pico W can be used to scan for Wifi networks and display the results on a small OLED display. We can make this extra fun by turning it into a Ghostbusters inspired handheld device.

<div class="row row-cols-4 g-3">

{% include card.html cardtitle="Ghostbusters Wifi-scanner" description="Who you gonna call?" link="/blog/ghostbusters-wifi" img="/assets/img/blog/ghostbusters/ghost02.jpg" tl="2" dl="3" %}

</div>

---

## 9. Print a stand for your Pico

Sometimes you just want a simple stand to hold your Pico while you work on it. This project shows you how to design and print a simple stand for your Pico.

<div class="row row-cols-4 g-3">

{% include card.html cardtitle="Pi Stands" description="Cute Stands for Raspberry Pis" img="/assets/img/blog/pi_stand/pi_stands.jpg" link="/blog/pistands" tl="2" dl="3" %}

</div>

---

## 10. Web controllable Big Mouth Billy Bass Fish

he Pico W can be used to create a simple web server that can be accessed from a web browser. This is a great way to learn about networking and web development. This project takes it to the next level.

<div class="row row-cols-4 g-3">

{% include card.html cardtitle="Hack a Fish with a Pico W" description="Hack the Big Mouth and make it move using a Pico, and then make this controllable via a webpage." link="/blog/big-mouth" img="/assets/img/bigmouthbillybass/fish01.jpg" tl="2" dl="3" %}

</div>

---

## Thats a wrap!

I hope you enjoyed this collection of Pico Projects. If you'd like more inspiration, check out the idea pages:

<div class="row row-cols-4 g-3">

{% include card.html cardtitle="Pico Ideas - MicroPython" description="More MicroPython inspired links" link="/ideas/micropython" img="/assets/img/groups/micropython.png" tl="3" dl="3" %}

{% include card.html cardtitle="Pico Ideas - Pico Projects" description="Yet more Pico projecs for you to explore" link="/ideas/pico" img="/assets/img/groups/pico.png" tl="3" dl="3" %}

</div>

---
