---
layout: content
title: QR Code Robots
description: Build Robots that are controlled by QR Codes
subtitle: Raspberry Pis, Microbits, Arduino and more
excerpt: Learn about the differnt types of boards and the features they provide
---

{% include nav_resources.html %}

{% include breadcrumbs.html %}

![tiny code reader](/assets/img/qr/cover.jpg){:class="cover"}

# {{page.title}}

## {{page.description}}

---

Build a QR Code controlled robot using the [Tiny Code Reader](https://www.adafruit.com/product/5744) sensor for [Useful Sensors Inc](https://usefulsensors.com).

---

I purchased this tiny QR code reader from Adafruit and it can read QR codes and send data to a microcontroller via I2C. It has a built in image sensor and is powered by a Raspberry Pi RP2040 and is made by Useful Sensors Inc. It's only costs a couple of bucks so it's a great way to add simple data entry to your projects.

The QR in QR codes means quick response and it was originally developed by a Japanese company tracking automotive parts.

---

## Gallery

[![tiny code reader](/assets/img/qr/cover.jpg){:class="img-fluid w-25 shadow-lg rounded-3"}](/assets/img/qr/cover.jpg)

[![RP2040 - reverse side](/assets/img/qr/rp2040.jpg){:class="img-fluid w-25 shadow-lg rounded-3"}](/assets/img/qr/rp2040.jpg)

---

## QR Control cards for use with a robot

![QR Control Cards](/assets/img/qr/cards.png){:class="img-fluid w-50 shadow-lg rounded-3"}

I've created a bunch of QR code control cards that you can print out and use to control a robot using the micropython code below. You will need to add your own code to control the robots motors etc.

[Download the QR Control cards here](/assets/pdf/qr_control_cards.pdf)

---

## MicroPython code

<script src="https://gist.github.com/kevinmcaleer/1e5c5bc882cf2af6888832578b108c59.js"></script>

---
