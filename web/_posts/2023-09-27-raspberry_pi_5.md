---
layout: blog
title: Raspberry Pi 5
description: Learn about the new Raspberry Pi 5
short_title: Raspberry Pi 5
short_description: Learn about the new Raspberry Pi 5
date: 2023-09-27
author: Kevin McAleer
excerpt: Learn all about the new features and changes 
cover: /assets/img/blog/raspberrypi5/raspberrypi5.jpg
tags: 
 - Raspberry Pi
---

It's finally [here](https://www.raspberrypi.com/products/raspberry-pi-5/) and it's packed full of new features. The Raspberry Pi 5 is the next generation of the Raspberry Pi Single Board Computer, and will be available to buy in October 2023 (about 4 weeks from launch), along with the new 5V5A Power Supply, Case, and active cooling Fan. The Raspberry Pi 5 will also be ***only*** be available to Makers until the New Year once sufficient supplies have become available, when it will be available to Industrial users, too.

Let's take a deep dive and learn about what has changed, why you'll want one, and when it's actually available.

---

## Videos

{% include youtubeplayer.html id="O8RxVpMxrz0" %}
{% include youtubeplayer.html id="vXYzJ1os4NA" %}
{% include youtubeplayer.html id="ysNTEy1lUKo" %}
{% include youtubeplayer.html id="NiDBiUGvZus" %}

---

{:toc}
* toc

---

![Whats new](/assets/img/blog/raspberrypi5/pi5_001.jpg){:class="img-fluid w-75"}

## What’s new

Here is an overview of the new features:

* All new Raspberry Pi Silicon RP1 chip
* Dual Camera / Display connectors
* PCIe connector
* Improved GPU
* Power Button
* RTC with External Battery connector

---

![Whats new](/assets/img/blog/raspberrypi5/pi5_002.jpg){:class="img-fluid w-75"}

* RP1 “southbridge”, The first full-size Raspberry Pi computer using in-house built silicon
* Featuring a 64-bit quad-core Arm Cortex-A76 processor running at 2.4GHz
* Delivers a 2–3× increase in CPU performance relative to Raspberry Pi 4
* Graphics performance from an 800MHz VideoCore VII GPU
* Smooth desktop experience with new RPi Image Signal Processor
* Faster transfer speeds to external USB drives with > double aggregate USB bandwidth
* A pair of four-lane 1.5Gbps MIPI camera and display interfaces present
* Peak SD card performance is doubled via SDR104 high-speed mode
* PCI Express 2.0 interface, providing support for high-bandwidth peripherals

![Whats new](/assets/img/blog/raspberrypi5/southbridge.png){:class="img-fluid w-75"}

A `Southbridge` is a secondary processor, designed to handle slower operations such as Input/Output, USB and Camera & Display operations. The `Northbridge` handles the fastest operations such as CPU, GPU and data transfers from main memory.

---

## Processor Speed

![Whats new](/assets/img/blog/raspberrypi5/pi5_003.jpg){:class="img-fluid w-75"}

The new processor is 150x more powerful than the original Pi, and is on par with the processing capabilities of the 2015 Apple MacBook Air.

---

## New Dual Display & Camera connectors

![Whats new](/assets/img/blog/raspberrypi5/pi5_004.jpg){:class="img-fluid w-75"}

The display connector has been moved next to the camera connector and can now be used with 2 cameras, 1 camera and 1 display or 2 displays.

**Note** that the ribbon cable is the smaller size, used on the Raspberry Pi Zero & Zero 2W.

---

## Faster 4K performance

![Whats new](/assets/img/blog/raspberrypi5/pi5_005.jpg){:class="img-fluid w-75"}

The HDMI ports can now display 4k at 60 frames per second (4kp60), and unlike the Raspberry Pi 4, can comfortably handle full screen video at 4k.

---

## Power to the Ethernet

![Whats new](/assets/img/blog/raspberrypi5/pi5_006.jpg){:class="img-fluid w-75"}

The Power over Ethernet (PoE) has been upgraded to PoE+ (802.3at), meaning it can deliver up to 30W of power with 25W guaranteed to be available at the powered device.

A separate PoE+ HAT is required to use this feature, available shortly after launch.

---

## USB 3, full speed ahead

USB speed has been improved, and each USB 3 port can now support 5gb of operation simultaneously. The USB 2 ports also have dedicated controllers, increasing the maximum available data transfer rates.

![Whats new](/assets/img/blog/raspberrypi5/pi5_007.jpg){:class="img-fluid w-75"}

---

## Vidcore VII

---

![Whats new](/assets/img/blog/raspberrypi5/pi5_008.jpg){:class="img-fluid w-75"}

The GPU has been improved and now runs at an impressive 800MHz, now supporting OpenGL ES 3.1 and Vulkan 1.2.

---

## Power Management

![Whats new](/assets/img/blog/raspberrypi5/pi5_009.jpg){:class="img-fluid w-75"}

The Raspberry Pi 5 now has a new power management chip that can intelligently manage the amount of power drawn from the power supply. It also talks to the power button to respond to signals (on/off/soft & hard shutdown)

* 5V/5A DC power via USB-C
* Power Delivery support: a technology that allows for higher power levels, enabling devices to charge quickly and even deliver power to larger devices like laptops.
* USB Power Delivery is typically associated with USB Type-C connectors. It works by negotiating power requirements between the charger and the device, allowing for dynamic power delivery based on the device's needs.
* Power Delivery is not just about speed but also safety and efficiency, as it ensures that devices receive the appropriate amount of power without overloading or damaging them.

---

![Whats new](/assets/img/blog/raspberrypi5/pi5_010.jpg){:class="img-fluid w-75"}

## Dimensions

The mounting holes and footprint of the 5 are identical to the 4, apart from the USB and Ethernet ports that have moved back to the previous orientation with the Ethernet now back to the bottom of the board.

There are an additional 2 mounting holes on the board for the Active Cooling fan, which is now available from Raspberry Pi, which both monitors the temperature of the CPU and intelligently adjusts the speed of the fan to cool.

---

![Whats new](/assets/img/blog/raspberrypi5/pi5_011.jpg){:class="img-fluid w-75"}

The Pi 5 also has a new power button, and a real-time clock with an external battery connector.

## Evolution of the Pi

![Whats new](/assets/img/blog/raspberrypi5/pi5_012.jpg){:class="img-fluid w-75"}

---

## Raspberry Pi OS - Bookworm

The Pi 5 also comes with a new OS release - Bookworm (the latest Debian release). It includes a new User Interface layer called `Wayfire` which is based on the Wayland protocol. This results in a 2x faster interface, which feels more responsive, has nicer launch and close animations and subtle shadows. It's very much a premium Desktop OS feel.

Note as Wayfire is a signifiant departure from the X Windows based Mutter UI, do expect some teething problems with apps not written to take advantage of the new UI.

---

## Summary - why you'll want one

In a nutshell the new Pi is faster in every way, more capable with new PCIe port, dual camera connectors and the battery connector and realtime clock mean your Pi can remember the time without needing to connect to the Internet.

---
