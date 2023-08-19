---
layout: how_it_works
title: MQTT
short_title: How it works - MQTT
short_description: Learn about MQTT
date: 2023-01-12
author: Kevin McAleer
excerpt:
cover: /assets/img/how_it_works/mqtt01.jpg
tags:
 - Robot
 - Tips
 - MQTT
---

`MQTT` (MQ Telemetry Transport, not Message Queuing Telemetry Transport as its often mistakenly called) is a lightweight messaging protocol designed by IBM for low-bandwidth, high-latency, unreliable networks.

It works by establishing a connection between two or more devices, and then sending messages between them.

The messages are organized into `topics`, and each device is assigned an ID that is used to identify it when sending and receiving messages.
The protocol is designed to be lightweight, efficient and secure.
It is used in many Internet of Things (IoT) applications, such as controlling home automation systems and monitoring sensors.
The MQTT protocol is based on a `publish-subscribe` model, where one device (the `publisher`) sends messages to other devices (`subscribers`) that are subscribed to the same topic.
This allows for quick, efficient communication between devices, and allows for multiple devices to be involved in the same conversation.
Thanks to its efficient design, MQTT is ideal for applications with limited bandwidth or high latency.

---

[![Slides from video](/assets/img/how_it_works/mqtt02.jpg){:class="img-fluid w-100"}](/assets/img/how_it_works/mqtt02.jpg)

---

[![Slides from video](/assets/img/how_it_works/mqtt03.jpg){:class="img-fluid w-100"}](/assets/img/how_it_works/mqtt03.jpg)

---

[![Slides from video](/assets/img/how_it_works/mqtt04.jpg){:class="img-fluid w-100"}](/assets/img/how_it_works/mqtt04.jpg)

---

[![Slides from video](/assets/img/how_it_works/mqtt05.jpg){:class="img-fluid w-100"}](/assets/img/how_it_works/mqtt05.jpg)

---

[![Slides from video](/assets/img/how_it_works/mqtt06.jpg){:class="img-fluid w-100"}](/assets/img/how_it_works/mqtt06.jpg)

---

## Video

Here is a video about MQTT, and how to use it with Node-Red.

{% include youtubeplayer.html id="4UIUC1YU_Sk" %}

---
