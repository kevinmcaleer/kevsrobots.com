---
title: "Hacky Temperature and Humidity Sensor"
description: >-
    This is a quick, cheap project to create a temperature and humidity sensor using a DHT22 sensor and a Raspberry Pi Pico
excerpt: >-
    Build a quick outdoor sensor for temperature and humidity using a DHT22 sensor and a Raspberry Pi Pico
layout: showcase
date: 2025-01-04
author: Kevin McAleer
difficulty: beginner
cover: /assets/img/blog/hackysensor/cover.jpg
hero:  /assets/img/blog/hackysensor/hero.png
mode: light
tags:
 - dht22
 - raspberry pi pico
 - pico
 - micropython
groups:
 - pico
 - micropython
 - electronics
# videos:
#  - f4p6wBqGEJI
repo:
 - https://gist.github.com/kevinmcaleer/711ced03eab0cc7b10edd1d492dd1ae9
---

This is a quick project to create a temperature and humidity sensor using a `DHT22` sensor and a Raspberry Pi Pico. The DHT22 sensor is a low-cost sensor that can measure temperature and humidity. The Raspberry Pi Pico is a microcontroller board that is based on the RP2040 microcontroller chip. 

The DHT22 sensor is connected to the Raspberry Pi Pico using a single data wire. The data wire is connected to a GPIO pin on the Raspberry Pi Pico. The Raspberry Pi Pico reads the data from the DHT22 sensor and then sends the data to a computer over a WiFi connection to an MQTT server. This can then be monitored using another Raspberry Pi running Node-Red and data stored in a time serries database such as InfluxDB, and finally graphed in a dashboard using Grafana.

---

> ## Why Hacky?
>
> I call this a ***Hacky*** project simply because the `DHT22` is out of production and is not particularly accurate or reliable, but it's often good enough in a pinch. However, it is a fun project and a good way to learn about microcontrollers and sensors.

---

## Bill of Materials

Item                | Description                     | Quantity | Price | Total
--------------------|---------------------------------|:--------:|------:|-----:
Raspberry Pi Pico W | Microcontroller board           |    1     | £5.80 | £5.80
DHT22 Sensor        | Temperature and humidity sensor |    1     | £3.00 | £3.00
Jumper Wires        | 3 female to female jumper wires |    3     | £0.10 | £0.30
Total               |                                 |          |       | £9.10
{:class="table table-striped"}

---

## Wiring

The DHT22 sensor has 4 pins: `VCC`, `GND`, `DATA`, and `NC`. The `VCC` pin is connected to the `3.3V pin` on the Raspberry Pi Pico, the `GND` pin is connected to the `GND` pin on the Raspberry Pi Pico, and the `DATA` pin is connected to `GPIO pin 15` on the Raspberry Pi Pico. The `NC` (Not Connected) pin is not connected.

---

## Code

The code for this project is written in MicroPython. The code reads the temperature and humidity data from the DHT22 sensor and then sends the data to an MQTT server. The code uses the `umqtt.simple` library to connect to the MQTT server and publish the data.

Copy the `main.py` to the pico along with `simple.py`. You'll also need to create a `wifi_config.py` file with your WiFi credentials, and upload that to the pico too.

<script src="https://gist.github.com/kevinmcaleer/711ced03eab0cc7b10edd1d492dd1ae9.js"></script>

---

## Weatherproofing

You use this sensor outdoors, add I used a weatherproof box to house the sensor. I cut a hole in the box for the USB cable to exit, and this goes through a hole in front of the robotlab. I added some hotglue to prevent water ingress.

{% include gallery.html images='/assets/img/blog/hackysensor/box.jpg, /assets/img/blog/hackysensor/box2.jpg' titles='Weatherproof box, Closer view' %}

---

## Presenting the data

I used [Node-Red](https://www.nodered.org) to collect the data from the [MQTT server](/resources/how_it_works/mqtt) and store it in [InfluxDB](https://www.influxdata.com). I then used Grafana to create a dashboard to display the data.

I know this last bit is a bit light on detail - I'll be sure to add more detail in a future post, along with courses on Node-Red, InfluxDB, and Grafana.

{% include gallery.html images='/assets/img/blog/hackysensor/nodered.png, /assets/img/blog/hackysensor/grafana.png' titles='Node-RED, Grafana Dashboard' %}

---
