---
title: "Weather Station Display"
description: "Build a Weather Station Display with Pimoroni Presto"
excerpt: >-
   
layout: showcase
date: 2025-02-22
author: Kevin McAleer
difficulty: beginner
cover: /assets/img/blog/presto/cover.jpg
hero:  /assets/img/blog/presto//hero.png
mode: light
videos:
  - 9F3R4p2UXoo
tags:
 - Pico
 - Weather Station
 - Pimoroni
 - MicroPython
groups:
 - raspberrypi
 - python
 - micropython
 - electronics
code:
 - https://www.github.com/kevinmcaleer/weather-station

---

## Pimoroni Presto

Pimoroni Presto is a 4" Touch-screen display built around the RP2350. Its housed in a sleek black aluminum stand with a 4" square (480 x 480 pixel) IPS LCD screen with capacitive touch overlay. The display is powered by the RP2350B (Dual Arm Cortex M33 running at up to 150MHz with 520KB of SRAM) and has 16MB of QSPI flash supporting XiP.

---

## Features

- Powered by RP2350B (Dual Arm Cortex M33 running at up to 150MHz with 520KB of SRAM)
- 16MB of QSPI flash supporting XiP
- 8MB of PSRAM
- Raspberry Pi RM2 module (CYW43439), supporting IEEE 802.11 b/g/n wireless LAN, and Bluetooth
- 4" square (480 x 480 pixel) IPS LCD screen with capacitive touch overlay
- Rainbow backlighting, courtesy of 7x mini SK6812 (Neopixel-compatible) RGB LEDs
- USB-C connector for programming and power
- Piezo speaker
- Reset and boot buttons (the boot button can also be used as a user button)
- MicroSD card slot
- Qw/ST (Qwiic/STEMMA QT) connector
- 2-pin JST-PH connector for adding a battery (3V - 5.5V)
- Pre-installed in a black anodised aluminium stand with rubber feet
- Comes fully-assembled (no soldering required)
- Programmable with C/C++ or MicroPython

---

## Weather Station Project

In this project, we will build a weather station display using the Pimoroni Presto. The weather station will display the current temperature, humidity, and pressure. The data will be collected from various sensors from the existing weather station project. 

The sensor data from the weather station is pushed to an InfluxDB database. We will create a python script to query the database and send that simplified data to an MQTT topic so we can easily subscribe to it from the Presto.

---

## Displaying the data with PiChart

I previously created a python library called [PiChart](https://www.github.com/kevinmcaleer/pichart) that allows you to create simple charts on displays that use the Pimoroni PicoGraphics-based displays. The library provides simple, but powerful charting components such as bar chars, line charts and simple information cards. It will arrange these charts in a grid layout on the display.

---

## The Python script

My InfluxDB has plenty of data in it, including:

- Temperature (from inside and outside sensors)
- Humidity (from inside and outside sensors)
- Pressure (from inside and outside sensors)
- Light levels
- Wind speed
- Wind direction
- Rainfall
- Gas levels
- Particle levels

For this project I just need the last 24 hours of data for temperature, humidity and pressure. We don't need all the data, just one reading per hour to show how its changing throgout the day. The charts on the display are quite small, so we don't need a lot of data to show the trends.

Python can use a statistical modelling library to forecast the weather based on current and historical barometric pressure data.
We can publish the results of the forecast as a text string to an MQTT topic, which the Presto can subscribe to and display.

As we just want to display the pressure data trend, we don't need to include timestamp information in the MQTT payload; the order of the 24 readings will be enough to show the trend; PiChart will be able to display this data in a line chart.

---

## PiChart

I created [PiChart](https://www.github.com/kevinmcaleer/pichart) to make it easy to create simple charts on displays that use the Pimoroni PicoGraphics-based displays. The library provides simple, but powerful charting components such as bar chars, line charts and simple information cards. It will arrange these charts in a grid layout on the display.

PiChart will grab the minimum and maximum values from the data and scale the chart accordingly. It will also add a label to the top of the chart with the title of the chart and the current value. You can turn off the labels, Axes and gridlines if you want to create a more minimalistic chart.

---

Grab the code and try it out for yourself:

- [Presto Weather Station Display](https://github.com/kevinmcaleer/weather_forecast)
- [PiChart](https://www.github.com/kevinmcaleer/pichart)

---
