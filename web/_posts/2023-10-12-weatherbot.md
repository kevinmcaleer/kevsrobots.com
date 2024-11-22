---
title: WeatherBot
description: A cute robot that can show you the temperature
layout: showcase
mode: light
cover: /assets/img/blog/weatherbot/weatherbot.png
hero: /assets/img/blog/weatherbot/hero.png
short_title: WeatherBot
short_description: A cute robot that can show you the temperature
date: 2023-10-12
date_updated: 2024-11-22
author: Kevin McAleer
excerpt: Learn how to build a cute robot that can show you the temperature with a servo
tags:
 - weather
 - esp8266
 - micropython
groups:
 - robots
 - garden
 - 3dprinting
videos:
 - lynvr7M1F6g
code:
 - https://www.github.com/kevinmcaleer/nodemcu_wifi
stl:
    - name: body
      link: /assets/stl/weatherbot/body.stl
    - name: head
      link: /assets/stl/weatherbot/head.stl
---

## Overview

![assembly instructions](/assets/img/blog/weatherbot/weatherbot.001.jpeg){:class="img-fluid w-50"}

I designed `WeatherBot` to be a fun robot that can show a tempareture reading in an innovate and unusual way; it uses a servo to point to a value on a dial, on its stomach. It even holds the temperature sensor in its hand, with the wires going to the back of the robot where the microcontroller is housed.

## Works with Pico Too!

Though this project was orignally designed for use with an ESP8266, it works just as well with a Raspbery Pi Pico W.

---

## Bill of Materials

Weatherbot is quite a simple robot and only needs a couple of parts:

Item                | Description                    | Qty | Unit Price |  Total
--------------------|--------------------------------|:---:|-----------:|------:
Temperature Sensors | DHT22                          |  1  |      £3.00 |  £3.00
Servo               | SG90                           |  1  |      £4.00 |  £4.00
MicroController     | ESP8266 or Raspberry Pi Pico W |  1  |      £4.00 |  £4.00
                    |                                |     |      Total | £11.00
{:class="table table-striped"}

---

## 3D Printable STL files

WeatherBot is made up of two main pieces:

* [`body.stl`](/assets/stl/weatherbot/body.stl) - the main body
* [`head.stl`](/assets/stl/weatherbot/head.stl) - the Robots head

Copy both the `umqttsimple.py` and `weatherbot.py` to the Pico or ESP8266 microcontroller using [Thonny](https://thonny.org). You can configure weatherbot to send its readings to an MQTT server by adding a wifi usernane and password, and MQTT server to the varibles:

```python
ssid = 'insert your SSID here'
password = 'your password'
mqtt_server = 'your MQTT IP'
```

---

## Assembly

![assembly instructions](/assets/img/blog/weatherbot/weatherbot.002.jpeg){:class="img-fluid w-50"}

![assembly instructions](/assets/img/blog/weatherbot/weatherbot.003.jpeg){:class="img-fluid w-50"}

![assembly instructions](/assets/img/blog/weatherbot/weatherbot.004.jpeg){:class="img-fluid w-50"}

![assembly instructions](/assets/img/blog/weatherbot/weatherbot.005.jpeg){:class="img-fluid w-50"}

![assembly instructions](/assets/img/blog/weatherbot/weatherbot.006.jpeg){:class="img-fluid w-50"}

![assembly instructions](/assets/img/blog/weatherbot/weatherbot.007.jpeg){:class="img-fluid w-50"}

![assembly instructions](/assets/img/blog/weatherbot/weatherbot.008.jpeg){:class="img-fluid w-50"}

![assembly instructions](/assets/img/blog/weatherbot/weatherbot.009.jpeg){:class="img-fluid w-50"}

![assembly instructions](/assets/img/blog/weatherbot/weatherbot.010.jpeg){:class="img-fluid w-50"}

![assembly instructions](/assets/img/blog/weatherbot/weatherbot.011.jpeg){:class="img-fluid w-50"}

![assembly instructions](/assets/img/blog/weatherbot/weatherbot.012.jpeg){:class="img-fluid w-50"}

![assembly instructions](/assets/img/blog/weatherbot/weatherbot.013.jpeg){:class="img-fluid w-50"}

![assembly instructions](/assets/img/blog/weatherbot/weatherbot.014.jpeg){:class="img-fluid w-50"}

The head piece should simply slot into the body section.

I also created a small sticker for the dial, and an eye sticker too:

* [`weatherbot_stickers.pdf`](/assets/pdf/weatherbot_stickers.pdf) - the eye and dial stickers

You can print out the stickers, cut them out with scissors and then glue them on the the 3d printed robot using a pva gluestick.

---
