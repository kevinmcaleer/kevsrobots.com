---
layout: code
title: Ping
description: Detect distance using a Range finder
level: 1
cover: /assets/img/code/ping.png
languages: 
  - python
  - micropython
gist: https://gist.github.com/kevinmcaleer/47ae0720ecad7a12e9db33a094f37ec3

---

## Usage

Here is an example program that uses the `Range_Finder` class. In this example GPIO pin `1` is connected to the `Echo` Pin on the range finder, and GPIO pin `2` is connected to the `Trigger` Pin.

```python
sensor = Range_Finder(echo_pin=1, trigger_pin=2)

distance = sensor.ping()
```

---
