---
layout: blog
title: Ultrasonic sensors
short_title: How it works - Ultrasonic sensors
short_description: Learn about Ultrasonic sensors
date: 2023-01-10
author: Kevin McAleer
excerpt: 
cover: /assets/img/how_it_works/ultrasonic02.png
tags:
 - Robot
 - Tips
---

`Ultrasonic` sensors use high-frequency sound waves to measure the distance between the sensor and an object.

When the sensor emits a sound wave, it will bounce off the object it hits and return back to the sensor. The sensor measures the time it takes for the sound wave to travel in order to calculate the distance between the object and the sensor.

Ultrasonic sensors are commonly used for industrial automation and robotics, as well as for safety and security applications.

The most commonly used Ultrasonic sensor, also known as a range finder, is the HC-SR04.

---

![Picture of an Ultrasonic sensor](/assets/img/how_it_works/ultrasonic01.jpg){:class="img-fluid w-100"}

---

## MicroPython Code

A typical piece of code for measuring distance will look like this:

```python

from machine import Pin, PWM
import utime

trigger = Pin(14, Pin.OUT)
echo = Pin(15, Pin.IN)

buzzer = PWM(Pin(16))
buzzer.freq(1000)


def measure_distance():
    """ Returns the distance in mm"""
    trigger.low()
    utime.sleep_us(2)
    trigger.high()
    utime.sleep_us(5)
    trigger.low()
    
    while echo.value() == 0:
        signaloff = utime.ticks_us()
    
    while echo.value() == 1:
        signalon = utime.ticks_us()
    
    timepassed = signalon - signaloff
    distance = (timepassed * 0.0343) / 2
    return distance

 
while True:
    distance = measure_distance()
    print(round(distance))
    sleep(1)
```
