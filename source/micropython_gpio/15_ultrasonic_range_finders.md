---
layout: lesson
title: Ultrasonic Range Finders
type: page
description: Learn how ultrasonic range finders work and how to use them with the Raspberry Pi Pico and MicroPython.
---

## Overview

Welcome to Lesson 15 of the `Raspberry Pi Pico with MicroPython - GPIO Mastery` course. In this lesson, you will learn:

* What ultrasonic range finders are and how they work
* How to connect an ultrasonic range finder to the Raspberry Pi Pico
* How to write MicroPython code to read distance measurements from the ultrasonic range finder

---

## What are Ultrasonic Range Finders?

[Ultrasonic range finders](/resources/how_it_works/ultrasonic) are sensors that measure distance by emitting high-frequency sound waves and measuring the time it takes for the sound waves to bounce back from an object. They are commonly used in robotics and automation applications to detect the presence of objects and determine their distance.

---

## Connecting an Ultrasonic Range Finder to the Raspberry Pi Pico

To connect an ultrasonic range finder to the Raspberry Pi Pico, you will need to connect four pins:

* `VCC` - Connect this to a `5V` pin on the Raspberry Pi Pico
* `GND` - Connect this to a `GND` pin on the Raspberry Pi Pico
* `Trig` - Connect this to a `GPIO` pin on the Raspberry Pi Pico
* `Echo` - Connect this to another `GPIO` pin on the Raspberry Pi Pico

---

## Reading Distance Measurements with MicroPython

To read distance measurements from the ultrasonic range finder using MicroPython, you will need to:

1. Set the trigger pin as an output and the echo pin as an input
2. Send a high signal to the trigger pin for a brief amount of time
3. Measure the amount of time it takes for the echo pin to receive a high signal
4. Calculate the distance based on the time measurement

Here is an example MicroPython code that reads distance measurements from an ultrasonic range finder:

```python
import machine
import time

trig_pin = machine.Pin(2, machine.Pin.OUT)
echo_pin = machine.Pin(3, machine.Pin.IN)

while True:
    # Send a 10 microsecond pulse to the trigger pin
    trig_pin.value(0)
    time.sleep_us(2)
    trig_pin.value(1)
    time.sleep_us(10)
    trig_pin.value(0)

    # Measure the duration of the echo pulse
    duration = machine.time_pulse_us(echo_pin, 1)

    # Calculate the distance in centimeters
    distance = duration / 58

    print("Distance: {} cm".format(distance))
    time.sleep(1)
```

This code continuously reads distance measurements from the ultrasonic range finder and prints the results to the console.

---

## Summary

In this lesson, you learned what ultrasonic range finders are, how to connect them to the Raspberry Pi Pico, and how to use MicroPython to read distance measurements from them.

---
