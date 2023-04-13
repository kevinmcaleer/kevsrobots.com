---
layout: lesson
title: Using Potentiometers
type: page
description: Learn how to use potentiometers with the Raspberry Pi Pico and MicroPython.
---

<!-- ![Cover photo of a potentiometer connected to a Raspberry Pi Pico board](assets/raspberry_pi_pico_potentiometer.jpg){:class="cover"} -->

## Overview

Welcome to Lesson 9 of the `Raspberry Pi Pico with MicroPython - GPIO Mastery` course. In this lesson, we will learn how to use potentiometers with the Raspberry Pi Pico and MicroPython.

## Course Content

In this lesson, you will learn:

* What potentiometers are and how they work
* How to connect a potentiometer to the Raspberry Pi Pico
* How to write MicroPython code to read potentiometer values

---

## What are Potentiometers?

`Potentiometers` are three-terminal resistors with a sliding contact that forms a variable voltage divider. By adjusting the position of the sliding contact, the output voltage can be varied between the input voltage and ground. You can learn more about [how potentiometers work here](/resources/how_it_works/pots).

---

## Connecting a Potentiometer to Raspberry Pi Pico

To connect a potentiometer to your Raspberry Pi Pico board, follow these steps:

1. Connect the potentiometer's outer pins to the 3.3V and GND pins on the Raspberry Pi Pico board.
2. Connect the potentiometer's center pin to a GPIO pin on the Raspberry Pi Pico board.

---

## Reading Potentiometer Values with MicroPython

To read potentiometer values using MicroPython, you can use the `machine.ADC` module. Here's some example code to get you started:

```python
from machine import Pin, ADC
import time

potentiometer_pin = Pin(26, Pin.IN)
adc = ADC(potentiometer_pin)
adc.atten(ADC.ATTN_11DB)

while True:
    potentiometer_value = adc.read()
    print(potentiometer_value)
    time.sleep(0.1)
```

---

## Conclusion

In this lesson, you learned how to use potentiometers to control the input voltage to the Raspberry Pi Pico board, and how to read potentiometer values using MicroPython. You can use this knowledge to create a variety of projects with your Raspberry Pi Pico board.

---

## Key Results

After you have completed this lesson, you will know how to use potentiometers with the Raspberry Pi Pico and MicroPython. You will be able to connect a potentiometer to the Pico, write MicroPython code to read potentiometer values, and understand the basics of potentiometer operation.

---
