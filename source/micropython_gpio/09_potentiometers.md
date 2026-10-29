---
layout: lesson
title: Using Potentiometers
type: page
description: Learn how to use potentiometers with the Raspberry Pi Pico and MicroPython.
---

## Overview

A potentiometer (or "pot") is a knob that gives you a voltage you can change by hand. It's the perfect first analog input. In this lesson you'll wire one up and read it.

---

## What are potentiometers?

`Potentiometers` are three-terminal resistors with a sliding contact that forms a variable voltage divider. Turn the knob and the middle pin's voltage moves smoothly between the voltages on the two outer pins. You can learn more about [how potentiometers work here](/resources/how_it_works/pots.html).

---

## Connecting a potentiometer to the Pico

1. Connect one outer pin to `3V3(OUT)` (physical pin 36)
2. Connect the other outer pin to `GND` (or `AGND`, physical pin 33, for the cleanest readings)
3. Connect the middle pin (the wiper) to `GP26` (physical pin 31)

A 10kΩ potentiometer is a good choice.

---

## Reading potentiometer values with MicroPython

```python
from machine import ADC
from time import sleep

pot = ADC(26)

while True:
    value = pot.read_u16()
    percent = value * 100 // 65535
    print(value, f"{percent}%")
    sleep(0.1)
```

Turn the knob and watch the numbers change. Thonny's **View > Plotter** draws them as a graph.

> ## ESP32 code won't work here
>
> Examples for the ESP32 use `adc.atten()` and `adc.read()`. The Pico doesn't have these: use `read_u16()`.

---

## Try it yourself

Combine this with lesson 6: use the potentiometer to set the LED brightness by passing `pot.read_u16()` straight to `led.duty_u16()`.

{% quiz %}
? Which potentiometer pin goes to the ADC input?
- [ ] Either outer pin
- [x] The middle pin (the wiper)
- [ ] None, it connects to 3V3
> The wiper's voltage changes as you turn the knob. The outer pins go to 3V3 and GND.
{% endquiz %}

---
