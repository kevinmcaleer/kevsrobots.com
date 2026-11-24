---
layout: lesson
title: Using Potentiometers
type: page
description: Learn how to use potentiometers with the Raspberry Pi Pico and MicroPython.
cover: /learn/micropython_gpio/assets/banners/09_potentiometers.jpg
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to wire a potentiometer to an ADC pin and use its position as an input.

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

---

## Wiring table

| Potentiometer pin | Connects to |
| --- | --- |
| Outer pin 1 | 3V3(OUT), physical pin 36 |
| Middle pin (wiper) | GP26, physical pin 31 |
| Outer pin 2 | AGND, physical pin 33 |
{:class="table table-single"}

---

## Try it Yourself

1. Run the program and turn the knob. Open View > Plotter in Thonny to see it as a graph.
2. Swap the two outer pins. What changes?
3. Challenge: use the reading to set an LED's brightness by passing `pot.read_u16()` straight to `led.duty_u16()` on a PWM pin.

## Common Issues

- **Problem**: The reading jumps between 0 and 65535.
- **Solution**: Check the wiper (middle pin) is on GP26, not an outer pin.
- **Why**: The outer pins are fixed at 3V3 and GND, so reading one gives a steady extreme.

- **Problem**: The reading never reaches the full range.
- **Solution**: Check the outer pins really go to 3V3(OUT) and GND.
- **Why**: The wiper can only reach the voltages on the outer pins.

## Check your understanding

{% quiz %}
? Which potentiometer pin goes to the ADC input?
- [ ] Either outer pin
- [x] The middle pin (the wiper)
- [ ] None, it connects to 3V3
> The wiper's voltage changes as you turn the knob. The outer pins go to 3V3 and GND.

? What is the potentiometer's middle pin called?
- [ ] The anode
- [x] The wiper
- [ ] The rail
> The wiper slides along the resistor and gives the variable voltage.

? Swapping the two outer pins does what?
- [ ] Breaks the pot
- [x] Reverses the direction the reading changes
- [ ] Nothing
> The wiper now rises as the knob turns the other way.
{% endquiz %}

---
