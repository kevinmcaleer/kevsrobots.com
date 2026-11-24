---
layout: lesson
title: Ultrasonic Range Finders
type: page
description: Learn how ultrasonic range finders work and how to use them with the Raspberry Pi Pico and MicroPython.
cover: /learn/micropython_gpio/assets/banners/15_ultrasonic_range_finders.jpg
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to measure distance with an HC-SR04 and wire it to the Pico without damaging the input.

## Overview

In this lesson, you will learn:

* What ultrasonic range finders are and how they work
* How to connect an HC-SR04 range finder to the Pico safely
* How to write MicroPython code to measure distance

---

## What are ultrasonic range finders?

[Ultrasonic range finders](/resources/how_it_works/ultrasonic.html) measure distance by sending out a burst of high-frequency sound and timing how long the echo takes to come back. Robots use them to spot obstacles.

Sound travels at about 343 metres per second, or 0.0343cm per microsecond. The sound goes there *and* back, so:

```
distance (cm) = echo time (µs) x 0.0343 / 2   ≈   echo time / 58
```

---

## Connecting an HC-SR04 to the Pico

* `VCC` to `VBUS` (5V)
* `GND` to `GND`
* `Trig` to `GP2`
* `Echo` to `GP3` **through a voltage divider** (1kΩ from Echo to GP3, then 2kΩ from GP3 to GND)

> ## Protect the Pico
>
> The standard HC-SR04 runs on 5V and its Echo pin outputs 5V, which is too much for a Pico input (see lesson 3). Use the divider, or buy the HC-SR04P, which works at 3.3V and can be powered from `3V3`.

---

## Reading distance measurements with MicroPython

1. Send a 10µs high pulse on `Trig`
2. Measure how long `Echo` stays high
3. Turn that time into a distance

```python
from machine import Pin, time_pulse_us
from time import sleep, sleep_us

trig = Pin(2, Pin.OUT)
echo = Pin(3, Pin.IN)

def distance_cm():
    trig.value(0)
    sleep_us(2)
    trig.value(1)
    sleep_us(10)
    trig.value(0)
    duration = time_pulse_us(echo, 1, 30000)   # give up after 30ms
    if duration < 0:
        return None                            # no echo: nothing in range
    return duration / 58

while True:
    d = distance_cm()
    print("Out of range" if d is None else f"Distance: {d:.1f} cm")
    sleep(0.5)
```

`time_pulse_us()` returns a negative number if no echo arrives before the timeout, so we check for that rather than printing a nonsense distance.

---

---

## Wiring table

| HC-SR04 pin | Connects to |
| --- | --- |
| VCC | VBUS (5V), physical pin 40 |
| GND | GND |
| Trig | GP2 |
| Echo | 1kΩ to GP3, with 2kΩ from GP3 to GND |
{:class="table table-single"}

---

## Try it Yourself

1. Run the program and point the sensor at a wall. Move it nearer and further away.
2. Check the reading against a ruler. How close is it?
3. Challenge: light an LED when something is closer than 20cm.

## Common Issues

- **Problem**: It always prints "Out of range".
- **Solution**: Check `Trig` and `Echo` are on the right pins and VCC has 5V.
- **Why**: No echo means the pulse never arrived before the timeout.

- **Problem**: Readings jump around.
- **Solution**: Point at a flat surface, and average a few readings.
- **Why**: Soft or angled surfaces scatter the sound.

> **Teachers**: The HC-SR04 needs 5V and a divider, so check each group's circuit before powering up. Pre-build the divider on a small piece of breadboard if time is short.

## Check your understanding

{% quiz %}
? Why do we divide by 58 (rather than 29) to get centimetres?
- [ ] Sound is slower in air than we think
- [x] The sound travels to the object and back again
- [ ] The sensor counts in half-microseconds
> The echo time covers the round trip, so the distance is half of what the sound travelled.

? Why does a standard HC-SR04's Echo pin need a voltage divider?
- [x] It outputs 5V, and the Pico's inputs are 3.3V only
- [ ] To make the echo louder
- [ ] To slow the pulse down
> The divider brings the 5V echo signal down to a safe level for the Pico.

? How long does the Trig pulse need to be?
- [ ] 1 second
- [x] About 10 microseconds
- [ ] 30 milliseconds
> The program sends a 10 microsecond high pulse on Trig.
{% endquiz %}

---
