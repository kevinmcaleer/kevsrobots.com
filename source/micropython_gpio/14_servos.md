---
layout: lesson
title: Introduction to Servos
type: page
description: Learn about servos and how to control them with the Raspberry Pi Pico and MicroPython.
cover: /learn/micropython_gpio/assets/banners/14_servos.jpg
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to turn a servo to a chosen angle using a 50Hz PWM signal.

## Overview

A servo is a motor that turns to a position you choose and holds it there, which makes it perfect for robot arms, legs and pan-tilt heads. In this lesson you'll wire one up and control it with PWM.

---

## What is a servo?

A [servo](/resources/how_it_works/servos.html) has three wires:

* **Power** (usually red) - 5V for hobby servos like the SG90
* **Ground** (brown or black)
* **Signal** (orange or yellow) - a PWM signal that sets the position

The servo reads the **pulse width**: a pulse is sent 50 times a second, and its length sets the angle. For an SG90, about 0.5ms is 0°, 1.5ms is 90° and 2.5ms is 180°.

---

## Connecting a servo to the Pico

* Power to `VBUS` (5V from USB, physical pin 40)
* Ground to `GND`
* Signal to `GP18` (physical pin 24). Any GPIO pin will work

One small servo is fine from USB power. For more than one, or bigger servos, use a separate 5V supply and connect its ground to the Pico's ground.

---

## Controlling a servo with MicroPython

At 50Hz each cycle lasts 20ms, and `duty_u16()` covers that whole 20ms with 0 to 65535. So 1ms is about `65535 / 20 = 3277`.

```python
from machine import Pin, PWM
from time import sleep

servo = PWM(Pin(18))
servo.freq(50)

def set_angle(angle):
    pulse_ms = 0.5 + (angle / 180) * 2.0          # 0.5ms to 2.5ms
    servo.duty_u16(int(pulse_ms * 65535 / 20))

for angle in (0, 90, 180, 90):
    set_angle(angle)
    sleep(1)

servo.deinit()
```

- `servo.freq(50)` sends 50 pulses per second, which is what servos expect
- `set_angle()` turns an angle into a pulse width, then into a duty value
- `servo.deinit()` stops the signal, so the servo relaxes

> ## Check your servo's range
>
> Servos vary. If yours buzzes or strains at 0° or 180°, narrow the range, for example 1.0ms to 2.0ms.

---

---

## Wiring table

| Servo wire | Connects to |
| --- | --- |
| Red (power) | VBUS, physical pin 40 |
| Brown or black (ground) | GND |
| Orange or yellow (signal) | GP18, physical pin 24 |
{:class="table table-single"}

---

## Try it Yourself

1. Run the program and watch the servo sweep.
2. Change the angle sequence to `(0, 45, 90, 135, 180)`.
3. Narrow the pulse range to 1.0 to 2.0ms if your servo strains at the ends.
4. Challenge: use a potentiometer to set the angle.

## Common Issues

- **Problem**: The servo buzzes and strains at the ends.
- **Solution**: Narrow the range, for example 1.0 to 2.0ms.
- **Why**: Some servos hit their mechanical stop before 0.5 or 2.5ms.

- **Problem**: The Pico resets when the servo moves.
- **Solution**: Power the servo from its own 5V supply and share the ground.
- **Why**: The servo draws more current than USB can supply during moves.

> **Teachers**: Servos pull a surge of current when they move. Several on one USB port can reset the Pico or the computer. Give groups one servo each, or use a powered USB hub, and always check for a shared ground.

## Check your understanding

{% quiz %}
? What sets a servo's position?
- [ ] The voltage on the power wire
- [x] The width of the pulses on the signal wire
- [ ] The PWM frequency
> The servo measures how long each pulse lasts. Around 1.5ms is the middle position.

? At 50Hz, roughly what `duty_u16()` value gives a 1.5ms pulse?
- [ ] 7500
- [x] 4915
- [ ] 32768
> 1.5ms is 7.5% of a 20ms cycle, and 7.5% of 65535 is about 4915.

? What pulse width is the middle position of a typical servo?
- [ ] 0.5ms
- [x] 1.5ms
- [ ] 20ms
> 1.5ms is the centre, and each pulse repeats every 20ms.
{% endquiz %}

---
