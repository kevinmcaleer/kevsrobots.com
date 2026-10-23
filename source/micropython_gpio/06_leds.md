---
layout: lesson
title: Controlling LEDs with PWM
type: page
description: Learn how to control LEDs using Pulse Width Modulation (PWM) with the Raspberry Pi Pico and MicroPython.
---

## Overview

Time to put PWM to work. In this lesson you'll wire up an LED and make it fade smoothly up and down.

---

## What you'll need

* A Raspberry Pi Pico
* An LED and a 220Ω resistor
* A breadboard and jumper wires

---

## Wiring the LED

1. Connect `GP15` (physical pin 20) to one end of the 220Ω resistor
2. Connect the other end of the resistor to the LED's **long** leg (the anode)
3. Connect the LED's **short** leg (the cathode) to a `GND` pin

The resistor limits the current so the LED and the Pico pin aren't damaged.

---

## Turning the LED on and off

Before using PWM, check the wiring with a plain digital output:

```python
from machine import Pin

led = Pin(15, Pin.OUT)
led.value(1)
```

---

## Using PWM to control the brightness

```python
from machine import Pin, PWM
from time import sleep

led = PWM(Pin(15))
led.freq(1000)

while True:
    # fade up
    for duty in range(0, 65536, 1024):
        led.duty_u16(duty)
        sleep(0.01)
    # fade down
    for duty in range(65535, -1, -1024):
        led.duty_u16(duty)
        sleep(0.01)
```

- `led.freq(1000)` switches the LED on and off 1000 times a second, far too fast to see
- The first loop steps the duty from 0 up to 65535 in steps of 1024, so the LED brightens
- The second loop steps it back down again

> ## Brightness isn't linear
>
> Your eyes notice changes at low brightness much more than at high brightness, so the fade looks quick at the start and slow at the end. Squaring the value (`led.duty_u16(duty * duty // 65535)`) gives a smoother-looking fade.

---

## Try it yourself

Add a potentiometer later in the course (lesson 9) and use its reading to set the LED brightness directly.

{% quiz %}
? Why does the LED need a resistor?
- [ ] To make PWM work
- [x] To limit the current through the LED and the GPIO pin
- [ ] To convert 5V to 3.3V
> Without a resistor the LED would draw too much current and could damage itself and the pin.

? Which leg of the LED goes towards GND?
- [ ] The long leg (anode)
- [x] The short leg (cathode)
> Current flows from the anode (long leg) to the cathode (short leg).
{% endquiz %}

---
