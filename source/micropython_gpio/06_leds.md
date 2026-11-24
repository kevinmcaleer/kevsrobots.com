---
layout: lesson
title: Controlling LEDs with PWM
type: page
description: Learn how to control LEDs using Pulse Width Modulation (PWM) with the Raspberry Pi Pico and MicroPython.
cover: /learn/micropython_gpio/assets/banners/06_leds.jpg
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to wire an LED safely and fade it up and down with PWM.

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

---

## Choosing the resistor with Ohm's law

A red LED drops about 2V, and a GPIO pin outputs 3.3V, so the resistor has to absorb the other 1.3V. With a 220Ω resistor:

```
I = V / R = (3.3V - 2.0V) / 220Ω = 1.3 / 220 = about 6mA
```

6mA is comfortably below the 12mA limit from lesson 3, and still bright. A 330Ω resistor gives about 4mA, which is dimmer but gentler on the pin. Do not go much below 100Ω, which would push the current past 12mA.

| Part | Connects to |
| --- | --- |
| GP15 (physical pin 20) | 220Ω resistor |
| 220Ω resistor | LED long leg (anode) |
| LED short leg (cathode) | GND (physical pin 18) |
{:class="table table-single"}

---

## Try it Yourself

1. Run the fade program and watch the LED.
2. Change the `sleep(0.01)` to `sleep(0.002)`. How does the speed change?
3. Use the squared value from the note (`duty * duty // 65535`) and compare the look of the fade.
4. Challenge: wire a second LED to GP14 and make the two fade in opposite directions.

## Common Issues

- **Problem**: The LED never lights.
- **Solution**: Flip it round so the long leg faces the resistor, and check the resistor is in the circuit.
- **Why**: LEDs only conduct one way, and without a complete path to GND no current flows.

- **Problem**: The LED is very dim even at full duty.
- **Solution**: Check the resistor reads 220Ω (red red brown) not 220kΩ.
- **Why**: A resistor 1000 times too large cuts the current to almost nothing.

## Check your understanding

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

? A red LED drops about 2V and a pin gives 3.3V. What is the current with a 220Ω resistor?
- [ ] About 15mA
- [x] About 6mA
- [ ] About 60mA
> I = (3.3 - 2.0) / 220 is about 6mA.
{% endquiz %}

---
