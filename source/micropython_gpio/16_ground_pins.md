---
layout: lesson
title: Ground Pins
type: page
description: Learn about ground pins on the Raspberry Pi Pico and how to use them in your projects.
---

## Overview

Ground pins don't do anything clever, but every circuit needs them and a missing ground is behind a lot of "it doesn't work" moments. This short lesson explains why.

---

## What are ground pins?

Ground (`GND`) is the 0V reference that every other voltage in your circuit is measured against. It's also the return path that lets current flow back to the power supply and complete the circuit.

Ground pins aren't GPIO pins: you can't switch them on or off from code. They're always 0V.

---

## Identifying ground pins on the Pico

The Pico has eight ground pins, at physical pins **3, 8, 13, 18, 23, 28, 33 and 38**. They're spread out so there's always one near the pins you're using. On the board they have square pads rather than round ones.

Pin 33 is labelled `AGND`. It's the ground for the analog circuitry, so use it with potentiometers and analog sensors for the cleanest ADC readings.

All the ground pins are connected together, so you can use whichever is closest.

---

## Using ground pins in your circuits

* **Every circuit needs a ground.** An LED goes from a GPIO pin, through a resistor, to GND
* **Share grounds between power supplies.** When a motor or servo has its own battery, connect the battery's negative to the Pico's GND. Without a shared ground the Pico's signals have nothing to be measured against, and things behave strangely or not at all
* **Use the breadboard rail.** Run one wire from a GND pin to the breadboard's blue (`-`) rail, then connect everything else to the rail

---

{% quiz %}
? A servo has its own battery pack and twitches randomly. What's the most likely fix?
- [ ] Use a faster PWM frequency
- [x] Connect the battery's negative to the Pico's GND
- [ ] Move the signal wire to an ADC pin
> Without a shared ground, the servo can't read the Pico's signal reliably.

? Which ground pin is best for analog sensors?
- [ ] Pin 3
- [x] Pin 33, AGND
- [ ] It must be a GPIO pin set to 0
> AGND is the reference for the ADC, so it gives the cleanest readings.
{% endquiz %}

---
