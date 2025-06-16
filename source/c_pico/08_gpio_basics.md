---
title: GPIO Basics
description: Learn how to control the Raspberry Pi Pico’s GPIO pins using C to turn digital outputs on and off.
layout: lesson
type: page
cover: assets/pico-c-cover.jpg
date_updated: 2025-06-15
---

![Cover](assets/pico-c-cover.jpg){:class="cover"}

---

Now that you’ve written a program and sent it to your Raspberry Pi Pico, it’s time to **interact with the real world** using the Pico’s **GPIO** pins.

GPIO stands for **General Purpose Input/Output** — these are the physical pins on your Pico that can send or receive digital signals.

---

## What You’ll Learn

- What GPIO pins are and how they work
- How to set up a pin for output
- How to turn an LED on and off using C

---

## What You’ll Need

- Raspberry Pi Pico (or Pico W)
- Breadboard (optional)
- 1x LED
- 1x 330Ω resistor
- Jumper wires

> ⚠️ Be sure to connect the long leg (anode) of the LED to the GPIO pin, and the short leg (cathode) to GND through the resistor.
>
> You can use the mnemonic - 'long is live' to remember that the long leg is positive (anode) and connects to the GPIO pin.

---

## GPIO Pin Overview

The Raspberry Pi Pico has **26 usable GPIO pins** (0–28, skipping 23–24).

Each pin can be configured as:

- **Output**: send a HIGH or LOW signal (3.3V or 0V)
- **Input**: read whether the pin is HIGH or LOW

---

## Blinking a Pin (Without Delay for Now)

Here’s the simplest GPIO example — we’ll just turn a pin **HIGH**, wait a bit, then turn it **LOW**.

```c
#include "pico/stdlib.h"

int main() {
    const uint LED_PIN = 15;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while (true) {
        gpio_put(LED_PIN, 1);  // HIGH
        sleep_ms(1000);
        gpio_put(LED_PIN, 0);  // LOW
        sleep_ms(1000);
    }
}
```

> Replace `15` with the GPIO pin number you connected your LED to.

---

## What the Code Does

- `gpio_init(pin)` sets up the pin for use
- `gpio_set_dir(pin, GPIO_OUT)` makes the pin an output
- `gpio_put(pin, value)` sends a HIGH (`1`) or LOW (`0`) signal
- `sleep_ms(ms)` pauses the program for the given milliseconds

---

## Try This

- Change the pin number and wire up a different GPIO
- Change the blink speed with shorter or longer `sleep_ms()` values
- Try using `true` and `false` instead of `1` and `0`

---

## Summary

You now know how to:

- Set up a GPIO pin as an output
- Control voltage levels from C
- Blink an LED using code!

In the next lesson, you’ll combine all of this into the **classic blinking LED project** using your own circuit.

---

Next up: [Blinking an LED](09_blinking_led)

---
