---
title: "Switches and Pull Resistors"
description: "Learn how a push button works with a Pico input, why inputs need pull-up or pull-down resistors, and how to debounce."
layout: lesson
type: page
cover: /learn/electronics_basics/assets/banners/10_switches_and_pull_resistors.jpg
date_updated: 2026-11-21
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

## Overview

A push button sounds like the simplest part there is, but wiring one to a microcontroller has a trap: an input pin that is connected to nothing is not reliably 0 or 1. In this lesson we learn about pull-up and pull-down resistors, and add a button to our night light as a manual override.

## The floating input

A switch either connects two points or does not. When the button is pressed, the pin is connected to something. When it is released, the pin is connected to... nothing. A pin left unconnected is called **floating**, and it picks up stray electrical noise from your hand, the table, and the room. It might read 0, then 1, then 0, with no one touching anything.

The fix is a **pull resistor**, a resistor, usually 10k, that gives the pin a default level.

## Pull-down and pull-up

```text
Pull-down                       Pull-up
3.3 V                           3.3 V
  |                               |
[button]                        [10k]
  |                               |
  +---- pin                       +---- pin
  |                               |
[10k]                           [button]
  |                               |
 GND                             GND
```

- **Pull-down**: the resistor holds the pin at 0 V. Pressing the button connects the pin to 3.3 V, so it reads 1 when pressed.
- **Pull-up**: the resistor holds the pin at 3.3 V. Pressing the button connects the pin to GND, so it reads 0 when pressed.

The resistor is large (10k) so that, when the button is pressed, only a tiny current flows: 3.3 V / 10k = 0.33 mA. A small resistor would waste power for no reason.

It is the same voltage divider idea as the last lesson: with the button open, the pin sees the resistor's voltage with no load. With the button closed, the pin takes the button's voltage.

## The Pico has them built in

The Pico's pins have internal pull resistors, about 50k to 80k, which you switch on in code. So you can wire a button directly from a pin to ground and skip the external resistor. We use GP14:

Wiring: one leg of the button to **GP14** (physical pin 19), the diagonally opposite leg to **GND** (physical pin 18).

```python
# MicroPython on the Pico
# Button on GP14 to GND, with the internal pull-up
from machine import Pin
from time import sleep_ms

button = Pin(14, Pin.IN, Pin.PULL_UP)   # reads 1 normally, 0 when pressed

while True:
    if button.value() == 0:
        print("Pressed")
    else:
        print("Released")
    sleep_ms(200)
```

Because it is a pull-up, the logic is inverted: **0 means pressed**. This catches everyone once.

## Bounce

When a metal contact closes, it does not close cleanly. It bounces for a few milliseconds, so a single press can look like five presses. The easy fix: after seeing a press, ignore the button for about 50 ms. Here is a toggle that handles that, which we will reuse in the project:

```python
# MicroPython on the Pico
# Toggle a flag with each button press, with debounce
from machine import Pin
from time import sleep_ms, ticks_ms, ticks_diff

button = Pin(14, Pin.IN, Pin.PULL_UP)
led = Pin("LED", Pin.OUT)       # the on-board LED on a Pico W or Pico 2 W

override = False
last_change = ticks_ms()
was_pressed = False

while True:
    pressed = button.value() == 0
    # Act only on the moment the button goes down, and ignore bounces
    if pressed and not was_pressed and ticks_diff(ticks_ms(), last_change) > 50:
        override = not override
        last_change = ticks_ms()
        led.value(override)
        print("Override is", "ON" if override else "OFF")
    was_pressed = pressed
    sleep_ms(5)
```

On a plain Pico or Pico 2 (not the W versions) use `Pin(25, Pin.OUT)` instead of `Pin("LED")`.

**Why this matters:** every limit switch, bumper and user button on every robot uses this pattern.

**Common mistakes:**

- Leaving the pin floating. Always use a pull resistor.
- Forgetting the pull-up inverts the logic.
- Using two legs that are already joined inside the button. On a 4-pin tactile switch, pick legs diagonal to each other.

## Try it Yourself

1. Run the first program and watch the output while you press the button.
2. Remove `Pin.PULL_UP` from the code and see what the pin does when you leave it alone. Put it back afterwards.
3. Challenge: change the toggle program so that holding the button for a second does something different from a quick tap.

## Common Issues

**Problem**: The program always says Pressed.
**Solution**: Check the wire is on the right button leg, and that it is not stuck in the ground row.
**Why**: Two legs on the same side of a 4-pin button are always connected.

**Problem**: One press toggles several times.
**Solution**: Increase the debounce time to 80 ms or so.
**Why**: Some cheap buttons bounce for longer.
