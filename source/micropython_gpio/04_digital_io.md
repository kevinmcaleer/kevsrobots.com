---
layout: lesson
title: Digital I/O
type: page
description: Learn about digital input and output with Raspberry Pi Pico using MicroPython.
cover: /learn/micropython_gpio/assets/banners/04_digital_io.jpg
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to switch an output pin on and off and read a button using the Pico's built-in pull-up resistor.

## Overview

Digital I/O is the simplest thing a GPIO pin can do: a pin is either on (`1`) or off (`0`). In this lesson you'll make outputs, read inputs and wire up a button the right way.

---

## Digital input and output

**Digital output** means the Pico sets a pin high (3.3V) or low (0V), for example to turn an LED on or off.

**Digital input** means the Pico reads whether a pin is high or low, for example to tell whether a button is pressed.

---

## Configuring pins for digital I/O

You choose what a pin does with the `Pin` class from the `machine` module.

```python
from machine import Pin

led = Pin(15, Pin.OUT)                   # GP15 as an output
button = Pin(14, Pin.IN, Pin.PULL_UP)    # GP14 as an input with the pull-up resistor on
```

---

## Writing digital values

```python
led.value(1)   # on
led.value(0)   # off
led.toggle()   # flip to the other state
```

## Reading digital values

```python
print(button.value())
```

`value()` with no argument returns `0` or `1`.

---

## Wiring a button with a pull-up

An input pin that isn't connected to anything "floats" and reads random values. A pull-up resistor holds it at `1` until something pulls it to ground. The Pico has these resistors built in, so you only need `Pin.PULL_UP`.

Wire one leg of the button to `GP14` and the other leg to `GND`. Now the pin reads `1` when the button is up and `0` when it's pressed.

```python
from machine import Pin
from time import sleep

led = Pin(15, Pin.OUT)
button = Pin(14, Pin.IN, Pin.PULL_UP)

while True:
    if button.value() == 0:   # pressed
        led.value(1)
    else:
        led.value(0)
    sleep(0.01)
```

---

---

## Wiring table

| Part | Connects to |
| --- | --- |
| LED long leg (anode), via a 220Ω resistor | GP15 (physical pin 20) |
| LED short leg (cathode) | GND |
| Button, one leg | GP14 (physical pin 19) |
| Button, other leg | GND |
{:class="table table-single"}

---

## Try it Yourself

1. Run the button program and watch the LED follow the button.
2. Swap `Pin.PULL_UP` for `Pin.PULL_DOWN` and move the button's second leg to `3V3`. What does `value()` read when pressed now?
3. Challenge: make each press toggle the LED instead of lighting it only while held. Hint: remember the last button value and only act when it changes from `1` to `0`.

## Common Issues

- **Problem**: The LED stays on or flickers when the button is not pressed.
- **Solution**: Check the button goes between GP14 and GND, and that you used `Pin.PULL_UP`.
- **Why**: Without a pull resistor the pin floats and picks up noise, so it reads random values.

- **Problem**: One press toggles the LED several times.
- **Solution**: Add `sleep(0.2)` after a press is detected.
- **Why**: Mechanical buttons bounce: the contact makes and breaks many times in a few milliseconds.

## Check your understanding

{% quiz %}
? A button is wired between GP14 and GND, with `Pin.PULL_UP`. What does `button.value()` return while it's pressed?
- [x] 0
- [ ] 1
- [ ] It changes randomly
> Pressing the button connects the pin to ground, so it reads 0. The pull-up makes it read 1 when released.

? What happens to an input pin with no pull-up or pull-down and nothing connected?
- [ ] It always reads 0
- [x] It floats and can read random values
- [ ] It reads 3.3V as an analog value
> A floating input picks up electrical noise. Pull resistors give it a known default.

? Which constant turns on the Pico's internal pull-up resistor?
- [ ] `Pin.OUT`
- [x] `Pin.PULL_UP`
- [ ] `Pin.IN` on its own
> `Pin(14, Pin.IN, Pin.PULL_UP)` enables the resistor, so no external one is needed.
{% endquiz %}

---
