---
title: Potentiometer
description: >-
  A potentiometer is a rotary variable resistor — the classic analog input.
  Here's how to wire one to a Raspberry Pi Pico and read it with MicroPython's
  ADC.
layout: content
---

{% include breadcrumbs.html %}

# Potentiometer

A `potentiometer` ("pot") is a rotary variable resistor with three terminals.
Wire the two outer pins to power and ground and the middle **wiper** pin forms
a variable voltage divider — turning the knob sweeps the wiper voltage smoothly
from 0 V up to the supply voltage. Read that voltage with an ADC pin and you've
got a knob your code can react to: volume controls, speed dials, menu
selectors, servo testers.

The common hobby part is a **10 kΩ** rotary pot, which is a good value for
microcontroller ADC inputs.

You can learn more about [how potentiometers work here](/resources/how_it_works/pots.html).

---

## Wiring

| Pin | Connect to |
|-----|------------|
| VCC (outer) | **3V3** |
| OUT (middle, the wiper) | an ADC-capable GPIO — Pico: **GP26 / GP27 / GP28** |
| GND (outer) | GND |

Two things worth knowing on the Pico:

- Only three ADC channels come out on the pins: **GP26 (ADC0)**, **GP27
  (ADC1)** and **GP28 (ADC2)**.
- Use **3V3** for the pot, not 5 V — the Pico's ADC inputs are 3.3 V max.

If the value moves the "wrong way" when you turn the knob, just swap the two
outer wires.

---

## MicroPython example

With the wiper on **GP26**:

```python
from machine import ADC, Pin
import time

pot = ADC(Pin(26))                # OUT wired to GP26 (ADC0)

while True:
    raw = pot.read_u16()          # 0 … 65535
    pct = raw * 100 // 65535      # 0 … 100 %
    volts = raw / 65535 * 3.3
    print(pct, "%", round(volts, 2), "V")
    time.sleep(0.1)
```

`read_u16()` always returns a 16-bit value (0–65535), whatever the chip's
actual ADC resolution — so the maths above works on any MicroPython board.

The reading is always a little noisy. If you need a steadier value, average a
few samples:

```python
def read_smooth(adc, samples=16):
    return sum(adc.read_u16() for _ in range(samples)) // samples
```

---

## Use it in Snakie

The potentiometer is in the Standard parts library of
[Snakie](https://www.snakie.org), my free, open-source MicroPython editor.
Drag it onto the Board View to see the wiring, and the Potentiometer
instrument — a vintage 0–100 % meter — follows the knob live as you turn it.

---

## Related learning

- [Potentiometers — GPIO Mastery course](/learn/micropython_gpio/09_potentiometers.html)
- [Analog to Digital Converters (ADC)](/learn/micropython_gpio/08_adc.html)
- [How potentiometers work](/resources/how_it_works/pots.html)
