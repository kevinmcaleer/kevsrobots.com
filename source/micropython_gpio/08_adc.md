---
layout: lesson
title: Analog Input (ADC)
type: page
description: Learn about analog input and the ADC on the Raspberry Pi Pico using MicroPython.
cover: /learn/micropython_gpio/assets/banners/08_adc.jpg
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to read an analog voltage with the ADC and turn the reading into volts.

## Overview

Digital pins only see on or off. Many sensors give you a voltage that can be anywhere in between. In this lesson you'll learn how the Pico's Analog-to-Digital Converter (ADC) turns a voltage into a number.

---

## Analog input and ADC

An analog signal can have any value in a range, like the position of a dial or the amount of light in a room. The ADC measures the voltage on a pin and turns it into a number your code can use.

The Pico's ADC is available on three pins:

| ADC channel | GP pin | Physical pin |
|-------------|--------|--------------|
| ADC0 | GP26 | 31 |
| ADC1 | GP27 | 32 |
| ADC2 | GP28 | 34 |
| ADC4 | - | Internal temperature sensor |
{:class="table table-single"}

The input must stay between 0V and 3.3V.

---

## Configuring a pin for analog input

```python
from machine import ADC

adc = ADC(26)   # GP26, ADC0
```

You can also write `ADC(Pin(26))`. Either way, only GP26, GP27 and GP28 will work.

---

## Reading analog values

```python
value = adc.read_u16()
print(value)
```

`read_u16()` returns a number from `0` (0V) to `65535` (3.3V).

To turn it into a voltage:

```python
voltage = value * 3.3 / 65535
print(f"{voltage:.2f} V")
```

> ## Resolution
>
> The Pico's ADC is really 12-bit (0 to 4095). MicroPython scales it up to 16 bits so that `read_u16()` gives the same range on every board.

---

---

## A complete example

Join GP26 to the 3V3(OUT) pin (physical pin 36) with a jumper, then run:

```python
from machine import ADC
from time import sleep

adc = ADC(26)

while True:
    value = adc.read_u16()
    voltage = value * 3.3 / 65535
    print(value, f"{voltage:.2f} V")
    sleep(0.5)
```

You should see a value near 65535 and about 3.30V. Move the wire to GND and it falls to near 0.

---

## Try it Yourself

1. Run the example with the jumper on 3V3, then on GND.
2. Remove the jumper entirely. What does the reading do, and why?
3. Challenge: print "bright" when the reading is above 50000 and "dark" otherwise.

## Common Issues

- **Problem**: `ValueError` when creating `ADC`.
- **Solution**: Use GP26, GP27 or GP28 (or channel 4 for the internal sensor).
- **Why**: Only those pins are wired to the converter.

- **Problem**: The reading wobbles when nothing is connected.
- **Solution**: Connect the pin to a known voltage, or ignore the reading.
- **Why**: A pin with nothing attached floats and picks up noise, just like a digital input.

## Check your understanding

{% quiz %}
? Which of these will work as an ADC input?
- [ ] `ADC(0)` on GP0
- [x] `ADC(26)` on GP26
- [ ] `ADC(15)` on GP15
> Only GP26, GP27 and GP28 are connected to the ADC.

? `read_u16()` returns 32768. Roughly what voltage is on the pin?
- [ ] 0.33V
- [x] 1.65V
- [ ] 3.3V
> 32768 is half of 65535, so the voltage is about half of 3.3V.

? The Pico's ADC is really how many bits?
- [ ] 8
- [x] 12
- [ ] 16
> The hardware is 12-bit (0 to 4095). MicroPython scales it to 16 bits for `read_u16()`.

? What is the highest voltage you can put on an ADC pin?
- [ ] 5V
- [x] 3.3V
- [ ] 12V
> The input must stay between 0V and 3.3V.
{% endquiz %}

---
