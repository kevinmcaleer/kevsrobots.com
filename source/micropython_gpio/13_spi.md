---
layout: lesson
title: Introduction to SPI
type: page
description: Learn about SPI communication protocol and how to use it with the Raspberry Pi Pico and MicroPython.
---

## Overview

In this lesson, you'll learn about the SPI (Serial Peripheral Interface) bus. SPI is faster than I2C and is often used for colour displays and SD cards.

---

## What is SPI?

SPI is a synchronous bus that uses four wires:

* `SCK` - the clock, driven by the Pico
* `MOSI` (also called `TX` or `SDO`) - data from the Pico to the device
* `MISO` (also called `RX` or `SDI`) - data from the device to the Pico
* `CS` (chip select, sometimes `SS`) - set low to tell one device "I'm talking to you"

Several devices can share `SCK`, `MOSI` and `MISO`, but each needs its own `CS` pin.

---

## Setting up SPI on the Pico

The Pico has two SPI buses, `SPI0` and `SPI1`. This example uses `SPI0` on `GP2` (SCK), `GP3` (MOSI) and `GP4` (MISO), with `GP5` as chip select:

```python
from machine import Pin, SPI

spi = SPI(0, baudrate=1_000_000, polarity=0, phase=0,
          sck=Pin(2), mosi=Pin(3), miso=Pin(4))
cs = Pin(5, Pin.OUT, value=1)    # high means "not selected"
```

`polarity` and `phase` describe how the clock behaves. Mode 0 (both `0`) is the most common; your device's datasheet will say which it needs.

---

## Sending and receiving data

```python
cs.value(0)                       # select the device
spi.write(b'\x01\x02\x03')       # send three bytes
data = spi.read(3)                # read three bytes back
cs.value(1)                       # deselect
print(data)
```

SPI sends and receives at the same time. To send and capture the reply in one go, use `write_readinto()`:

```python
reply = bytearray(3)
cs.value(0)
spi.write_readinto(b'\x9f\x00\x00', reply)
cs.value(1)
```

---

{% quiz %}
? What is the chip select (CS) pin for?
- [ ] It carries the clock signal
- [x] It picks which device on the bus is being talked to
- [ ] It carries data back to the Pico
> Each device has its own CS pin. Pulling it low selects that device.

? Which is usually faster?
- [x] SPI
- [ ] I2C
> SPI commonly runs at several MHz, while I2C is usually 100kHz or 400kHz.
{% endquiz %}

---
