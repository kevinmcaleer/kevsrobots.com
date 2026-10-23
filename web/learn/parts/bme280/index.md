---
title: BME280 Environmental Sensor
description: >-
  The Bosch BME280 measures temperature, barometric pressure and humidity in
  one tiny I²C chip. Here's how to wire a BME280 breakout to a Raspberry Pi
  Pico and read it with MicroPython.
layout: content
---

{% include breadcrumbs.html %}

# BME280 Environmental Sensor

The `BME280` is a Bosch sensor that measures **temperature**, **barometric
pressure** and **relative humidity** in one tiny chip — a complete weather
station on a breakout board. It talks **I²C** at address **0x76** (or **0x77**,
selected with the ADDR pin/trace depending on your breakout), so it wires
straight to a Raspberry Pi Pico, ESP32, micro:bit or Raspberry Pi with just
four wires.

Breakouts such as Pimoroni's run from **2–6 V** and also offer a Qw/ST
(Qwiic / STEMMA QT) connector, so you can plug it in with a single cable — no
soldering.

> Buying tip: the **BMP280** looks almost identical and is often sold in the
> same listings, but it has **no humidity sensor**. If you want all three
> readings, check it really is a BME280.

---

## What it measures

| Reading | Unit |
|---------|------|
| Temperature | °C |
| Barometric pressure | hPa (1 hPa = 1 mbar; sea level is ~1013 hPa) |
| Relative humidity | %RH |

Pressure is the interesting one — because pressure falls as you go up, the
BME280 is also commonly used as an **altimeter**, and pressure trends are the
basis of simple weather forecasting.

---

## Wiring

Pin order below is Pimoroni's Breakout Garden header — check the silkscreen on
your breakout, as other makers order the pins differently:

| Breakout pin | Connect to |
|--------------|------------|
| **2-6V** | 3V3 |
| **SDA** | an I²C SDA pin — Pico I2C0: **GP0** |
| **SCL** | an I²C SCL pin — Pico I2C0: **GP1** |
| **INT** | leave unconnected |
| **GND** | GND |

---

## MicroPython example

First check the sensor is answering on the bus:

```python
from machine import I2C, Pin

i2c = I2C(0, sda=Pin(0), scl=Pin(1))
print(i2c.scan())     # expect [118] (0x76) or [119] (0x77)
```

Then, with the widely-used community `bme280` driver
([micropython-bme280](https://github.com/robert-hh/BME280)) copied to your
board:

```python
from machine import I2C, Pin
import bme280
import time

i2c = I2C(0, sda=Pin(0), scl=Pin(1))
bme = bme280.BME280(i2c=i2c)

while True:
    temperature, pressure, humidity = bme.values
    print(temperature, pressure, humidity)
    time.sleep(1)
```

If the driver can't find the sensor, it's usually one of: SDA/SCL swapped, the
sensor on the other address (try `0x77`), or a missing ground connection — run
`i2c.scan()` again and see what comes back.

The readings are per-unit accurate out of the box: every BME280 is factory
calibrated, and the driver reads each chip's own calibration constants and
applies the compensation maths from the Bosch datasheet for you.

---

## Use it in Snakie

The BME280 is in the Standard parts library of
[Snakie](https://www.snakie.org), my free, open-source MicroPython editor.
Drag it onto the Board View and Snakie shows you the wiring and copies its
bundled `bme280` driver to the board — `read()` hands you `(temperature °C,
pressure hPa, humidity %RH)` as plain floats, ready for the live instruments.

---

## Related learning

- [BME280 Deep Dive — I2C and SPI Explained course](/learn/i2c_spi/06_bme280-deep-dive.html)
- [I2C and SPI Explained](/learn/i2c_spi/) — the full course
- [I2C — GPIO Mastery course](/learn/micropython_gpio/12_i2c.html)
