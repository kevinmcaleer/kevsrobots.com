---
title: Parts
description: >-
  Reference guides for common robotics parts — what each part is, how to wire it
  up, and how to use it from MicroPython.
layout: content
---

{% include breadcrumbs.html %}

# Parts

## Quick reference guides for common robotics parts

Each guide covers what the part is, its key specs, how to wire it to a
Raspberry Pi Pico (or any MicroPython board), and a small example program to
get you going.

These parts are also in the Standard parts library of
[Snakie](https://www.snakie.org), my free, open-source MicroPython editor —
drag a part onto the Board View and Snakie shows you where the wires go and
hands you the driver and example code.

---

## The parts

| Part | What it does |
|------|--------------|
| [SG90 Micro Servo](/learn/parts/sg90/) | 9 g hobby servo with ~180° of travel — pan/tilt rigs, grippers, robot legs |
| [Potentiometer](/learn/parts/potentiometer/) | A rotary knob that gives you an analog value — the classic ADC input |
| [BME280](/learn/parts/bme280/) | Bosch environmental sensor — temperature, pressure and humidity over I²C |
| [ICM20948](/learn/parts/icm20948/) | 9-DoF motion sensor — accelerometer, gyroscope and magnetometer over I²C |

---

## Want to go deeper?

These courses use the parts above:

- [Raspberry Pi Pico with MicroPython — GPIO Mastery](/learn/micropython_gpio/) — PWM, ADC, I²C and more
- [I2C and SPI Explained](/learn/i2c_spi/) — how the buses these sensors sit on actually work
- [Learn MicroPython — The basics](/learn/micropython/) — start here if MicroPython is new to you
