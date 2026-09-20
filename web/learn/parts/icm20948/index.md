---
title: ICM20948 9-DoF Motion Sensor
description: >-
  The TDK InvenSense ICM-20948 packs a 3-axis accelerometer, gyroscope and
  magnetometer into one I²C chip — nine degrees of freedom. Here's how to wire
  a breakout to a Raspberry Pi Pico and read it with MicroPython.
layout: content
---

{% include breadcrumbs.html %}

# ICM20948 9-DoF Motion Sensor

The `ICM-20948` is a TDK InvenSense motion sensor that packs a **3-axis
accelerometer**, a **3-axis gyroscope** and a **3-axis magnetometer** (an
on-board AK09916) into one tiny I²C package — **nine degrees of freedom** in
total. That's everything you need for tilt sensing, orientation tracking,
balancing robots and a compass heading.

Breakouts such as Pimoroni's run at **3.3 V or 5 V**, so it's happy on a
Raspberry Pi Pico, an ESP32 or a Raspberry Pi.

---

## Key specs

| Spec | Value |
|------|-------|
| Accelerometer | ±2 / 4 / 8 / 16 g |
| Gyroscope | ±250 / 500 / 1000 / 2000 °/s |
| Magnetometer | ±4900 µT (AK09916) |
| Interface | I²C, address **0x68** (or **0x69** via the address jumper/trace) |
| Power | 3.3 V or 5 V (breakout-dependent) |

The magnetometer sits on the ICM-20948's internal auxiliary I²C bus — a good
driver configures the chip's aux-bus master to stream it for you, so there's
nothing extra to wire.

---

## Wiring

Pin order below is Pimoroni's Breakout Garden header — check the silkscreen on
your breakout:

| Breakout pin | Connect to |
|--------------|------------|
| **3-5V** | 3V3 (or 5 V) |
| **GND** | GND |
| **SDA** | an I²C SDA pin — e.g. Pico **GP4** (I2C0) |
| **SCL** | an I²C SCL pin — e.g. Pico **GP5** (I2C0) |
| **INT** | *optional* — any GPIO, for the data-ready interrupt |

---

## MicroPython example

First check the sensor is answering on the bus:

```python
from machine import I2C, Pin

i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=400_000)
print(i2c.scan())     # expect [104] (0x68) or [105] (0x69)
```

Pimoroni's custom MicroPython build includes a driver for their breakout, and
[Snakie](https://www.snakie.org) bundles a plain-MicroPython `icm20948` driver
with the part — with that on your board, reading all nine axes looks like this:

```python
from machine import I2C, Pin
from icm20948 import ICM20948
import time

i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=400_000)
imu = ICM20948(i2c)                # auto-detects 0x68 / 0x69

while True:
    ax, ay, az = imu.read_accel()  # g
    gx, gy, gz = imu.read_gyro()   # degrees / second
    mx, my, mz = imu.read_mag()    # microtesla (µT)
    print("accel g   ", ax, ay, az)
    print("gyro dps  ", gx, gy, gz)
    print("mag  uT   ", mx, my, mz)
    print("-")
    time.sleep(0.5)
```

A quick sanity check: a flat, still board reads roughly **1 g on the Z axis**
and about **0 °/s** on all gyro axes — gravity is always there, rotation isn't.

---

## Troubleshooting

- **Nothing in `i2c.scan()`** → check SDA/SCL aren't swapped, and that power
  and ground are solid. The sensor should show up as `104` (0x68) or `105`
  (0x69).
- **It scans but reads fail** → the bus can't clock data reliably: add strong
  pull-up resistors on SDA and SCL to 3V3, shorten the wires, and check the
  common ground. On **RP2350** boards use **~2.2 kΩ** pull-ups — erratum
  RP2350-E9 makes the usual 4.7 kΩ marginal.
- **Compass heading is off** → keep the sensor away from motors, speakers and
  magnets, and calibrate for your local environment.

---

## Use it in Snakie

The ICM20948 is in the Standard parts library of
[Snakie](https://www.snakie.org), my free, open-source MicroPython editor.
Drag it onto the Board View and Snakie shows you the wiring and copies the
`icm20948` driver to your board — and the IMU instrument gives you a live 3-D
attitude view driven straight from the sensor.

---

## Related learning

- [I2C — GPIO Mastery course](/learn/micropython_gpio/12_i2c.html)
- [I2C and SPI Explained](/learn/i2c_spi/) — scanning, datasheets and debugging
- [Robotics with MicroPython](/learn/micropython_robotics/)
