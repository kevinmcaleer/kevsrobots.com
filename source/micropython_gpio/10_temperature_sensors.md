---
layout: lesson
title: Using Temperature Sensors
type: page
description: Learn how to use temperature sensors with the Raspberry Pi Pico and MicroPython.
---

## Overview

In this lesson you'll read temperatures three ways: with the sensor built into the Pico, a DS18B20 digital probe and a DHT11 temperature and humidity sensor.

---

## What you'll need

* A Raspberry Pi Pico
* Optional: a DS18B20 sensor and a 4.7kΩ resistor
* Optional: a [DHT11](/resources/glossary#dht11) sensor (modules usually include the pull-up resistor)
* A breadboard and jumper wires

---

## What are temperature sensors?

Temperature sensors measure how hot or cold something is. Some, like thermistors, give an analog voltage. Others, like the DS18B20 and DHT11, do the measuring themselves and send the result as digital data.

---

## The built-in sensor

The RP2040 chip has a temperature sensor on ADC channel 4. It measures the chip's temperature, so it reads a little warmer than the room, but it needs no wiring at all:

```python
from machine import ADC
from time import sleep

sensor = ADC(4)

while True:
    voltage = sensor.read_u16() * 3.3 / 65535
    temperature = 27 - (voltage - 0.706) / 0.001721
    print(f"{temperature:.1f} C")
    sleep(1)
```

The formula comes from the RP2040 datasheet. There's a whole course on it: [Using the Raspberry Pi Pico's Built-in Temperature Sensor](/learn/pico_temp_sensor/).

---

## DS18B20 example

The DS18B20 uses the 1-Wire bus. Connect its `VDD` to `3V3`, `GND` to `GND` and `DQ` (data) to `GP4`, with a 4.7kΩ resistor between `DQ` and `3V3`.

```python
from machine import Pin
from time import sleep_ms
import onewire, ds18x20

sensor = ds18x20.DS18X20(onewire.OneWire(Pin(4)))
roms = sensor.scan()
print("Found sensors:", roms)

while True:
    sensor.convert_temp()
    sleep_ms(750)                     # the sensor needs time to measure
    for rom in roms:
        print(f"{sensor.read_temp(rom):.2f} C")
```

- `scan()` returns a list with the address of every DS18B20 on the bus. You can connect several to the same pin
- `convert_temp()` asks every sensor to take a reading, which takes up to 750ms
- `read_temp(rom)` reads the result from one sensor

---

## DHT11 example

Connect the DHT11's `VCC` to `3V3`, `GND` to `GND` and its data pin to `GP15`.

```python
from machine import Pin
from time import sleep
import dht

sensor = dht.DHT11(Pin(15))

while True:
    sensor.measure()
    print(f"Temperature: {sensor.temperature()} C, Humidity: {sensor.humidity()} %")
    sleep(2)
```

`measure()` takes a reading, then `temperature()` and `humidity()` return the values. The DHT11 can only take one reading every second or so, which is why we wait two seconds between readings.

---

## Common issues

**Problem:** The DS18B20 `scan()` returns an empty list.
**Solution:** Check the 4.7kΩ pull-up resistor between data and 3V3.
**Why:** The 1-Wire bus relies on the pull-up to hold the line high between messages.

**Problem:** The DHT11 raises `OSError: [Errno 110] ETIMEDOUT`.
**Solution:** Check the data wire and wait at least a second between readings.
**Why:** The sensor didn't reply in time, usually because of wiring or reading too often.

---

{% quiz %}
? How long should you wait after `convert_temp()` before reading a DS18B20?
- [ ] No time at all
- [ ] 10 microseconds
- [x] Up to 750 milliseconds
> At full resolution the sensor takes up to 750ms to finish a measurement.

? Which ADC channel is the Pico's built-in temperature sensor on?
- [ ] ADC0
- [ ] ADC2
- [x] ADC4
> ADC0 to ADC2 are GP26 to GP28. ADC4 is wired to the on-chip temperature sensor.
{% endquiz %}

---
