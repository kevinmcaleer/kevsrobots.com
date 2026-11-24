---
layout: lesson
title: The Multimeter
author: Kevin McAleer
type: page
cover: /learn/snakie_instruments/assets/banners/04_the_multimeter.jpg
date: 2026-11-20
previous: 03_the_plotter.html
next: 05_the_oscilloscope.html
description: Read a sensor voltage on Snakie's on-screen Multimeter with minimum,
  maximum and average while your MicroPython loop keeps running
percent: 45
duration: 5
quiz_total: 30
date_updated: 2026-11-24
navigation:
- name: Snakie Instruments - Oscilloscope, Multimeter and Plotter
- content:
  - section: Overview
    content:
    - name: Snakie Instruments - Oscilloscope, Multimeter and Plotter
      link: 00_intro.html
    - name: The Instrument Dock
      link: 01_the_instrument_dock.html
    - name: Telemetry and LIVE
      link: 02_telemetry_and_live.html
  - section: The Instruments
    content:
    - name: The Plotter
      link: 03_the_plotter.html
    - name: The Multimeter
      link: 04_the_multimeter.html
    - name: The Oscilloscope
      link: 05_the_oscilloscope.html
    - name: Watch and Update
      link: 06_watch_and_update.html
  - section: Blocks and Logging
    content:
    - name: Instruments from Blocks
      link: 07_instruments_from_blocks.html
    - name: Logging Data
      link: 08_logging_data.html
  - section: Project
    content:
    - name: Project - Light Scanner
      link: 09_project_light_scanner.html
    - name: Summary and What's Next
      link: 10_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to read a pin's voltage on the Multimeter with inst.read_adc() and use MIN, MAX and AVG to catch a brief dip.

A plot shows the shape of a signal. Sometimes you just want the number. That's a multimeter's job.

---

## Volts from a pin

A Pico has three analog input pins: GP26, GP27 and GP28. In MicroPython you read one with an `ADC` object (analog to digital converter), and you get back a number from 0 to 65535. Nobody thinks in those units, so the first thing most programs do is turn it into volts.

`inst.read_adc()` does three jobs in one call:

1. Reads the pin
2. Converts the reading to volts and sends it to the Multimeter
3. Returns the volts, so your program can use them

Open the Multimeter in the dock and run this in the full app.

{% capture snakie_code %}
from machine import ADC
import time
import instruments as inst

sensor = ADC(26)

for i in range(150):
    volts = inst.read_adc(sensor, ch="sensor")
    if volts < 1.0:
        print("Low reading:", round(volts, 2), "V")
    time.sleep(0.1)
{% endcapture %}
{% include snakie.html code=snakie_code name="meter.py" view="code" %}

![The Multimeter showing a live reading](/learn/snakie_instruments/assets/meter.jpg){:class="img-fluid w-100"}

On the simulated device, GP26 gives a slow wave so there's something to watch. On a real Pico you'd see whatever is wired to the pin.

## Reading the meter

- The **big digits** are the latest reading in volts
- The **bar** fills from 0 to full scale, like the needle on an analog meter. It's the quickest way to see movement
- **MIN**, **MAX** and **AVG** track the lowest, highest and average reading since the meter started

MIN and MAX are the quiet heroes here. Run your robot for a minute, then look at them. If a battery sagged to 2.9 V for a split second while the motors started, MIN caught it, even though you were looking the other way.

## Your program still works

Look at the console in the picture. The program is printing **Low reading** messages at the same time as the meter updates. That's the difference from LIVE mode: telemetry rides along with a running program.

And because `read_adc()` returns the volts, measuring costs you nothing. The line you'd have written anyway now feeds an instrument too.

## Naming the channel

`ch="sensor"` names the channel. With one sensor it hardly matters. With two it matters a lot:

```python
battery = inst.read_adc(batt_pin, ch="battery")
light = inst.read_adc(light_pin, ch="light")
```

The drop-down at the top of the Multimeter picks which channel is on the display.

![Choosing the Multimeter's source](/learn/snakie_instruments/assets/meter_source.jpg){:class="img-fluid w-100"}

## Sending any value

Not every reading comes from an ADC. `inst.meter()` sends any number you like, with a unit:

```python
inst.meter(21.5, ch="room", unit="C")
```

Use it for a temperature from an I²C sensor, a current you've calculated, or a distance.

> **Teachers**: On the simulator this lesson is completely safe. On real boards the Pico's analog pins only measure 0 to 3.3 V, and a higher voltage can damage them, so check what is wired to a pin before connecting it. Keep learners to sensors and the 3V3 and GND pins for first tests, and leave batteries and mains-derived supplies to a supervised session.

---

## Try it Yourself

1. Change the threshold from `1.0` to `2.0`. Use the MIN and MAX readings to predict how often the message will print, then check.
2. Add `inst.plot(volts=volts)` to the loop and open the Plotter too. Which instrument is better for spotting the lowest point?
3. Use `inst.meter()` to show the reading as a percentage of 3.3 V, with `unit="%"`.

## Common Issues

**Problem**: The reading on a real Pico wanders about with nothing connected.
**Solution**: Connect the pin to something. For a test, wire it to GND (0 V) or 3V3.
**Why**: An unconnected input "floats" and picks up stray electrical noise.

**Problem**: The meter shows a different reading from the one I'm sending.
**Solution**: Check the source drop-down at the top of the meter.
**Why**: The simulator has a demo channel of its own, and the meter shows one channel at a time.

**Problem**: My reading is stuck at 3.3 V.
**Solution**: Check the sensor's output never goes above 3.3 V.
**Why**: A Pico's ADC measures 0 to 3.3 V. Anything higher reads as full scale, and can damage the pin.

## Check your understanding

{% quiz %}
? What does inst.read_adc(sensor) give back to your program?
- [ ] Nothing
- [ ] A number from 0 to 65535
- [x] The reading in volts
> It reads the pin, sends the volts to the Multimeter and returns them for your code to use.

? Why is the MIN reading useful?
- [x] It catches a brief dip you weren't watching for
- [ ] It makes the meter more accurate
- [ ] It resets the board
> A momentary sag is easy to miss on the digits, but MIN remembers it.

? What's the highest voltage a Pico's ADC pin should be given?
- [ ] 5 V
- [x] 3.3 V
- [ ] 12 V
> Pico pins are 3.3 V. Higher voltages need a voltage divider first.
{% endquiz %}
