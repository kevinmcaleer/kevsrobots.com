---
layout: lesson
title: Instruments from Blocks
author: Kevin McAleer
type: page
cover: /learn/snakie_instruments/assets/banners/07_instruments_from_blocks.jpg
date: 2026-11-20
previous: 06_watch_and_update.html
next: 08_logging_data.html
description: Feed Snakie's Plotter, Multimeter and Oscilloscope from a Blocks program
  using the Instruments drawer, and read the Python it writes
percent: 72
duration: 4
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

By the end of this lesson you will be able to use the Instruments drawer in the Blocks workspace and read the Python that the blocks write.

You don't need to type Python to use an instrument. Everything in this course has a block.

---

## The Instruments drawer

Click **Blocks** in the workspace switcher. In the toolbox on the left, click **Instruments**.

![The Instruments drawer in the Blocks workspace](/learn/snakie_instruments/assets/blocks_instruments.jpg){:class="img-fluid w-100"}

The blocks read like sentences. Here are the ones that match what you've learned:

| Block | Python it writes | Lesson |
|---|---|---|
| **plot** value **=** 0 | `inst.plot(value=0)` | 3 |
| **multimeter: show** 0 **V on channel** adc0 | `inst.meter(...)` | 4 |
| **read volts on** GP26 **and show on the multimeter** | `inst.read_adc(...)` | 4 |
| **oscilloscope: plot** 0 **on channel** ch1 | `inst.scope(...)` | 5 |
| **read the PWM on** GP15 **and show on the oscilloscope** | `inst.read_pwm(...)` | 5 |
{:class="table table-single"}

Further down the drawer are blocks for the other instruments in the dock: **radar: show** a distance, **IMU: roll pitch yaw**, **barometer**, **scan the I²C bus**, **scan for Wi-Fi networks** and more.

## A light meter in four blocks

Build this:

1. From **Control**, drag out a **forever** block
2. From **Instruments**, drop a **plot value =** block inside it and change the name to `light`
3. Drop **read volts on GP26 and show on the multimeter** into the plot block's value slot
4. From **Wait**, add **wait 400 milliseconds** under the plot block and change it to 200

The **read volts** block is round, which means it gives back a value. It fits anywhere a number fits. So one block feeds the Multimeter *and* hands the Plotter its number.

## Read the Python

The code pane beside your blocks shows what Snakie wrote:

{% capture snakie_code %}
import time
import instruments as inst
from machine import ADC
adc_26 = ADC(26)
while True:
    inst.plot(light=inst.read_adc(adc_26))
    time.sleep_ms(200)
{% endcapture %}
{% include snakie.html code=snakie_code name="blocks.py" view="code" %}

Every line should look familiar. `import instruments as inst` appeared on its own when you used the first Instruments block, and the ADC on GP26 was set up for you.

Switch to **Code** to see the Instrument Dock, open the Plotter and Multimeter, and press **Run**.

## Two blocks for later

Two blocks near the top of the drawer do something different:

- **let Snakie drive this board**
- **check for Snakie commands**

These work the other way round. They let panels in the dock, such as the Servo and Motor outputs, send commands *to* your running program. We won't need them in this course, but now you know what they're for.

---

## Try it Yourself

1. Add a second **plot** block that plots the same reading multiplied by 100 (use a block from **Maths**). Which line fills the chart?
2. Swap the **read volts** block for **read the PWM on GP15 and show on the oscilloscope**. Run it and check the scope.
3. Edit a number in the Python and switch back to Blocks. What does Snakie do?

## Common Issues

**Problem**: I can't see the Instrument Dock in Blocks.
**Solution**: Switch to the **Code** workspace. Your program comes with you.
**Why**: The dock is part of the Code workspace's side panel.

**Problem**: The **read volts** block won't snap into the loop.
**Solution**: Drop it into a value slot, such as the one on a **plot** block.
**Why**: Round blocks are values. They fit into slots, not between other blocks.

**Problem**: The plot is called `value`.
**Solution**: Click the name on the plot block and type a better one.
**Why**: `value` is just the block's starting name.

## Check your understanding

{% quiz %}
? Which Python does the "plot value = 0" block write?
- [ ] print(0)
- [x] inst.plot(value=0)
- [ ] inst.scope(0)
> Each Instruments block writes one call to the instruments library.

? Why can the "read volts" block sit inside the plot block?
- [x] It returns a value, so it fits any number slot
- [ ] It's the same colour
- [ ] All blocks fit everywhere
> Like inst.read_adc() in Python, it reports to the meter and gives the volts back.

? Who adds "import instruments as inst" to a Blocks program?
- [ ] You have to type it
- [ ] The board
- [x] Snakie, as soon as you use an Instruments block
> Blocks takes care of imports and pin setup for you.
{% endquiz %}
