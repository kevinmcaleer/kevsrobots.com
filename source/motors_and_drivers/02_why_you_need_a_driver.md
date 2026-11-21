---
title: Why you need a motor driver
description: Understand motor current, back-EMF and flyback diodes, and why a Pico GPIO pin must never drive a motor directly.
layout: lesson
type: page
cover: /learn/motors_and_drivers/assets/banners/02_why_you_need_a_driver.jpg
date_updated: 2026-11-21
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

It is tempting to plug a motor wire into a GPIO pin and call `pin.value(1)`. At best nothing happens. At worst you damage the Pico. Let's see why, because understanding this is the difference between copying wiring and *designing* it.

## Problem one: current

A GPIO pin is a signal, not a power supply. It is happy with a few milliamps. A Pico pin drives 4 mA by default and 12 mA at most, and the whole chip has a limit too.

A little TT gear motor spinning freely might draw 150 to 250 mA. Pressed against a wall, or with the wheel held, it can draw an amp or more. That is the **stall current**, and it is the number your driver and battery must be able to handle.

```python
# How many GPIO pins would we need to power ONE motor?
gpio_limit_ma = 12      # the most a Pico pin should supply
motor_running_ma = 200  # a typical TT motor, no load
motor_stalled_ma = 1000 # the same motor with the wheel held

print(motor_running_ma / gpio_limit_ma)  # about 17 pins, just to spin it freely
print(motor_stalled_ma / gpio_limit_ma)  # about 83 pins, if it jams
```

Obviously that is not going to work. A driver takes a tiny signal from the Pico and uses it to switch a much larger current from a separate battery.

## Problem two: back-EMF

A motor is also a generator. Spin the shaft and it produces a voltage, which is how a bicycle dynamo works. While the motor runs, it makes a voltage that opposes the supply: the **back-EMF** (electromotive force). That is why a running motor draws less current than a stalled one.

The nasty part comes when you switch the motor off. A motor is also a coil, and a coil resists changes in current. Cut the power suddenly and the coil pushes back with a sharp voltage spike, often many times the supply voltage, trying to keep the current flowing.

```text
Switch closes   -> current builds up slowly through the coil
Switch opens    -> coil resists the change
                -> voltage spike across the switch (inductive kick)
```

That spike can punch through a transistor, or couple back into the Pico and cause a random reset.

## The fix: flyback diodes

A **flyback diode** is wired across the motor, backwards, so it does nothing during normal running. When the spike arrives, the diode gives the current a safe path to circulate and die away.

```text
   +V ---+------------+
         |            |
       Motor        Diode  (stripe towards +V)
         |            |
   Switch+------------+
         |
        GND
```

You usually do not add these yourself:

- An **L298N module** has a ring of diodes built in (look for the row of small black parts marked 1N4007).
- A **DRV8833** has diodes built into the chip.

If you ever drive a bare motor with a single transistor, add the diode. It costs pennies and saves a Pico.

## What a driver gives you

1. A path for far more current than the Pico can supply
2. Protection against back-EMF spikes
3. Direction control, from the H-bridge circuit in lesson 4
4. A separate power input, so noisy motors stay away from your logic

## Try it Yourself

1. Find the stall current of your motor in its datasheet or seller listing. Compare it to the current limit of your driver (about 2 A per channel for an L298N, about 1.5 A for a DRV8833).
2. Look at your L298N board and find the diodes. How many are there, and why that number?
3. Challenge: spin a motor by hand while a multimeter on its terminals reads volts. You have just built a generator.

## Common Issues

**Problem**: The Pico resets or freezes whenever the motors start.
**Solution**: Keep the motor supply separate from the Pico's, join only the grounds, and check your flyback diodes.
**Why**: Motor spikes and current surges travel down shared wires and disturb the Pico's supply.

**Problem**: The motor gets very hot while the wheel is held.
**Solution**: Do not hold a motor stalled for more than a moment.
**Why**: In a stall there is no back-EMF to limit current, so all the power turns into heat.
