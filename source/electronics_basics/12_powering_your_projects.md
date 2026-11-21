---
title: "Powering Your Projects"
description: "Batteries, regulators, 3.3 V versus 5 V, common ground, and why you never power motors from the Pico."
layout: lesson
type: page
cover: /learn/electronics_basics/assets/banners/12_powering_your_projects.jpg
date_updated: 2026-11-21
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

## Overview

A robot with great code and the wrong power supply will reset, stutter or give strange readings. Power is the quiet reason behind many mysterious faults. In this lesson we cover the power choices you will meet, the number one rule about grounds, and the reason motors never run from the Pico.

## Batteries

- **AA alkaline**: 1.5 V each (1.6 V fresh, dropping as they drain). Easy, cheap, safe. Three give 4.5 V, four give 6 V.
- **AA NiMH rechargeable**: 1.2 V each. Four give 4.8 V, which is great for the Pico's VSYS pin.
- **LiPo / Li-ion**: 3.7 V nominal each (4.2 V full, 3.0 V empty). Light and powerful, but they need respect: never short them, never puncture or crush them, never discharge below about 3.0 V per cell, and charge them only with a proper LiPo charger. Use a protected cell or add a protection board.
- **9 V PP3**: tiny capacity, high resistance. It is usually a poor choice for robots.

Battery life is easy to estimate:

```python
# Run on your computer or the Pico
# Estimate battery life
capacity_mah = 2000        # typical AA alkaline
load_ma = 40               # Pico plus LED, roughly

hours = capacity_mah / load_ma
print(f"About {hours:.0f} hours ({hours / 24:.1f} days)")
```

Output:

```text
About 50 hours (2.1 days)
```

Real life is somewhat less, because capacity drops under heavier loads.

## Regulators

Your robot may have a 6 V battery but chips that want 3.3 V or 5 V. A **regulator** converts one to the other.

- **Linear regulator** (for example the 7805, AMS1117): simple, quiet, but it burns the extra voltage as heat. Heat is (Vin - Vout) x current.
- **Buck converter** (switching): much more efficient, a bit noisier. Cheap modules are widely available.

```python
# Run on your computer or the Pico
# How much heat does a linear regulator make?
vin, vout, amps = 9.0, 5.0, 0.2

watts = (vin - vout) * amps
print(f"Wasted as heat: {watts:.1f} W")
```

Output:

```text
Wasted as heat: 0.8 W
```

0.8 W is enough to make a small regulator too hot to touch, so a buck converter is better here.

## 3.3 V versus 5 V

The Pico's chip runs at **3.3 V**. Its GPIO pins are **not** 5 V tolerant: feeding 5 V into a pin can damage it. The Pico can be powered from:

- **USB**: 5 V arrives at the VBUS pin (physical pin 40), and the Pico makes its own 3.3 V.
- **VSYS** (physical pin 39): accepts roughly 1.8 V to 5.5 V, so three AA cells work nicely.
- **3V3(OUT)** (physical pin 36): a 3.3 V output for your circuits, up to about 300 mA shared between everything you hang on it.

## Common ground

If two supplies are connected to the same circuit, say the Pico on USB and a battery pack for the LED, their **grounds must be joined**. Voltage is a difference between two points, so signals need a common reference. Without it, the transistor's base has no defined voltage compared with its emitter, and nothing switches reliably.

Rule: separate positive supplies, shared ground.

## Why not power motors from the Pico?

- **Current**: a small motor draws 200 mA running and more than 1 A when stalled. The Pico's 3V3 pin gives up to about 300 mA and a GPIO pin only a few milliamps.
- **Noise**: motors create electrical spikes that can reset the Pico or corrupt ADC readings.
- **Brownouts**: if the motor takes a gulp of current, the voltage sags, and the Pico reboots.

Give motors their own battery, use a driver to switch them, and join the grounds. We cover this in the Motors and motor drivers course.

**Why this matters:** half of "my robot resets when the motors start" stories end here.

**Common mistakes:**

- Shorting battery terminals, even for a moment. Keep wires tidy, and fit a fuse on bigger packs.
- Joining a 5 V signal to a Pico pin.
- Forgetting the common ground.

## Try it Yourself

1. Work out how long your battery pack would last running a 100 mA load.
2. Measure the voltage of your battery pack with a multimeter under no load, then with the LED circuit connected.
3. Challenge: work out the heat from a 7805 running 12 V to 5 V at 300 mA. Would you add a heatsink?

## Common Issues

**Problem**: The Pico resets when something else switches on.
**Solution**: Give the big load its own supply, and add a capacitor (100 uF) across the Pico's power.
**Why**: The load pulls the shared voltage down for a moment.

**Problem**: A transistor does not switch when the load has its own supply.
**Solution**: Connect the two grounds together.
**Why**: The base voltage needs the same zero as the emitter.
