---
title: "The Breadboard"
description: "Learn how a solderless breadboard is wired inside, and set up a clean power layout for the night light."
layout: lesson
type: page
cover: /learn/electronics_basics/assets/banners/03_the_breadboard.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to say which breadboard holes are connected, and set up a clean power layout for the night light.

## Overview

A breadboard lets you build circuits without soldering. Push parts and jumper wires into the holes, and if you make a mistake, pull them out and try again. It is the best friend of every robot builder. In this lesson we learn what is hidden inside the board, so a circuit never fails because of a connection you thought existed but did not.

## What is inside

A breadboard is a plastic block with rows of little metal clips under the holes. The clips join certain holes together.

- **Terminal strips** (the main area): each row of 5 holes, labelled a to e on one side and f to j on the other, is joined across. Holes a1, b1, c1, d1 and e1 are all connected to each other. The gap down the middle separates the two halves, so f1 is *not* connected to e1.
- **Power rails** (the long strips along the edges, marked red and blue or + and -): each rail is joined along its whole length. Use the red one for positive and the blue one for negative (ground).

Careful: on some full size boards the rails are split in the middle, so the top half and the bottom half are not connected. If your parts at the far end have no power, bridge the gap with a jumper wire. Check with your multimeter, which we meet in lesson 5.

Here is what is joined and what is not:

| Holes | Connected? | Why |
|---|---|---|
| a10, b10, c10, d10, e10 | Yes | One row of five, joined by a clip underneath |
| e10 and f10 | No | The central gap splits the board in two |
| a10 and a11 | No | Different rows are separate |
| Any hole in the red rail | Yes, along the rail | A rail is one long strip (check for a split in the middle) |
| Red rail and blue rail | No | They are separate strips |
{:class="table table-single"}

## Real example: a clean power layout

Let's set up the board we will use for the whole course.

1. Connect the battery pack's red lead to the top **red rail** and the black lead to the top **blue rail**. If your battery has bare wire ends, use a spare jumper or a small screw terminal adapter.
2. Keep the Pico away from the rails for now. We will add it in lesson 13.
3. Build circuits in the middle area, with components straddling the central gap if they have two rows of legs (like ICs).
4. Use short wires and keep them neat. A tidy board is easier to debug.

Colour discipline helps enormously:

- Red wires: positive supply
- Black or blue wires: ground
- Other colours: signals

That is not a rule of nature, but if you stick to it, a quick glance tells you whether something is connected wrong.

## Placing parts

- Resistors can go in either direction. Bend the legs and push them in so they span the row you need.
- LEDs have a **long leg (anode, positive)** and a **short leg (cathode, negative)**. They only work one way round. A flat edge on the LED rim also marks the cathode.
- Jumpers: push them in firmly. A half-inserted wire is the most common cause of a "dead" circuit.

## A quick connectivity test in code

Not sure which holes are joined? Here is a sneaky way to check once you have the Pico: connect a jumper from GP14 to a hole, another from GND to a different hole, then see whether the Pico sees a connection. Do not worry if this is a little early, we cover pins fully in the later lessons. For now, the continuity beeper on the multimeter in lesson 6 does the same job better.

```python
# MicroPython on the Pico
# Quick-and-dirty continuity check using GP14 with the internal pull-up
from machine import Pin
from time import sleep

probe = Pin(14, Pin.IN, Pin.PULL_UP)   # reads 1 unless something pulls it to GND

while True:
    connected = probe.value() == 0
    print("Connected to ground" if connected else "Open (not connected)")
    sleep(0.5)
```

Touch a wire from GND to the probe wire's hole. If the holes are joined, you will see "Connected to ground".

**Why this matters:** most "my circuit does not work" moments come from a hole that is not where you thought it was. Knowing the internal wiring turns guessing into checking.

**Common mistakes:**

- Putting both legs of a resistor in the *same* row. That shorts it out, as if it was not there.
- Using the wrong side of the central gap.
- Assuming the rails run all the way around.

> **Teachers**: breadboards from class sets often have worn clips and bent jumper ends, which cause the most "it does not work" calls. Check kits before the lesson and keep a few spare boards. Remind learners to leave the battery pack disconnected while they build, and to use short wires so a short circuit across the rails is easy to spot.

## Try it Yourself

1. Place an LED and a resistor so they share a row, then another pair on a different row. Sketch which holes are connected.
2. Test whether your power rails are split in the middle, by checking a jumper from the top of the red rail to the bottom.
3. Challenge: build the layout with the battery connected to the rails, with a red and a black jumper bringing power to the working area.

## Common Issues

- **Problem**: A part does nothing even though everything looks plugged in.
- **Solution**: Wiggle each leg, and check it sits in the right row.
- **Why**: Clips can be worn out, and legs can sit between rows. Move the part to a fresh set of holes.

- **Problem**: The circuit works on one side of the board but not the other.
- **Solution**: Check for a split power rail and bridge it with a jumper.
- **Why**: Many boards break the rail in the middle.

## Check your understanding

{% quiz %}
? Which holes are joined together?
- [ ] e5 and f5
- [ ] a5 and a6
- [x] a5, b5, c5, d5 and e5
> A row of five holes is joined across, but the central gap separates e from f, and neighbouring rows are separate.

? Half your breadboard has no power, but the other half is fine. What do you check first?
- [ ] Whether the resistor is too big
- [x] Whether the power rail is split in the middle
- [ ] Whether the LED is the right colour
> Some full size boards break the rails in the middle. Bridge the gap with a jumper.

? Which LED leg is the anode (positive)?
- [ ] The short leg
- [x] The long leg
- [ ] Either, it does not matter
> The long leg is positive. The short leg and the flat edge on the rim mark the cathode (negative).
{% endquiz %}
