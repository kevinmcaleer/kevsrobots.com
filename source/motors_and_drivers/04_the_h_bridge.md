---
title: The H-bridge
description: See how four switches let a driver spin a DC motor in either direction, brake it, or let it coast.
layout: lesson
type: page
cover: /learn/motors_and_drivers/assets/banners/04_the_h_bridge.jpg
date_updated: 2026-11-21
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

A DC motor reverses when you swap its two wires. A robot cannot swap wires by hand, so we build a circuit that does it electronically. That circuit is the **H-bridge**, and it lives inside every motor driver you will use.

## Four switches in an H

Draw the motor in the middle, and a switch at each corner:

```text
        +V (motor supply)
         |            |
       [S1]         [S2]
         |            |
         +-----(M)----+
         |            |
       [S3]         [S4]
         |            |
        GND          GND
```

Real drivers use transistors for the switches, but think of them as little taps. Which ones you close decides what the motor does:

| S1 | S2 | S3 | S4 | Motor |
|---|---|---|---|---|
| on | off | off | on | Current flows left to right: spins forward |
| off | on | on | off | Current flows right to left: spins in reverse |
| off | off | on | on | Both ends tied to ground: brakes |
| on | on | off | off | Both ends tied to +V: brakes |
| off | off | off | off | Nothing connected: coasts |
{:class="table table-single"}

Never close S1 and S3 together (or S2 and S4). That connects the supply straight to ground, which is called **shoot-through**, and it destroys the transistors. Driver chips like the L298N and DRV8833 have logic that makes this impossible, which is a big reason we use them.

## From four switches to two inputs

The driver turns those four switches into two pins per motor, **IN1** and **IN2**. The L298N adds a third, **EN** (enable), which switches the whole bridge on or off. This is the truth table you will code against:

```python
# What does one L298N channel do for each input combination?
def bridge_state(in1, in2, en):
    if not en:
        return "coast (bridge disabled)"
    if in1 and not in2:
        return "forward"
    if in2 and not in1:
        return "reverse"
    return "brake (both inputs the same)"

for en in (0, 1):
    for in1 in (0, 1):
        for in2 in (0, 1):
            print("EN", en, "IN1", in1, "IN2", in2, "->", bridge_state(in1, in2, en))
```

Run it in Thonny and you will see all eight combinations. Keep this in your head, because everything we do for the next five lessons is just choosing these three values.

## Coast versus brake

- **Coast**: the motor is disconnected. The robot keeps rolling until friction stops it.
- **Brake**: the motor terminals are shorted together through the bridge. The motor's own back-EMF now fights its motion, so it stops quickly.

Scout will use brake when it needs to stop precisely at the corner of a square, and coast when we do not care. Lesson 9 tries both side by side.

## The DRV8833 does it differently

The DRV8833 has no EN pin. It has two inputs per motor, and you vary the PWM on those. The truth table collapses to:

| IN1 | IN2 | Motor |
|---|---|---|
| PWM | low | Forward at that speed |
| low | PWM | Reverse at that speed |
| high | high | Brake |
| low | low | Coast |
{:class="table table-single"}

Same H-bridge, different front door.

## Try it Yourself

1. Change `bridge_state` so it prints "stopped" when both inputs are low and EN is high. Then check against the real driver: does the motor free-wheel (coast) or resist being turned (brake)? What does that tell you about the truth table?
2. Write out, by hand, which of S1 to S4 are closed in each of the five rows above for a bridge turning the motor forward.
3. Challenge: a robot rolls down a slope. Would you expect coast or brake to be better at holding it still? Why?

## Common Issues

**Problem**: My motor does not stop when I set both inputs low.
**Solution**: Remember that with the L298N, both inputs equal and EN high is a brake, while EN low is a coast. Check what your EN pin is doing.
**Why**: The enable pin decides whether the bridge is connected at all. The inputs only decide *how* it is connected.
