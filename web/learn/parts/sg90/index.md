---
title: SG90 Micro Servo
description: >-
  The SG90 is a tiny, cheap 9 g hobby servo with around 180° of travel,
  controlled by a 50 Hz PWM signal. Here's how to wire one to a Raspberry Pi
  Pico and drive it from MicroPython.
layout: content
---

{% include breadcrumbs.html %}

# SG90 Micro Servo

The `SG90` is the classic 9 g hobby servo — small, light, cheap, and
everywhere. It gives you around 180° of controlled rotation from a single PWM
signal, which makes it perfect for pan/tilt rigs, robot arms, grippers,
walking robots and RC projects.

You can learn more about [how servos work here](/resources/how_it_works/servos.html).

---

## Key specs

| Spec | Value |
|------|-------|
| Weight | ~9 g |
| Torque | ~1.8 kg·cm @ 4.8 V |
| Speed | ~0.1 s / 60° |
| Rotation | ~180° |
| Operating voltage | 4.8–6 V |
| Control signal | 50 Hz PWM, ~1–2 ms pulse width |

---

## Wiring

The SG90 has three flying leads:

| Wire | Signal | Connect to |
|------|--------|-----------|
| **Orange** | Signal (PWM) | any GPIO (e.g. Pico **GP16**) |
| **Red** | VCC (+5 V) | a **5 V** supply (4.8–6 V) |
| **Brown** | GND | GND — **shared** with your supply |

> ⚠️ Don't power a servo from the Pico's **3V3** pin — servos draw big current
> spikes under load and will brown the board out. Use the **VBUS** pin (5 V
> from USB) or a separate 4.8–6 V supply, and make sure the grounds are
> connected together.

---

## How the control signal works

The servo expects a **50 Hz** signal — one pulse every 20 ms. The **width** of
that pulse sets the angle:

- **1.0 ms** → 0°
- **1.5 ms** → 90° (centre)
- **2.0 ms** → 180°

Many SG90s accept a wider **0.5–2.5 ms** range for a fuller sweep — if your
servo doesn't reach its end stops, widen the pulse range a little at a time.

---

## MicroPython example

Using just the built-in `machine` module — signal wire on **GP16**:

```python
from machine import Pin, PWM
import time

servo = PWM(Pin(16))
servo.freq(50)                      # 50 Hz — one pulse every 20 ms

def angle(degrees):
    # 0° = 500 µs, 180° = 2500 µs
    pulse_us = 500 + (degrees / 180) * 2000
    servo.duty_ns(int(pulse_us * 1000))

angle(90)                           # centre
time.sleep(1)
angle(0)                            # one end
time.sleep(1)
angle(180)                          # the other end
```

If the servo buzzes when it's holding position at an end stop, stop sending
pulses with `servo.deinit()` — it releases the holding torque and the buzzing
stops.

---

## Dimensions

![Dimensioned drawing of the SG90 micro servo](/assets/img/design/sg90.png){:class="img-fluid" loading="lazy"}

The body is roughly **23 × 12 × 29 mm** with two mounting tabs (32.2 mm across
the tabs), and the output shaft takes the usual push-fit horns.

---

## Troubleshooting

- **Jitter / twitching** → shared ground plus a dedicated 5 V supply (not 3V3).
- **Buzzing at rest** → `servo.deinit()` once it reaches the target.
- **Limited range** → widen the pulse range (try 500–2500 µs) a little at a time.

---

## Use it in Snakie

The SG90 is in the Standard parts library of [Snakie](https://www.snakie.org),
my free, open-source MicroPython editor. Drag it onto the Board View and Snakie
shows you the wiring, copies a ready-made `servo` driver to your board, and the
Servo instrument gives you a dial you can drag to move the real servo live.

---

## Related learning

- [Servos — GPIO Mastery course](/learn/micropython_gpio/14_servos.html)
- [How servos work](/resources/how_it_works/servos.html)
- [Robotics with MicroPython](/learn/micropython_robotics/)
