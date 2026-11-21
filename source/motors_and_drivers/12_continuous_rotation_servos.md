---
title: Continuous rotation servos
description: Turn a hobby servo into a geared wheel motor with pulse-width speed control, and learn when it beats a DC motor and driver.
layout: lesson
type: page
cover: /learn/motors_and_drivers/assets/banners/12_continuous_rotation_servos.jpg
date_updated: 2026-11-21
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Here is a clever hybrid. Take a servo, remove its position sensor's travel stop, and what comes out is a **continuous rotation servo**. It has a gearbox, a built-in driver and a single signal wire, but instead of an angle, the pulse width sets *speed and direction*.

## The meaning of the pulse

The same 50 Hz pulse, with a new meaning:

| Pulse width | What the servo does |
|---|---|
| About 1500 us | Stops |
| Longer than 1500 us | Spins one way, faster as it gets longer |
| Shorter than 1500 us | Spins the other way |
{:class="table table-single"}

With a typical 1000 to 2000 us range, 1000 is full speed one way and 2000 is full speed the other. We use the speed scale from the DC motor lessons, -100 to 100:

```python
def speed_to_pulse(speed, stop_us=1500, span_us=500):
    """Map speed -100..100 to a pulse width around the stop point."""
    speed = max(-100, min(100, speed))
    return stop_us + span_us * speed / 100


print(speed_to_pulse(0))      # 1500.0  stopped
print(speed_to_pulse(100))    # 2000.0  full speed one way
print(speed_to_pulse(-100))   # 1000.0  full speed the other way
print(speed_to_pulse(40))     # 1700.0  gentle
```

## A ContinuousServo class

It has the same `speed()` method as `Motor`, so it fits into a `DriveBase` too. Save this as `cr_servo.py`:

```python
# cr_servo.py - continuous rotation servo with the same interface as Motor
from machine import Pin, PWM
from servo import pulse_to_duty


def speed_to_pulse(speed, stop_us=1500, span_us=500):
    speed = max(-100, min(100, speed))
    return stop_us + span_us * speed / 100


class ContinuousServo:
    def __init__(self, pin, stop_us=1500, span_us=500, invert=False, freq=50):
        self.pwm = PWM(Pin(pin))
        self.pwm.freq(freq)
        self.freq = freq
        self.stop_us = stop_us      # trim this until the wheel is still
        self.span_us = span_us
        self.invert = invert
        self.coast()

    def speed(self, value):
        if self.invert:
            value = -value
        us = speed_to_pulse(value, self.stop_us, self.span_us)
        self.pwm.duty_u16(pulse_to_duty(us, self.freq))

    def brake(self):
        self.speed(0)           # a servo holds still when told to stop

    def coast(self):
        self.pwm.duty_u16(0)    # no pulses: the motor is released
```

It reuses `pulse_to_duty` from your `servo.py`, so keep both files on the Pico.

## Trimming the stop point

Run it, and the wheel will probably creep even at speed 0, because the true stop is rarely exactly 1500 us. Nudge `stop_us` until it is still:

```python
from time import sleep_ms
from cr_servo import ContinuousServo

wheel = ContinuousServo(10)

try:
    wheel.speed(0)
    print("is it still? try stop_us values in the next step")
    sleep_ms(2000)

    for speed in (30, 60, 100, 0, -30, -60, -100, 0):
        print("speed", speed)
        wheel.speed(speed)
        sleep_ms(1500)
finally:
    wheel.coast()
```

If the wheel creeps one way at speed 0, create it with, say, `ContinuousServo(10, stop_us=1480)` and try again. Many continuous servos have a tiny trimmer screw to do this mechanically, adjusted through a hole in the case.

## DC motor or continuous servo?

| | DC motor and driver | Continuous servo |
|---|---|---|
| Parts | Motor plus driver board | One part, gearbox built in |
| Pico pins | 2 or 3 per motor | 1 per motor |
| Power | High, strong torque | Modest, plastic gears |
| Speed control | Fine, with PWM | Coarse, with a narrow useful range |
| Good for | Scout, and most robots | Tiny, simple robots and classroom kits |
{:class="table table-single"}

Continuous servos are a great way to get a first robot rolling, but Scout uses DC motors for the strength and the finer speed control.

## Try it Yourself

1. If you have a continuous servo, find its stop point by trial and error and write down the `stop_us` value.
2. Check which speeds actually change the wheel speed. Does 5 move it? Does 90 look different from 100?
3. Challenge: build a `DriveBase` out of two `ContinuousServo` objects (one with `invert=True`, since they face opposite ways) and run the lesson 8 test.

## Common Issues

**Problem**: The wheel never fully stops.
**Solution**: Adjust `stop_us` by 10 us at a time, or use the trimmer on the servo.
**Why**: The factory centre point is only approximately 1500 us.

**Problem**: One wheel is faster than the other at the same speed.
**Solution**: Use `span_us` to give the quicker one a smaller range.
**Why**: Cheap servos vary in their gearing and electronics.
