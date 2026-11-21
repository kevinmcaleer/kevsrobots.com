---
title: Ramping and braking
description: Start and stop smoothly with a speed ramp, and choose between coasting and braking to stop where you mean to.
layout: lesson
type: page
cover: /learn/motors_and_drivers/assets/banners/09_ramping_and_braking.jpg
date_updated: 2026-11-21
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

If you have jumped Scout straight to speed 100, you will have seen it lurch, maybe lift a wheel, and possibly reset the Pico. Real vehicles accelerate gradually. This lesson adds that to Scout, then looks at the two ways to stop.

## Why ramp?

When a motor starts, it draws a big surge of current, and a sudden jump to full speed also makes the wheels slip on smooth floors. Slipping means your robot goes less far than you expect, which will ruin a square later. A **ramp** moves the speed up in small steps instead.

The step list comes from a plain function:

```python
def ramp_values(start, end, steps):
    """The speeds to visit when moving from start to end in `steps` steps."""
    return [int(round(start + (end - start) * i / steps)) for i in range(1, steps + 1)]


print(ramp_values(0, 100, 5))     # [20, 40, 60, 80, 100]
print(ramp_values(60, -60, 4))    # [30, 0, -30, -60]
print(ramp_values(50, 50, 3))     # [50, 50, 50]
```

Each value is the speed for one tiny time slice. Visit them 20 ms apart and the speed changes smoothly.

## ramp.py

Create `ramp.py`. It uses the `left_speed` and `right_speed` that `DriveBase` already remembers:

```python
# ramp.py - smooth speed changes for a DriveBase
from time import sleep_ms


def ramp_values(start, end, steps):
    """The speeds to visit when moving from start to end in `steps` steps."""
    return [int(round(start + (end - start) * i / steps)) for i in range(1, steps + 1)]


def ramp_to(base, left, right, ms=400, step_ms=20):
    """Smoothly change both wheel speeds over `ms` milliseconds."""
    steps = max(1, ms // step_ms)
    lefts = ramp_values(base.left_speed, left, steps)
    rights = ramp_values(base.right_speed, right, steps)
    for l, r in zip(lefts, rights):
        base.drive(l, r)
        sleep_ms(step_ms)
```

A 400 ms ramp in 20 ms steps takes 20 steps, so every step changes the speed by only a few percent. Ramp up from rest, hold, then ramp down:

```python
from time import sleep_ms
from motor import Motor
from drive import DriveBase
from ramp import ramp_to

left = Motor(0, 1, 2, deadzone=30)
right = Motor(3, 4, 5, deadzone=30)
base = DriveBase(left, right)

try:
    ramp_to(base, 70, 70, ms=600)    # ease up to speed
    sleep_ms(1000)
    ramp_to(base, 0, 0, ms=600)      # ease back down
    base.stop()
finally:
    base.coast()
```

The same function turns Scout smoothly, for example `ramp_to(base, 60, -60)` goes from a stop into a spin, and from forward straight through zero into reverse without a jolt.

## Coast versus brake

Remember lesson 4: coast disconnects the motor, brake shorts it. Which one should Scout use? Try this side by side. Push Scout to a steady speed, then stop each way and measure how far it rolls:

```python
from time import sleep_ms
from motor import Motor
from drive import DriveBase
from ramp import ramp_to

left = Motor(0, 1, 2, deadzone=30)
right = Motor(3, 4, 5, deadzone=30)
base = DriveBase(left, right)

try:
    ramp_to(base, 70, 70, ms=500)
    sleep_ms(800)
    print("coasting")
    base.coast()           # let it roll
    sleep_ms(3000)

    ramp_to(base, 70, 70, ms=500)
    sleep_ms(800)
    print("braking")
    base.stop()            # short the motors
    sleep_ms(3000)
finally:
    base.coast()
```

Mark the floor where each stop finishes. Typically:

| Stop style | Result | Best for |
|---|---|---|
| Coast | Rolls on for a while, kind to the battery | Cruising to a stop, saving power |
| Brake | Stops within a centimetre or two | Precise moves, stopping at the corners of a square |
| Ramp down, then brake | Smooth and precise | Anything with a load or a camera |
{:class="table table-single"}

**Brake safety note:** holding a brake against a strong push is fine for short moments, but brake current flows through the motor and driver. Do not hold a brake with the wheels forced to turn for a long time, and let the driver cool if it feels hot.

## Try it Yourself

1. Change the ramp time from 600 ms to 100, then to 2000. What does each feel like?
2. Add a coast stop *after* a ramp down. Does it look different from braking?
3. Challenge: make an `ease_in_out` version of `ramp_values` that starts and ends slowly, using a smoothstep curve. (Hint: map `t` from 0 to 1 to `3 * t * t - 2 * t * t * t`.)

## Common Issues

**Problem**: The ramp appears to do nothing, then jumps.
**Solution**: Make sure your ramp passes through your dead zone gently. Use a smaller `deadzone` or a longer ramp.
**Why**: Below the dead zone nothing moves, so the visible motion all happens in the top part of the ramp.

**Problem**: Scout still rolls after `stop()`.
**Solution**: Check you are calling `stop()` (brake) and not `coast()`, and that the EN pin is being driven high by the brake.
**Why**: The L298N only brakes when its enable pin is high.
