---
title: Stepper motors
description: Drive a 28BYJ-48 stepper motor through a ULN2003 board with a half-step sequence and move it an exact number of degrees.
layout: lesson
type: page
cover: /learn/motors_and_drivers/assets/banners/13_stepper_motors.jpg
date_updated: 2026-11-21
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

DC motors spin, servos point, and steppers *count*. A stepper moves one tiny, precise step each time its coils are energised in the next pattern. No sensor, no feedback, just a sequence and a counter. This lesson is a side quest from Scout's drive base, but steppers are everywhere in maker projects, so it is worth meeting one.

This lesson is optional if you do not have a 28BYJ-48, so feel free to read it and skip the wiring.

## The 28BYJ-48 and ULN2003

The 28BYJ-48 is the cheap stepper bundled with a small **ULN2003 driver board**. Its four coils cannot be driven from GPIO directly (remember lesson 2), so the ULN2003 is a chip of seven transistor switches that handles the current. The board has four input pins, IN1 to IN4, each switching one coil, and four LEDs that light as each coil is energised.

| Pico pin | ULN2003 pin | Job |
|---|---|---|
| GP16 | IN1 | Coil 1 |
| GP17 | IN2 | Coil 2 |
| GP18 | IN3 | Coil 3 |
| GP19 | IN4 | Coil 4 |
| VBUS | + (power) | 5 V motor supply |
| GND | - (power) | Common ground |
{:class="table table-single"}

Check the label on your motor: the common one is 5 V. Power it from VBUS (5 V from USB) or a separate 5 V supply, and never from the 3V3 pin. It draws up to a few hundred milliamps, so keep it off the Pico's regulators.

## The step sequence

To turn the shaft, energise the coils one after another. The simplest **full-step** pattern uses two coils at a time. The smoother **half-step** pattern alternates between one coil and two, doubling the number of steps per turn:

```python
# Each tuple is (IN1, IN2, IN3, IN4)
HALF_STEP = [
    (1, 0, 0, 0),
    (1, 1, 0, 0),
    (0, 1, 0, 0),
    (0, 1, 1, 0),
    (0, 0, 1, 0),
    (0, 0, 1, 1),
    (0, 0, 0, 1),
    (1, 0, 0, 1),
]

FULL_STEP = [
    (1, 1, 0, 0),
    (0, 1, 1, 0),
    (0, 0, 1, 1),
    (1, 0, 0, 1),
]
```

Run through the list in order for one direction, and backwards for the other. The motor's internal gears (about 64:1) mean the output shaft turns slowly with plenty of torque.

## How many steps is a turn?

The motor has 512 cycles of the sequence per output revolution, so the number of steps depends on how many entries your sequence has:

```python
def degrees_to_steps(degrees, steps_per_rev):
    return round(degrees * steps_per_rev / 360)


half = 8 * 512                  # 8 entries x 512 cycles = 4096 half-steps per revolution
full = 4 * 512                  # 4 x 512 = 2048 full steps per revolution

print(degrees_to_steps(360, half))   # 4096
print(degrees_to_steps(90, half))    # 1024
print(degrees_to_steps(1, half))     # 11
print(degrees_to_steps(90, full))    # 512
```

The real gear ratio is about 63.68:1, not exactly 64, so one real revolution is about 4076 half-steps rather than 4096. Using 4096 means it creeps about 1.8 degrees extra per turn, which is fine for a camera pan, but not for a plotter.

## stepper.py

```python
# stepper.py - 28BYJ-48 via ULN2003
from machine import Pin
from time import sleep_ms

HALF_STEP = [
    (1, 0, 0, 0), (1, 1, 0, 0), (0, 1, 0, 0), (0, 1, 1, 0),
    (0, 0, 1, 0), (0, 0, 1, 1), (0, 0, 0, 1), (1, 0, 0, 1),
]


def degrees_to_steps(degrees, steps_per_rev):
    return round(degrees * steps_per_rev / 360)


class Stepper:
    def __init__(self, pins=(16, 17, 18, 19), sequence=HALF_STEP):
        self.coils = [Pin(p, Pin.OUT) for p in pins]
        self.sequence = sequence
        self.steps_per_rev = len(sequence) * 512
        self.index = 0
        self.release()

    def _apply(self, pattern):
        for coil, level in zip(self.coils, pattern):
            coil.value(level)

    def step(self, count, delay_ms=2):
        """Take `count` steps. Negative counts go the other way."""
        direction = 1 if count > 0 else -1
        for _ in range(abs(count)):
            self.index = (self.index + direction) % len(self.sequence)
            self._apply(self.sequence[self.index])
            sleep_ms(delay_ms)

    def rotate(self, degrees, delay_ms=2):
        self.step(degrees_to_steps(degrees, self.steps_per_rev), delay_ms)

    def release(self):
        """De-energise every coil so the motor stays cool."""
        for coil in self.coils:
            coil.value(0)
```

## Try it

```python
from time import sleep_ms
from stepper import Stepper

motor = Stepper()

try:
    motor.rotate(90)        # a quarter turn
    sleep_ms(500)
    motor.rotate(-90)       # and back again
    sleep_ms(500)
    motor.rotate(360)       # one full revolution
finally:
    motor.release()
```

The `delay_ms` between steps sets the speed. At 2 ms per half-step, one revolution takes 4096 x 2 ms = 8.2 seconds, about 7 revolutions per minute. Try 1 ms and it may skip steps or just buzz, as the rotor cannot keep up. That is the cost of a stepper: it is accurate, but it is slow.

**Could Scout use one?** A stepper could swing the head instead of a servo, and the same technique with a bigger stepper and driver powers 3D printers. For Scout's wheels, though, DC motors are faster, stronger and cheaper.

## Try it Yourself

1. Rotate through exactly 45 degrees and back. Does it return to where it started?
2. Switch to a full-step sequence by passing `sequence=FULL_STEP` (copy it from above). What changes in the sound and the speed?
3. Challenge: make the stepper sweep back and forth by 90 degrees repeatedly, like a head scan.

## Common Issues

**Problem**: The motor hums and vibrates but does not turn.
**Solution**: Check the wiring order of IN1 to IN4 against GP16 to GP19, and slow the `delay_ms` to 3 or 4.
**Why**: The coil sequence has to be in order, and going too fast makes the rotor miss the magnetic field.

**Problem**: The motor and the driver chip get warm when idle.
**Solution**: Call `release()` after a move.
**Why**: A stepper that is holding position keeps current flowing through a coil all the time.

**Problem**: It turns the wrong way.
**Solution**: Pass a negative number, or reverse the pin list to `(19, 18, 17, 16)`.
**Why**: Direction is just the order in which the coils fire.
