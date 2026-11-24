---
title: Driving two motors
description: Combine two Motor objects into a DriveBase that drives forward, backward, in arcs and spins on the spot.
layout: lesson
type: page
cover: /learn/motors_and_drivers/assets/banners/08_driving_two_motors.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to mix speed and steering into left and right wheel speeds, and drive Scout straight, round a curve and spin on the spot.

One motor is a test rig. Two motors on opposite sides of a chassis is a **differential drive**, the same design as a tank, a Roomba and most hobby robots, including Scout. There is no steering wheel. The robot turns by running its wheels at different speeds.

## How differential drive steers

| Left wheel | Right wheel | Scout does |
|---|---|---|
| Forward | Forward | Drives straight ahead |
| Reverse | Reverse | Drives straight back |
| Fast | Slow | Curves gently to the right |
| Forward | Reverse | Spins clockwise on the spot |
| Reverse | Forward | Spins anticlockwise on the spot |
| Stopped | Forward | Pivots around the stopped wheel |
{:class="table table-single"}

A spin on the spot is the party trick. Each wheel moves the same distance in opposite directions, so the robot's centre does not move.

## Mixing speed and steering

Instead of thinking about wheels, we think in two numbers: **speed** (how fast to go forward) and **steer** (how hard to turn). A tiny function called a mixer converts these to left and right wheel speeds:

```python
def clamp(value, low=-100, high=100):
    return max(low, min(high, value))


def mix(speed, steer):
    """Arcade-drive mixer. Positive steer turns right."""
    return clamp(speed + steer), clamp(speed - steer)


print(mix(60, 0))     # (60, 60)    straight ahead
print(mix(60, 20))    # (80, 40)    curve right
print(mix(0, 50))     # (50, -50)   spin right on the spot
print(mix(90, 30))    # (100, 60)   clamped at 100
```

The left wheel gets `speed + steer`, the right gets `speed - steer`. To turn right, the left wheel speeds up and the right slows down.

```mermaid
flowchart LR
    S["speed"] --> M["mix"]
    T["steer"] --> M
    M -->|"speed + steer"| L["left motor"]
    M -->|"speed - steer"| R["right motor"]
```

## drive.py

Now the `DriveBase` class. It holds two motor objects and remembers the last speeds, which we will use for ramping in the next lesson.

```python
# drive.py - a two-wheel differential drive base
from time import sleep_ms


def clamp(value, low=-100, high=100):
    return max(low, min(high, value))


def mix(speed, steer):
    """Arcade-drive mixer. Positive steer turns right."""
    return clamp(speed + steer), clamp(speed - steer)


class DriveBase:
    def __init__(self, left, right):
        self.left = left
        self.right = right
        self.left_speed = 0
        self.right_speed = 0

    def drive(self, left, right):
        """Set each wheel directly, -100 to 100."""
        self.left_speed = clamp(left)
        self.right_speed = clamp(right)
        self.left.speed(self.left_speed)
        self.right.speed(self.right_speed)

    def forward(self, speed=60):
        self.drive(speed, speed)

    def backward(self, speed=60):
        self.drive(-speed, -speed)

    def steer(self, speed, steer):
        self.drive(*mix(speed, steer))

    def spin_left(self, speed=60):
        self.drive(-speed, speed)

    def spin_right(self, speed=60):
        self.drive(speed, -speed)

    def stop(self):
        """Brake both wheels."""
        self.left_speed = 0
        self.right_speed = 0
        self.left.brake()
        self.right.brake()

    def coast(self):
        """Let both wheels roll freely."""
        self.left_speed = 0
        self.right_speed = 0
        self.left.coast()
        self.right.coast()

    def run_for(self, ms, left, right):
        """Drive for a set time, then brake."""
        self.drive(left, right)
        sleep_ms(ms)
        self.stop()
```

## Driving Scout around

Time for the real thing. Put Scout on blocks or hold it up the first time, so you can check each wheel does what you expect. This uses the pin layout from lesson 5, and each motor takes its own `deadzone` because no two motors are identical (start both at the number you found in lesson 6):

```python
from time import sleep_ms
from motor import Motor
from drive import DriveBase

left = Motor(0, 1, 2, deadzone=30)
right = Motor(3, 4, 5, deadzone=30)
base = DriveBase(left, right)

try:
    print("forward")
    base.forward(60)
    sleep_ms(1500)
    base.stop()
    sleep_ms(500)

    print("backward")
    base.backward(60)
    sleep_ms(1500)
    base.stop()
    sleep_ms(500)

    print("curve right")
    base.steer(60, 25)
    sleep_ms(1500)
    base.stop()
    sleep_ms(500)

    print("spin")
    base.spin_right(60)
    sleep_ms(1000)
    base.stop()
finally:
    base.coast()
```

The wheels should go the same way on "forward". If one is backwards, flip it with `invert=True` on that `Motor`. If Scout veers left or right when it should go straight, one motor is a little stronger than the other, which is normal. We will tune that out in the project.

## Try it Yourself

1. Adjust `invert` until Scout drives straight on "forward" and spins clockwise on "spin right".
2. Make Scout drive in a gentle figure of eight, using `steer` with +25, then -25.
3. Challenge: write a `pivot_left()` method that stops the left wheel and drives only the right.

## Common Issues

- **Problem**: Scout drives in a curve instead of a straight line.
- **Solution**: Try a slightly lower speed value on the faster wheel, for example `base.drive(60, 55)`, or give the faster motor a higher `deadzone`.
- **Why**: Real motors are never identical, and one will always be slightly quicker for the same duty.

- **Problem**: "Spin right" makes Scout spin left.
- **Solution**: Add `invert=True` to both motors, or swap which is passed as left and right.
- **Why**: Which direction is "forward" depends on how the motor wires and chassis are orientated.

- **Problem**: Scout stalls on spin even though it drives fine.
- **Solution**: Raise the spin speed. Spinning needs more torque to overcome wheel scrub.
- **Why**: Spinning drags the tyres sideways across the floor.

> **Teachers**: robots drive off table edges. Run the first tests with the chassis held up or on a block, and after that on the floor, not a desk. Give each group a clear patch of floor and ask everyone else to stay out of it while a robot is running. Only the group that owns the robot connects its battery.

## Check your understanding

{% quiz %}
? What does mix(60, 20) return?
- [x] (80, 40)
- [ ] (60, 20)
- [ ] (40, 80)
> The left wheel gets speed + steer = 80 and the right gets speed - steer = 40, so Scout curves right.

? How does a differential drive robot spin on the spot?
- [x] Wheels run at equal speeds in opposite directions
- [ ] Both wheels run forward
- [ ] One wheel stops
> Each wheel moves the same distance in opposite directions, so the centre does not move.

? Why might Scout curve when you ask for straight?
- [x] Real motors are never identical, so one is slightly quicker
- [ ] The battery is too full
- [ ] PWM is broken
> Tune it with different left and right speeds or a different deadzone for each motor.
{% endquiz %}
