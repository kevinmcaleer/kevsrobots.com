---
layout: lesson
title: Move with the Motors
author: Kevin McAleer
type: page
cover: /learn/robot_senses/assets/banners/09_move_the_motors.jpg
date: 2026-11-24
previous: 08_show_the_led_matrix.html
next: 10_readings_to_decisions.html
description: Drive SMARS with the Modulino Motors, work out the inverts and wiring,
  wire the 9V supply safely and read motor current
percent: 50
duration: 5
quiz_total: 10
date_updated: 2026-11-24
navigation:
- name: Give Your Robot Senses
- content:
  - section: Introduction
    content:
    - name: Introduction
      link: 00_intro.html
  - section: How robots sense
    content:
    - name: How Robots Sense
      link: 01_how_robots_sense.html
    - name: Meet Modulino SMARS
      link: 02_meet_modulino_smars.html
    - name: Qwiic and I2C
      link: 03_qwiic_and_i2c.html
    - name: Your First Connection
      link: 04_first_connection.html
  - section: One sense at a time
    content:
    - name: See with the Distance Module
      link: 05_see_distance.html
    - name: Feel Movement with the IMU
      link: 06_feel_movement.html
    - name: Speak with the Buzzer
      link: 07_speak_the_buzzer.html
    - name: Show with the LED Matrix
      link: 08_show_the_led_matrix.html
    - name: Move with the Motors
      link: 09_move_the_motors.html
  - section: From readings to behaviour
    content:
    - name: From Readings to Decisions
      link: 10_readings_to_decisions.html
    - name: Combining Senses
      link: 11_combining_senses.html
    - name: 'Project: The Reactive Robot'
      link: 12_project_reactive_robot.html
  - section: Navigation
    content:
    - name: Wall Following
      link: 13_wall_following.html
    - name: From Wall Following to Mapping and SLAM
      link: 14_mapping_and_slam.html
  - section: Wrapping up
    content:
    - name: Troubleshooting
      link: 15_troubleshooting.html
    - name: Summary
      link: 16_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

Sense five is movement, the one that makes the robot a robot. The **Modulino Motors** module drives SMARS's two stock N20 motors, and it also feels how hard they are working.

## Power first

The motor supply is the one connection on this robot that isn't a plug. The 9V battery connects to the `VIN` and `GND` screw terminals on the Modulino Motors. The module's motor supply wants 5 to 24 V, and 9 V is well inside that window. The logic still comes down the Qwiic cable at 3.3 V.

> Never power the motors from the Qwiic 3.3 V line. It can't supply the current, and a stalled motor could pull several amps through a cable that was never meant for it.

Check the wiring twice, then check it again with the battery disconnected. A wrong wire here can damage the module.

## The Modulino Motors module

It's built around a MAX22211 dual H-bridge, delivering up to 3.8 A per channel. Kev's code talks to it with four things:

- `speed_a` and `speed_b` set the speed of each motor, from 0 to 100 percent.
- `invert_a` and `invert_b` flip each motor's direction.
- `stop()` brakes. `release()` lets the motors coast.
- `sensed_current` reports the current each channel is drawing, in mA.

Notice that speeds are unsigned, 0 to 100. Direction comes from the invert flags, not from a negative speed.

## Real-world robot example

SMARS has a motor on each side, facing opposite ways. So one side has to be inverted for the robot to drive straight. On Kev's robot, motor A is the left side, motor B is the right, and the right one is inverted. This complete program drives forward for a second, then stops.

```python
from time import sleep_ms
from machine import I2C, Pin
from modulino import ModulinoMotors

i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400_000)
motors = ModulinoMotors(i2c_bus=i2c)

# Kev's values. Flip these if your robot spins instead of driving straight.
INVERT_LEFT = False
INVERT_RIGHT = True
CRUISE_SPEED = 60

print("Driving in 3 seconds. Hold the robot with its tracks free.")
sleep_ms(3000)

try:
    motors.invert_a = INVERT_LEFT
    motors.invert_b = INVERT_RIGHT
    motors.speed_a = CRUISE_SPEED
    motors.speed_b = CRUISE_SPEED
    sleep_ms(1000)
    print("Current draw (A, B):", motors.sensed_current)
    motors.stop()                 # brake
finally:
    motors.release()              # coast, kinder on the gearboxes
```
{: .no-snakie }

The 3-second wait is deliberate. A robot that drives off the bench the moment you plug it in is a robot you only own once.

**Common mistakes:**

- The robot spins instead of going forward. Flip `INVERT_RIGHT`.
- It drives backwards. Flip both inverts.
- Left and right are the wrong way round. Swap which motor you call left.
- It doesn't move on tracks. Raise the speed, because tracks have more friction than wheels.

Kev's repository has a `motor_setup.py` wizard that tries one motor at a time and works out the settings from your answers. If you'd rather do it by hand, the flips above cover the combinations.

## Steering with two numbers

To turn, run one side faster than the other, or run them in opposite directions. Here is a helper that takes a speed for each side from -100 to 100 and works out the two inverts and speeds. It's complete and runs anywhere, so you can test it without a robot:

```python
INVERT_LEFT = False
INVERT_RIGHT = True

def drive(left, right):
    """Each side is -100 (full reverse) to 100 (full forward)."""
    # A side is inverted if its base wiring needs it, or if we want reverse
    invert_a = INVERT_LEFT != (left < 0)
    invert_b = INVERT_RIGHT != (right < 0)
    speed_a = min(100, abs(int(left)))
    speed_b = min(100, abs(int(right)))
    return invert_a, speed_a, invert_b, speed_b

for name, l, r in (("forward", 60, 60), ("backward", -60, -60),
                   ("turn right", 55, -55), ("turn left", -55, 55)):
    print(name, drive(l, r))
```

The `!=` between two booleans acts like an exclusive or: the result is true if exactly one of them is true. That's how a motor that needs inverting for wiring gets un-inverted again when you ask for reverse. On the robot, you'd set `motors.invert_a`, `speed_a` and the rest from this result.

## Reading current

`sensed_current` gives you a tuple of two values in mA, one per channel. A jammed track draws hard. A dead motor draws almost nothing. It lets the robot know something is wrong without any extra sensor.

## Try it Yourself

1. Run the program and watch the direction. Fix it with the invert flags.
2. Change `CRUISE_SPEED` to 40 and then 80. Which speeds does your robot need on its tracks?
3. Challenge: after driving, print `sensed_current` while you hold one track still with your finger. What changes?

## Common Issues

**Problem**: The robot drives off the bench when powered up.
**Solution**: Keep the 3-second countdown, and test with the tracks off the ground first.
**Why**: Code in `main.py` runs as soon as the board powers on.

**Problem**: Motors run but the robot barely moves.
**Solution**: Check the 9V battery is connected to `VIN` and `GND`, and raise the speed.
**Why**: The motors have their own supply. Without it, only the logic is alive.

**Problem**: One motor doesn't turn.
**Solution**: Reseat the motor wires in the screw terminals and check `sensed_current`.
**Why**: A loose wire reads near zero. A jam reads high.
