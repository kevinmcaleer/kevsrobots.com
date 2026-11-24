---
layout: lesson
title: Feel Movement with the IMU
author: Kevin McAleer
type: page
cover: /learn/robot_senses/assets/banners/06_feel_movement.jpg
date: 2026-11-24
previous: 05_see_distance.html
next: 07_speak_the_buzzer.html
description: Read acceleration and rotation from the Modulino Movement module, work
  out which way the robot is tilted and why a gyroscope needs integrating
percent: 35
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

Sense two is balance. The **Modulino Movement** module lets the robot feel which way is down, and how fast it is turning. It's how SMARS knows when you've picked it up.

## What it measures

The module holds an **LSM6DSOX six-axis IMU**. IMU means inertial measurement unit. It has two sensors in one:

- An **accelerometer**, which reports acceleration along three axes, X, Y and Z, in units of **g**. One g is the pull of gravity.
- A **gyroscope**, which reports how fast the robot is rotating around those axes, in **degrees per second**.

When the robot sits still on the bench, the only acceleration is gravity. So one axis reads close to 1.0 g and the other two read close to zero. That is a handy sanity check that the module is working before you trust anything else it says.

## Real-world robot example

This is a complete program. It prints what the module feels. Pick the robot up and tip it.

```python
from time import sleep_ms
from machine import I2C, Pin
from modulino import ModulinoMovement

i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400_000)
movement = ModulinoMovement(i2c_bus=i2c)

print("Tilt the robot. Ctrl-C to stop.")
while True:
    a = movement.acceleration        # g, as .x / .y / .z
    w = movement.angular_velocity    # degrees per second
    print("accel x={:5.2f} y={:5.2f} z={:5.2f}".format(a.x, a.y, a.z))
    print("gyro  x={:6.1f} y={:6.1f} z={:6.1f}".format(w.x, w.y, w.z))
    sleep_ms(200)
```
{: .no-snakie }

Run it with the robot flat. Which axis reads about 1.0? Now tip the nose down and watch the numbers swap over.

## Which axis is which?

The axis names printed on the board may not match the robot's front, back, left and right. It depends on how the module is seated in its holder. On Kev's robot, tipping the nose down makes **Y positive**, and nose up makes it negative. So Y is pitch and X is roll. That is the opposite of the library's own naming, which assumes a different mounting.

Don't guess. Measure it on your robot and write down the result.

## From numbers to "tilted"

Once you know which axis is pitch and which is roll, deciding which way the robot is tilted is a simple threshold. Kev's code uses 0.35 g. This version runs anywhere, with made-up readings:

```python
TILT_THRESHOLD_G = 0.35     # from Kev's config.py

def tilt_direction(x, y):
    """Y is pitch and X is roll on Kev's robot. Check yours."""
    pitch, roll = y, x
    if pitch > TILT_THRESHOLD_G:
        return "nose_down"
    if pitch < -TILT_THRESHOLD_G:
        return "nose_up"
    if roll > TILT_THRESHOLD_G:
        return "right"
    if roll < -TILT_THRESHOLD_G:
        return "left"
    return None

# Made-up (x, y) readings in g: flat, nose down, nose up, tipped left
for x, y in ((0.02, 0.01), (0.05, 0.60), (0.00, -0.55), (-0.50, 0.04)):
    print((x, y), "->", tilt_direction(x, y))
```

Pitch is checked before roll. Tipping a tracked robot nose-first usually rolls it a little too, and reporting the bigger, deliberate movement is less twitchy.

**Why this matters:** a robot that knows it's been lifted can stop its motors. A robot that thinks it's driving while its tracks spin in the air wastes battery and can drop off the bench.

## The gyroscope measures rate, not angle

`angular_velocity` tells you how fast the robot is turning. It doesn't tell you the angle. Hold the robot still at 45 degrees and the gyroscope reads zero. To get an angle, you add up the rate over time. That's called **integrating**.

```python
# Made-up gyro readings: degrees per second on one axis, every 100 ms
rates = [0, 20, 40, 40, 20, 0, 0]
DT = 0.1          # seconds between readings

angle = 0.0
for rate in rates:
    # angle changes by (rate x time) each step
    angle = angle + rate * DT
    print("rate {:3d} deg/s -> angle {:.1f} deg".format(rate, angle))
```

Each step adds the rate multiplied by the time that passed. Small errors add up too, so an integrated angle slowly drifts. We'll see why that matters for mapping in lesson 14.

## Try it Yourself

1. Run the first program with the robot flat. Which axis reads about 1.0?
2. Tip the nose down, then up. Which axis changes sign?
3. Challenge: change `tilt_direction()` to return `"flat"` instead of `None` when it is level, and handle both in your own loop.

## Common Issues

**Problem**: The robot reports "left" when you tip it right.
**Solution**: Flip the sign of the roll axis in your code.
**Why**: How the module is seated decides which direction counts as positive. It's a mounting question, not a bug.

**Problem**: No axis reads near 1.0 g when the robot is flat.
**Solution**: Check the module is firmly in its holder and run the scan from lesson 4.
**Why**: At rest the only acceleration is gravity, so a missing 1 g suggests the module isn't talking sensibly.
