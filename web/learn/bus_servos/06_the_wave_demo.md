---
layout: lesson
title: The wave demo
author: Kevin McAleer
type: page
cover: /learn/bus_servos/assets/banners/06_the_wave_demo.jpg
date: 2026-11-24
previous: 05_servo_ids_and_chaining.html
next: 07_teach_and_repeat.html
description: Chain two servos and make them sweep in a smooth wave, with the second
  following the first, while printing live position feedback
percent: 42
duration: 4
quiz_total: 19
date_updated: 2026-11-24
navigation:
- name: Bus Servos and the Buddy Robot Arm
- content:
  - section: Introduction
    content:
    - name: Introduction
      link: 00_intro.html
  - section: Your first bus servo
    content:
    - name: Why bus servos
      link: 01_why_bus_servos.html
    - name: Parts and wiring
      link: 02_parts_and_wiring.html
    - name: The library and your first move
      link: 03_the_library_and_first_move.html
    - name: Reading feedback
      link: 04_reading_feedback.html
  - section: Two servos and beyond
    content:
    - name: Servo IDs and chaining
      link: 05_servo_ids_and_chaining.html
    - name: The wave demo
      link: 06_the_wave_demo.html
    - name: Teach and repeat
      link: 07_teach_and_repeat.html
    - name: Gearing and choosing servos
      link: 08_gearing_and_choosing_servos.html
  - section: The Buddy arm
    content:
    - name: Meet Buddy
      link: 09_meet_buddy.html
    - name: Calibrating offsets
      link: 10_calibrating_offsets.html
    - name: Smooth synchronised moves
      link: 11_smooth_synchronised_moves.html
    - name: Kinematics
      link: 12_kinematics.html
  - section: Control panel and wrap-up
    content:
    - name: The web control panel
      link: 13_web_control_panel.html
    - name: Troubleshooting
      link: 14_troubleshooting.html
    - name: Summary
      link: 15_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

Two servos, one cable, and a smooth wave rippling from one to the other. This is the demo from Kev's video, and it shows off everything so far: IDs, chained servos, fast moves and live feedback.

---

## Before you run it

You need servos on **ID 1 and ID 2**, chained as in the last lesson, with the `sts3215.py` driver on the Pico. Mount the servos so the horns can swing freely, or just lay them on the bench with nothing attached.

## How the wave works

The wave comes from three small ideas.

**A sine wave of targets.** We build a list of positions that follow a sine curve around the centre. Sine rises and falls smoothly, so the servo speeds up and slows down gently at each end instead of snapping back.

```python
# 60 positions that swing 1000 steps either side of centre (2048)
targets = [int(2048 + 1000 * math.sin(2 * math.pi * i / 60)) for i in range(60)]
```
{: .no-snakie }

**A lag.** Servo 1 reads the list at step `i`. Servo 2 reads it six steps *behind*, at `i - 6`, so it does what servo 1 did a moment ago. That's the ripple.

**Feedback.** After each step the program reads both positions back and prints them next to the targets, so you can see the servos tracking.

The amplitude of 1000 steps is about 88 degrees each side of centre, and 60 steps at 40 ms is a full cycle every 2.4 seconds or so. Reading and writing over the bus takes a little time too, so it runs a touch slower than that.

## The program

This is the complete demo, tidied from the repo's `demo.py`. Ctrl-C stops it, and the `except` block parks both servos in the middle and lets them go limp.

```python
import math
import time

from sts3215 import STS3215, position_to_degrees   # sts3215.py from the repo

SERVO_1 = 1
SERVO_2 = 2

CENTRE = 2048
AMPLITUDE = 1000   # about 88 degrees either side of centre
STEPS = 60         # samples per full sweep cycle
STEP_DELAY_MS = 40
FOLLOW_LAG = 6     # servo 2 trails servo 1 by this many steps
SPEED = 2400
ACC = 50


def sweep_positions(steps, centre, amplitude):
    """Build one full back-and-forth sweep using a sine wave."""
    return [int(centre + amplitude * math.sin(2 * math.pi * i / steps))
            for i in range(steps)]


def fmt_pos(pos):
    if pos is None:
        return "----  (---.-)"
    return "{:4d}  ({:5.1f})".format(pos, position_to_degrees(pos))


bus = STS3215(uart_id=0, tx_pin=0, rx_pin=1, baudrate=1_000_000)

for servo_id in (SERVO_1, SERVO_2):
    bus.set_torque(servo_id, True)

# Park both servos at the centre first.
bus.move(SERVO_1, CENTRE, speed=SPEED, acc=ACC)
bus.move(SERVO_2, CENTRE, speed=SPEED, acc=ACC)
time.sleep_ms(500)

targets = sweep_positions(STEPS, CENTRE, AMPLITUDE)

print("Starting wave demo. Press Ctrl-C to stop.")
try:
    i = 0
    while True:
        t1 = targets[i % STEPS]
        t2 = targets[(i - FOLLOW_LAG) % STEPS]

        bus.move(SERVO_1, t1, speed=SPEED, acc=ACC)
        bus.move(SERVO_2, t2, speed=SPEED, acc=ACC)

        p1 = bus.read_position(SERVO_1)
        p2 = bus.read_position(SERVO_2)

        print("step {:3d}  s1 target={:4d} pos={}  s2 target={:4d} pos={}".format(
            i, t1, fmt_pos(p1), t2, fmt_pos(p2)))

        time.sleep_ms(STEP_DELAY_MS)
        i += 1
except KeyboardInterrupt:
    print("Stopping. Releasing torque.")
    bus.move(SERVO_1, CENTRE, speed=SPEED, acc=ACC)
    bus.move(SERVO_2, CENTRE, speed=SPEED, acc=ACC)
    time.sleep_ms(500)
    bus.set_torque(SERVO_1, False)
    bus.set_torque(SERVO_2, False)
```

Notice that each `move()` is *not* waited for. We fire a new target every 40 ms and let the servo chase it. That's why the motion is smooth: the servo is always heading for a target that's only a tiny step away.

**Why this matters:** this "stream of small targets" pattern is how you animate anything, from a wave to a walking gait.

**Common mistakes:**

- Running it with only one servo connected. You'll see `----` in the second servo's column because it doesn't answer
- Stopping the program with the reset button instead of Ctrl-C. The servos stay powered where they were

## Try it Yourself

1. Change `FOLLOW_LAG` to 12, then to 0. What happens to the ripple?
2. Raise `AMPLITUDE` to 1400, then lower `STEP_DELAY_MS` to 20. How fast can you go before the printed positions fall behind the targets?
3. Challenge: add a third servo on ID 3 with a lag of 12 and make a three-link wave.

## Common Issues

**Problem**: The second servo never moves and its column shows dashes.
**Solution**: Run the scan from lesson 5 and check that ID 2 answers.
**Why**: If the ID isn't on the bus, `read_position` returns `None`, and the move goes nowhere.

**Problem**: The motion looks juddery.
**Solution**: Check the supply can deliver enough current, and try a lower `SPEED` or a larger `ACC`.
**Why**: A weak supply sags when both servos accelerate together.
