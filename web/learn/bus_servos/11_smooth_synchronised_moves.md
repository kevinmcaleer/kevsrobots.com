---
layout: lesson
title: Smooth synchronised moves
author: Kevin McAleer
type: page
cover: /learn/bus_servos/assets/banners/11_smooth_synchronised_moves.jpg
date: 2026-11-24
previous: 10_calibrating_offsets.html
next: 12_kinematics.html
description: See why joints that move at the same speed arrive at different times,
  and use move_all_sync so every joint arrives together
percent: 72
duration: 6
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

Tell every joint to go to its target at the same speed and something odd happens: the joint with the shortest trip arrives first, and the others carry on. The arm swings in a curve and looks nervous. The cure is to give each joint its own speed, so they all start together and **finish together**.

---

## The problem in numbers

Suppose the shoulder has to travel 100 degrees and the wrist only 20. At the same speed, the wrist finishes in a fifth of the time, then waits. The hand takes a lopsided path, which is bad when it's holding a cup.

The fix is a bit of school maths. If every joint must take the same time `t`, then each joint's speed has to be `distance / t`. A joint with a long way to go goes fast, and one with a short way to go goes slowly.

## How move_all_sync does it

`Buddy.move_all_sync(targets, duration_ms=None, max_speed=None, acc=50, wait=False, ...)` takes a dictionary of joint targets in user degrees. You say *how* to pace it with **exactly one** of two options:

- `duration_ms`: every joint takes this long. Each speed becomes `delta / seconds`, where `delta` is how many position steps that joint must travel
- `max_speed`: the longest-travelling joint goes at this speed, and the others are scaled down in proportion to their distance, so they all finish together

For each joint it reads the current position, works out the `delta` to the target, then sends all the `move()` commands one after another. This small program repeats that arithmetic, so you can see the speeds it would choose, with no hardware.

```python
# The speed planning from Buddy.move_all_sync, without any servos.
# deltas are how far each joint must travel, in position steps (4095 = 360 degrees)

def speeds_for_duration(deltas, duration_ms):
    seconds = duration_ms / 1000.0
    return {name: max(1, int(delta / seconds)) for name, delta in deltas.items()}


def speeds_for_max_speed(deltas, max_speed):
    max_delta = max(deltas.values()) or 1
    return {name: max(1, int(max_speed * delta / max_delta))
            for name, delta in deltas.items()}


# Shoulder travels 100 degrees, elbow 60, wrist 20. 4095 steps = 360 degrees.
deltas = {name: int(deg / 360 * 4095)
          for name, deg in {"shoulder": 100, "elbow": 60, "wrist_pitch": 20}.items()}
print("Travel in steps:", deltas)

print("Over 1500 ms:", speeds_for_duration(deltas, 1500))
print("Fastest joint at 600:", speeds_for_max_speed(deltas, 600))

# Check they finish together: time = distance / speed
for name, speed in speeds_for_duration(deltas, 1500).items():
    print(name, "takes about", round(deltas[name] / speed * 1000), "ms")
```

Run it, and every joint reports about 1500 ms, even though one has five times the distance of another. That's the synchronisation. Speeds are in position steps per second, and `max(1, ...)` stops a nearly-still joint getting speed 0, which on this servo would mean *maximum* speed.

**Why this matters:** every smooth motion on a real arm, from a wave to a pick-and-place, depends on this. It's also why the arm moves as one limb, not six separate ones.

## Waiting for the end

With `wait=True`, the method polls `read_moving` on each joint until all report stopped, or until `timeout_ms` (10 seconds by default) runs out. Without it, the call returns straight away and your program carries on while the arm moves.

## Syncing the arm

This complete program is based on the repo's `demo_sync.py`, but uses the calibrated frame from the last lesson, where 0 is home. It needs the limits from lesson 10 to be wide enough for these angles. Change the target angles to suit your arm, and start small. The first run should be gentle.

```python
import time

from sts3215 import STS3215   # sts3215.py, buddy.py and config.py from the repo
from buddy import Buddy

HOME = {"base": 0, "shoulder": 0, "elbow": 0, "wrist_pitch": 0, "wrist_roll": 0}
REACH = {"base": 40, "shoulder": 30, "elbow": 60, "wrist_pitch": -30, "wrist_roll": 20}

bus = STS3215(uart_id=0, tx_pin=0, rx_pin=1, baudrate=1_000_000)
buddy = Buddy(bus, config_path="/config.json")

print("Ping:", buddy.ping_all())
buddy.set_torque_all(True)
time.sleep_ms(200)

print("Unsynchronised move to REACH, all at speed 300")
buddy.move_all(REACH, speed=300, acc=20)
time.sleep(4)

print("Back to HOME, synchronised over 2000 ms")
buddy.move_all_sync(HOME, duration_ms=2000, acc=20, wait=True)

print("To REACH, synchronised over 2000 ms")
buddy.move_all_sync(REACH, duration_ms=2000, acc=20, wait=True)

print("Home again, paced by max_speed=600")
buddy.move_all_sync(HOME, max_speed=600, acc=20, wait=True)

buddy.set_torque_all(False)
print("Done. Torque off")
```

Watch the first move and the second one side by side. In the first, the short-travel joints get there first. In the second, everything lands together.

**Things to know:**

- `duration_ms` is a target, not a promise. Acceleration (`acc`) means it won't be exact to the millisecond
- `move_all_sync` doesn't cap speeds. A very short duration over a long distance asks for a speed above the servo's 1 to 4095 range, so keep durations realistic
- A joint that doesn't answer is skipped, with a `buddy: skip` message, and the rest still move
- Pass **exactly one** of `duration_ms` and `max_speed`, or you get a `ValueError`

## Try it Yourself

1. Change `duration_ms` to 800, then to 4000. How does the feel of the arm change?
2. Swap `duration_ms=2000` for `max_speed=300`. Which joint sets the pace?
3. Challenge: make a three-pose routine with `wait=True` between poses, ending with `buddy.home()` to park the arm.

## Common Issues

**Problem**: `ValueError: specify exactly one of duration_ms or max_speed`.
**Solution**: Pass one, not both and not neither.
**Why**: The two options each decide the pacing, so together they'd conflict.

**Problem**: The arm doesn't reach the pose I asked for, and `buddy: clamped` is printed.
**Solution**: Widen the joint limits in `config.json`.
**Why**: Requests outside `min_deg` and `max_deg` are clamped.

**Problem**: One joint finishes late.
**Solution**: Check for `buddy: skip` and `ping_all()`, and check that joint isn't carrying more load than its gearing allows.
**Why**: A joint that can't hold the speed asked for lags behind.

---

## Check your understanding

{% quiz %}
? Why do joints moving at the same speed arrive at different times?
- [ ] The servos run at different voltages
- [x] They have different distances to travel
- [ ] The bus sends commands at different speeds
> Equal speed over unequal distances gives unequal travel times.

? In `duration_ms` mode, what speed does each joint get?
- [ ] The same speed for every joint
- [x] Its travel distance divided by the duration
- [ ] Zero, which means stop
> A longer trip needs a higher speed to take the same time.

? What does `wait=True` do?
- [ ] Slows the moves down
- [x] Polls `read_moving` until the joints stop, or until the timeout
- [ ] Switches torque off
> Without it the call returns as soon as the commands are sent.
{% endquiz %}
