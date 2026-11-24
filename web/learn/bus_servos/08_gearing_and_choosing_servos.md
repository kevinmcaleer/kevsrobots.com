---
layout: lesson
title: Gearing and choosing servos
author: Kevin McAleer
type: page
cover: /learn/bus_servos/assets/banners/08_gearing_and_choosing_servos.jpg
date: 2026-11-24
previous: 07_teach_and_repeat.html
next: 09_meet_buddy.html
description: Understand the STS3215 gearing variants, match servo gearing to each
  joint, and protect your servos with position limits and a temperature check
percent: 54
duration: 5
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

Look at the box of an STS3215 and you'll see a model code such as **STS3215-C001** or **STS3215-C044**. Those codes mean different **gearing ratios**, and the gearing trades **speed for torque**. Pick the right one for each joint and the arm works much better.

---

## Speed or strength

Gears are a trade. A servo geared for more torque turns slower, and one geared for more speed is weaker. You can't have both from the same motor.

Kev's arm uses both kinds:

- **More torque, less speed** for the shoulder and elbow, where the mechanical load is greatest. They lift the rest of the arm
- **Less torque, more speed** for the wrist and gripper, where the load is small

The code on the box tells you which variant you have. The exact ratios and the no-load speed and stall torque are on the seller's listing and the Feetech datasheet. Check those numbers rather than trusting a table in a course, because suppliers change their variants.

```mermaid
flowchart TB
    S["Shoulder: lifts the whole arm"] --> T["Choose more torque"]
    E["Elbow: lifts forearm and hand"] --> T
    W["Wrist: carries a light hand"] --> F["Choose more speed"]
    G["Gripper: light load"] --> F
```

**Why this matters:** a shoulder servo that is too weak stalls and overheats. A wrist servo that is too slow makes the whole arm feel sluggish. You can't fix either in software.

## A sensible way to choose

1. List the joints and ask what each one *carries*. Everything beyond a joint loads it
2. Give the most torque to the joints nearest the base that carry the most
3. Give speed to the joints at the tip
4. When in doubt, think about the long lever. A heavy gripper on a long forearm loads the elbow far more than you'd guess

It's a judgement call for joints like the base, which turns the whole arm's weight sideways. Start with the supplier's torque figures and the weight of your build.

## Protect your servos: limits and temperature

Whatever you choose, a joint bolted into a mechanism can hit a hard stop or a cable run. Two cheap protections help, and the blog suggests the first:

- **Position limits.** Clamp the target in code before you send it
- **A temperature check.** A servo that is working too hard gets hot, and `read_temperature()` tells you

The servo also stores its own limits in its control table, which can enforce them regardless of your code. Writing those registers is beyond this course, so we clamp in software.

This complete program sweeps two joints between their limits at their own speeds, prints each servo's temperature, and switches everything off if one gets too hot. Adjust `JOINTS` to your own servos.

```python
import time

from sts3215 import STS3215   # sts3215.py from the repo

# One entry per servo. Limits are position steps (0 to 4095).
JOINTS = {
    1: {"name": "elbow",   "low": 1024, "high": 3072, "speed": 400},
    2: {"name": "gripper", "low": 1500, "high": 2600, "speed": 1200},
}
ACC = 50
TEMP_LIMIT = 60     # degrees C. My example only: pick yours from the datasheet

bus = STS3215(uart_id=0, tx_pin=0, rx_pin=1, baudrate=1_000_000)


def safe_move(servo_id, target):
    """Clamp the target to the joint's limits, then move."""
    joint = JOINTS[servo_id]
    target = max(joint["low"], min(joint["high"], target))
    bus.move(servo_id, target, speed=joint["speed"], acc=ACC)
    return target


def too_hot():
    for servo_id, joint in JOINTS.items():
        temperature = bus.read_temperature(servo_id)
        print("  ", joint["name"], "temperature:", temperature, "C")
        if temperature is not None and temperature > TEMP_LIMIT:
            print("  ", joint["name"], "is too hot")
            return True
    return False


for servo_id in JOINTS:
    bus.set_torque(servo_id, True)

stopped = False
for end in ("low", "high", "low"):
    print("Sweeping to the", end, "limits")
    for servo_id, joint in JOINTS.items():
        safe_move(servo_id, joint[end] - 500 if end == "low" else joint[end] + 500)
    time.sleep(2)
    if too_hot():
        stopped = True
        break

for servo_id in JOINTS:
    bus.set_torque(servo_id, False)
print("Stopped because of heat" if stopped else "Finished normally")
```

Notice the sweep deliberately asks for 500 steps *beyond* each limit. `safe_move()` pulls it back, so you can watch the clamp working, and the servo never goes past the limit.

## Try it Yourself

1. Set the limits of one joint tightly and command a position outside them. Does the servo stop at the limit?
2. Lower `TEMP_LIMIT` to just above the temperature you see at rest and rerun. Does it stop early?
3. Challenge: look up the datasheet limits for your servo, and write them into the program as a comment next to `TEMP_LIMIT`.

## Common Issues

**Problem**: The joint stalls and gets hot.
**Solution**: Reduce the load, shorten the lever, or use a higher-torque variant for that joint.
**Why**: A servo pushing against a load it can't move draws high current and heats up fast.

**Problem**: A joint is slow and sluggish.
**Solution**: Check you haven't put a high-torque servo where a fast one is needed, and raise `speed` before blaming the code.
**Why**: Gearing sets the top speed. `speed` can't beat it.

**Problem**: `TypeError` comparing temperature.
**Solution**: Check `temperature is not None` first, as in the program.
**Why**: A missed reply returns `None`.
