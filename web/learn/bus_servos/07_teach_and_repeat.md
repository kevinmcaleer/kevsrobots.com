---
layout: lesson
title: Teach and repeat
author: Kevin McAleer
type: page
cover: /learn/bus_servos/assets/banners/07_teach_and_repeat.jpg
date: 2026-11-24
previous: 06_the_wave_demo.html
next: 08_gearing_and_choosing_servos.html
description: Switch torque off, move the servos by hand, record each pose into a list,
  save them to a file and play the sequence back
percent: 48
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

This is one of the best features of a bus servo. Switch the torque off and the servo goes limp, but it still **reports its position**. So you can pose a robot with your hands, record where every joint is, and play it back later. Teaching by demonstration, with no maths and no guessing.

---

## The idea

Three steps:

1. **Torque off.** The servos go limp and you move them by hand.
2. **Record.** At each pose, read every servo's position and store it in a list.
3. **Playback.** Torque on, then move the servos to each recorded pose in order.

```mermaid
flowchart LR
    A["Torque off"] --> B["Move by hand"]
    B --> C["Read positions into a list"]
    C --> B
    C --> D["Torque on"]
    D --> E["Play the list back"]
```

The torque calls from the blog are the whole trick:

```python
bus.set_torque(1, False)   # go limp, free to move by hand
pos, _ = bus.read_pos_speed(1)
print("Recorded position:", pos)
bus.set_torque(1, True)    # re-energise
```
{: .no-snakie }

## One safety habit

When you switch torque back on, you want the servo to hold where it is, not lunge somewhere. So the program first sends each servo a `move()` to its *current* position, and only then enables torque. It's one extra write per servo and it means the target matches reality.

## The program

A pose is a list of positions, one per servo in `SERVO_IDS`. The program records a few poses, saves them to a file called `poses.json` so they survive a reboot, then plays them back. Change `SERVO_IDS` to match your chain.

```python
import json
import time

from sts3215 import STS3215   # sts3215.py from the repo

SERVO_IDS = [1, 2]        # the servos in your chain
POSE_FILE = "poses.json"
PLAY_SPEED = 600          # position steps per second
PLAY_ACC = 30
REPEATS = 3
SETTLE_MS = 300           # pause at each pose

bus = STS3215(uart_id=0, tx_pin=0, rx_pin=1, baudrate=1_000_000)


def read_pose():
    """Return a list of positions, one per servo, or None if any is missing."""
    pose = []
    for servo_id in SERVO_IDS:
        position = bus.read_position(servo_id)
        if position is None:
            return None
        pose.append(position)
    return pose


def go_limp():
    for servo_id in SERVO_IDS:
        bus.set_torque(servo_id, False)


def hold_here():
    """Aim every servo at where it is now, then switch torque on."""
    pose = read_pose()
    if pose is None:
        print("Can't read every servo, so torque stays off")
        return False
    for servo_id, position in zip(SERVO_IDS, pose):
        bus.move(servo_id, position, speed=PLAY_SPEED, acc=PLAY_ACC)
    for servo_id in SERVO_IDS:
        bus.set_torque(servo_id, True)
    return True


def wait_until_stopped(timeout_ms=5000):
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
        if all(bus.read_moving(s) == 0 for s in SERVO_IDS):
            return
        time.sleep_ms(50)


def record():
    count = int(input("How many poses will you record? "))
    poses = []
    go_limp()
    for n in range(count):
        input("Pose {}: move the servos by hand, then press ENTER ".format(n + 1))
        pose = read_pose()
        if pose is None:
            print("  No reading, so that pose was skipped")
            continue
        poses.append(pose)
        print("  Saved", pose)
    return poses


def save_poses(poses):
    with open(POSE_FILE, "w") as f:
        json.dump(poses, f)


def load_poses():
    try:
        with open(POSE_FILE) as f:
            return json.load(f)
    except (OSError, ValueError):
        return []


def play(poses):
    if not poses or not hold_here():
        print("Nothing to play")
        return
    for lap in range(REPEATS):
        print("Lap", lap + 1, "of", REPEATS)
        for pose in poses:
            for servo_id, position in zip(SERVO_IDS, pose):
                bus.move(servo_id, position, speed=PLAY_SPEED, acc=PLAY_ACC)
            time.sleep_ms(100)       # let the servos start before we ask
            wait_until_stopped()
            time.sleep_ms(SETTLE_MS)
    go_limp()
    print("Done. Torque off")


answer = input("Record a new sequence? (y/n) ")
poses = load_poses()
if answer.strip().lower() == "y" or not poses:
    poses = record()
    save_poses(poses)
play(poses)
```

Run it, answer `y`, and the servos go limp. Move them to a pose, press ENTER in Thonny's shell, move them to the next pose, and so on. Then watch your arm go through the moves on its own, three times.

## Notice the limit

Each pose moves every servo with the same `speed`, so a servo with a short way to go arrives first and has to wait for the others. It looks a little robotic. Lesson 11 fixes that by scaling each joint's speed so they all arrive together.

**Common mistakes:**

- Recording poses with torque on. The servo fights your hand, and you may strain it
- Playing back a sequence that starts far from where the arm is now. The first move will be large and fast, so keep clear

## Try it Yourself

1. Record a sequence of four poses that pick a pretend object up and put it down.
2. Open `poses.json` on the Pico and look at the numbers. Each inner list is one pose.
3. Challenge: make the program loop until you press Ctrl-C, then go limp inside a `try` and `except KeyboardInterrupt` block.

## Common Issues

**Problem**: `Can't read every servo, so torque stays off`.
**Solution**: Run the scan from lesson 5 and check every ID in `SERVO_IDS` answers.
**Why**: The program won't switch torque on blind, because it can't aim the servo at where it is.

**Problem**: Playback runs, but poses are skipped or wrong.
**Solution**: Delete `poses.json`, answer `y` and record again.
**Why**: An old file recorded with different servos has the wrong number of positions in each pose.
