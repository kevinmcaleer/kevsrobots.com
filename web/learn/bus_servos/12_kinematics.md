---
layout: lesson
title: Kinematics
author: Kevin McAleer
type: page
cover: /learn/bus_servos/assets/banners/12_kinematics.jpg
date: 2026-11-24
previous: 11_smooth_synchronised_moves.html
next: 13_web_control_panel.html
description: Learn forward and inverse kinematics from kinematics.py, then ask Buddy
  to put its gripper at a position in space instead of setting each joint
percent: 78
duration: 7
quiz_total: 19
code_target: cpython
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

So far you've told Buddy what each *joint* should do. But when you reach for a cup, you don't think "shoulder 30, elbow 60". You think "hand there". **Kinematics** is the maths that links the two, and `kinematics.py` does it with nothing but the `math` module.

---

## Two directions

- **Forward kinematics (FK):** given the joint angles, where is the gripper tip? Easy, because you just follow the arm from the base outwards
- **Inverse kinematics (IK):** given where you want the tip, what joint angles get it there? Harder, and it's the one you want

This lesson's programs run on your **computer** as well as on the Pico. They only need `math`, so you can explore the maths without the arm. Save `kinematics.py` from the repo in the same folder as each script, and run it with `python3` on a computer, or copy it to the Pico.

## The arm's geometry

`kinematics.py` describes Buddy as five links with lengths in millimetres, and five rotating joints:

- `L1` is the base column, `L2` the upper arm, `L3` the forearm, `L4` the wrist, `L5` the gripper
- The base turns about the vertical axis. The shoulder, elbow and wrist pitch tilt the arm. The wrist roll twists the hand
- At **home** (every joint at 0) the arm points straight up, and the tip sits at `(0, 0, L1+L2+L3+L4+L5)`

**The link lengths in the file are placeholders** (60, 110, 110, 40 and 70 mm). The mechanical design is Kev's own, so measure *your* arm, from joint centre to joint centre, and change the five constants at the top of `kinematics.py` before you trust any position it gives you.

## Forward kinematics

FK walks the chain. It starts at the base, then for each joint it **rotates** by the joint angle and **translates** along the link. Each step is a 4 by 4 matrix, and multiplying them together gives the tip's position and orientation. In plain terms: rotate, move up a link, rotate again, move up a link, and so on.

```python
from kinematics import forward_kinematics, L1, L2, L3, L4, L5   # kinematics.py from the repo

home = {"base": 0, "shoulder": 0, "elbow": 0,
        "wrist_pitch": 0, "wrist_roll": 0, "gripper": 0}

x, y, z, roll, pitch, yaw = forward_kinematics(home)
print("Home pose: x=%.0f y=%.0f z=%.0f mm" % (x, y, z))
print("Total link length:", L1 + L2 + L3 + L4 + L5)

# Lean the arm: shoulder forward 30, elbow bent 60, wrist back 30, base turned 20
leaning = dict(home, base=20, shoulder=30, elbow=60, wrist_pitch=-30)
x, y, z, roll, pitch, yaw = forward_kinematics(leaning)
print("Leaning: x=%.0f y=%.0f z=%.0f mm" % (x, y, z))
print("         pitch=%.0f yaw=%.0f degrees" % (pitch, yaw))
```

At home the tip is straight up at the sum of the link lengths. The shoulder (30) plus elbow (60) plus wrist (-30) add up to a **pitch of 60**, because in a vertical plane the three tilts just add. The base turned 20 degrees shows up as a yaw of 20.

## Inverse kinematics

IK has more than one possible answer, so the code makes the problem smaller in three steps:

1. **Base angle.** The arm lies in one vertical plane through the base axis, so the base must point at the target: `atan2(y, x)`
2. **Wrist position.** The wrist joint sits `L4 + L5` back from the tip along the direction the hand points. Step back by that distance, and what's left is a two-link arm, the upper arm and forearm
3. **Two-link arm.** Use the law of cosines on the triangle formed by the shoulder, elbow and wrist. It gives the elbow angle, then the shoulder angle. The wrist pitch is whatever's left to make the hand point the right way: `pitch - shoulder - elbow`

The triangle can bend two ways, **elbow down** and **elbow up**, and `elbow_up=True` picks the other one. If the target is further than `L2 + L3` can reach, or too close, there's no triangle, and `inverse_kinematics` returns `None`.

The pose is `(x, y, z, roll, pitch, yaw)`. The arm only has five effective degrees of freedom, so the yaw has to agree with `atan2(y, x)`, and the check rejects anything that doesn't.

This complete program asks IK for angles, then runs FK on the answer to prove it lands back on the target.

```python
from kinematics import forward_kinematics, inverse_kinematics   # kinematics.py from the repo

# (x_mm, y_mm, z_mm, roll, pitch, yaw)
targets = [
    (150.0, 0.0, 150.0, 0.0, 90.0, 0.0),      # forward, hand horizontal
    (220.0, 0.0, 60.0, 0.0, 90.0, 0.0),       # as far forward as it can go
    (100.0, 100.0, 200.0, 0.0, 45.0, 45.0),   # off to the side
    (400.0, 0.0, 60.0, 0.0, 90.0, 0.0),       # too far away
]

for pose in targets:
    angles = inverse_kinematics(pose)
    print("Target", pose)
    if angles is None:
        print("  unreachable")
        continue
    print("  angles:", {name: round(a, 1) for name, a in angles.items()})
    back = forward_kinematics(angles)
    print("  forward check:", tuple(round(v, 1) for v in back))

# The same reachable pose, with the elbow bent the other way
flipped = inverse_kinematics(targets[0], elbow_up=True)
print("Elbow up:", {name: round(a, 1) for name, a in flipped.items()})
```

The forward check should match each target, apart from tiny rounding. The fourth target prints "unreachable", because 400 mm is further than the arm can stretch.

**Why this matters:** now the control panel can offer a "move the gripper here" box, and the arm works out the rest.

## IK on the arm

On the Pico, `Buddy.move_to_pose()` wraps all of this. It solves IK, skips the gripper (a pose doesn't say how open it is), and hands the joint targets to `move_all_sync`, so the motion is smooth:

```python
buddy.move_to_pose(150, 0, 150, pitch=90, duration_ms=1500, wait=True)
```
{: .no-snakie }

If the pose can't be reached it raises `ValueError`, so put it in a `try` block. IK knows the arm's geometry but not your joint limits, so a solution may ask for an angle your limits clamp. You then see `buddy: clamped`, and the arm lands somewhere else. The repo's `demo_ik.py` drives the arm through a sequence of poses this way.

Notice the first target above needs a **negative shoulder angle**. That's one more reason lesson 10 told you to widen the limits.

## Try it Yourself

1. Change the five link lengths in `kinematics.py` to match your arm, then rerun the FK home pose. Does the height match a tape measure?
2. Find the furthest `x` the IK program can reach at `z=60`. Is it close to `L2 + L3`?
3. Challenge: write a loop that moves the target in a circle of radius 50 mm in the y direction at fixed `x` and `z`, and prints the joint angles at each step.

## Common Issues

**Problem**: `inverse_kinematics` returns `None` for a target that looks reachable.
**Solution**: Leave `yaw` as `atan2(y, x)` for off-axis targets, and check the distance against `L2 + L3`.
**Why**: The yaw check rejects poses whose yaw doesn't agree with the base angle, and the reach limit is real.

**Problem**: The arm ends up in the wrong place compared to the numbers.
**Solution**: Measure and set `L1` to `L5`, and recheck the calibration.
**Why**: IK uses the link lengths you give it, and placeholder values don't match your arm.

---

## Check your understanding

{% quiz %}
? What does forward kinematics compute?
- [ ] The joint angles for a target position
- [x] The gripper's position from the joint angles
- [ ] The best servo gearing
> FK follows the chain from the base outwards.

? Why can `inverse_kinematics` return `None`?
- [ ] The servos are switched off
- [x] The target is out of reach, or the yaw disagrees with the base direction
- [ ] The base has no angle
> No solution exists for an unreachable pose, so the function says so rather than guessing.

? What do you change so the maths matches your own arm?
- [ ] The baud rate
- [x] The link-length constants `L1` to `L5` in `kinematics.py`
- [ ] The servo IDs
> The values in the file are placeholders for you to tune.
{% endquiz %}
