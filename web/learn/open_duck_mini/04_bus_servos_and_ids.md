---
layout: lesson
title: Serial Bus Servos and IDs
author: Kevin McAleer
type: page
cover: /learn/open_duck_mini/assets/banners/04_bus_servos_and_ids.jpg
date: 2026-11-24
previous: 03_printing_the_parts.html
next: 05_assembling_the_legs.html
description: Why the Open Duck Mini uses serial bus servos, how the daisy chain works,
  and how to give each of the 14 servos its ID before you build
percent: 30
duration: 6
quiz_total: 12
date_updated: 2026-11-24
navigation:
- name: Build an Open Duck Mini Walking Robot
- content:
  - section: Meet the duck
    content:
    - name: Introduction
      link: 00_intro.html
    - name: What is the Open Duck Mini?
      link: 01_what_is_open_duck_mini.html
  - section: Parts and printing
    content:
    - name: The Parts List
      link: 02_parts_list.html
    - name: Printing the Parts
      link: 03_printing_the_parts.html
    - name: Serial Bus Servos and IDs
      link: 04_bus_servos_and_ids.html
  - section: Assembly
    content:
    - name: Assembling the Legs
      link: 05_assembling_the_legs.html
    - name: Head, Body and Battery
      link: 06_head_body_and_battery.html
    - name: Wiring the Electronics
      link: 07_wiring_the_electronics.html
  - section: Bring it to life
    content:
    - name: Setting up the Pi
      link: 08_setting_up_the_pi.html
    - name: Testing the IMU, Motors and Controller
      link: 09_testing_imu_motors_and_controller.html
    - name: Calibration and Offsets
      link: 10_calibration_and_offsets.html
    - name: First Steps
      link: 11_first_steps.html
  - section: The walking brain
    content:
    - name: How the Duck Learned to Walk
      link: 12_how_the_duck_learned_to_walk.html
    - name: Training Your Own Policy
      link: 13_training_your_own_policy.html
    - name: Troubleshooting
      link: 14_troubleshooting.html
    - name: Summary
      link: 15_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

This is the most important lesson to get right before you build. Every servo needs its own ID, and it needs to be set **before** the servo disappears inside a leg. Do it now and assembly is smooth. Skip it and you'll be taking a leg apart later.

---

## Why serial bus servos?

If you've used hobby servos before, you'll know each one needs its own signal wire and a pulse to tell it where to go. Fourteen of those would mean fourteen signal wires snaking through a small duck.

The Feetech STS3215 is a **serial bus servo**. Instead of a pulse, it listens for messages on a shared data line. Every message is addressed to an **ID**, so only the servo with that ID responds. It can also answer back with its position and speed, which the walking policy needs as part of what it observes.

That gives us three big wins:

- **Fewer wires.** The servos are **daisy-chained**: one cable runs from the control board to the first servo, the next cable runs on to the next servo, and so on.
- **Feedback.** The Pi can read where every joint actually is.
- **Addressing.** One bus, fourteen IDs, and every message reaches the right joint.

```mermaid
flowchart LR
    Pi["Raspberry Pi"] -->|USB| B["USB-serial servo board"]
    B --> S1["Servo ID 10"]
    S1 --> S2["Servo ID 11"]
    S2 --> S3["..."]
    S3 --> S4["Servo ID 33"]
```

The catch: if two servos on the bus share an ID, they both answer at once and the messages get garbled. That's why every servo gets configured **on its own**, before it joins the chain.

---

## The 14 joints and their IDs

These IDs come from `configure_motors.md` in the Open_Duck_Mini repo. The pattern is easy to remember: the tens digit is the limb (1 is the right leg, 2 the left leg, 3 the head) and the units digit counts down the limb.

| Joint | ID | Joint | ID |
|---|---|---|---|
| right_hip_yaw | 10 | left_hip_yaw | 20 |
| right_hip_roll | 11 | left_hip_roll | 21 |
| right_hip_pitch | 12 | left_hip_pitch | 22 |
| right_knee | 13 | left_knee | 23 |
| right_ankle | 14 | left_ankle | 24 |
{:class="table table-single"}

| Head joint | ID |
|---|---|
| neck_pitch | 30 |
| head_pitch | 31 |
| head_yaw | 32 |
| head_roll | 33 |
{:class="table table-single"}

Label each servo with its joint name and ID as soon as you've set it. A strip of masking tape and a pen will save you a lot of confusion later.

---

## Getting the runtime onto a computer

The configuration script lives in the runtime repo, so you need the runtime installed on whatever machine the servo board is plugged into. The simplest route is to set up the Pi first by following [lesson 8](/learn/open_duck_mini/08_setting_up_the_pi.html), then come back here with the servo board plugged into the Pi. Lesson 8 includes these same steps: clone the repo, switch to the `v2` branch and install it.

```bash
git clone https://github.com/apirrone/Open_Duck_Mini_Runtime
cd Open_Duck_Mini_Runtime
git checkout v2
pip install -e .
```

---

## Configuring one servo

Do this for each of the 14 servos, one at a time:

1. Connect **only one** servo to the servo board.
2. Power the servo.
3. Run the script with the ID that servo should have:

```bash
python configure_motor.py --id <id>
```

For example, `--id 13` for the right knee. Check the [runtime README](https://github.com/apirrone/Open_Duck_Mini_Runtime) for which folder the script is in on the `v2` branch.

4. The script sets the ID, then **moves the motor to its zero position**.
5. With the motor at zero, fit the servo horn as close to its zero position as you can.
6. Label the servo, unplug it and move on to the next one.

---

## Horns at zero, offsets later

You won't get every horn perfectly lined up. The horn's splines only fit in steps, so you'll often be a little off. That's fine. In lesson 10 we'll measure a small **offset** for every joint, and the software will correct it. What matters now is getting each horn as close as you can, and never fitting one a whole step or more out.

---

## Try it Yourself

1. Before you plug anything in, write the 14 joint names and IDs on a sheet of paper and tick each one off as you configure it.
2. Group your labelled servos into three piles: right leg, left leg and head.
3. Challenge: why do you think the left leg starts at 20 and not at 15? (Hint: think about what happens when you add a joint later.)

## Common Issues

**Problem**: The script can't find the servo.
**Solution**: Check the servo has power, the cable is fully seated, and only one servo is connected.
**Why**: The servo board can't talk to a servo that is unpowered, and two servos with the same ID clash.

**Problem**: You set the wrong ID on a servo.
**Solution**: Run `configure_motor.py` again on that servo with the right ID.
**Why**: The ID is just a stored setting, and you can change it any time while the servo is on its own.

**Problem**: The horn is a whole spline step off zero.
**Solution**: Take it off and refit it at the closest position.
**Why**: Offsets are for small corrections. A big error can push the joint against its limits.

## Check your understanding

{% quiz %}
? Why must each servo be configured on its own, before assembly?
- [ ] Because the servos only work when one is plugged in
- [x] Because servos that share an ID on the same bus will answer at the same time and clash
- [ ] Because configure_motor.py can only run once per day
> Every message on the bus is addressed to an ID. Two servos with the same ID garble each other's replies, so you set a unique ID on each one while it's alone.

? Which ID belongs to the left knee?
- [ ] 13
- [x] 23
- [ ] 31
> The left leg uses the 20s, and the knee is the fourth joint down: 23. The right knee is 13.

? What should you do when configure_motor.py moves the servo to zero?
- [x] Fit the horn as close to its zero position as you can
- [ ] Leave the horn off until the robot is fully built
- [ ] Turn the horn by hand until it clicks into place
> Getting the horn close now means the offsets you find in lesson 10 stay small.
{% endquiz %}
