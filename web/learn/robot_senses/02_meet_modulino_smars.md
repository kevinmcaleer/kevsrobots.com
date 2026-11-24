---
layout: lesson
title: Meet Modulino SMARS
author: Kevin McAleer
type: page
cover: /learn/robot_senses/assets/banners/02_meet_modulino_smars.jpg
date: 2026-11-24
previous: 01_how_robots_sense.html
next: 03_qwiic_and_i2c.html
description: Tour the Modulino SMARS robot, its FeatherS2 Neo brain, the five Modulino
  modules and how the two power supplies fit together
percent: 15
duration: 4
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

Let's meet the robot we'll be teaching. If you've built a [SMARS](/learn/smars/) before, most of it will look familiar. The chassis, the two N20 150RPM micro metal gear motors, the wheels and tracks, and the 9V battery are all stock. Only the electronics are new.

## What changed

The old SMARS brain was an Arduino Uno R3, a motor shield and an HC-SR04 ultrasonic sensor. All of that is gone. In its place are five Arduino **Modulino** modules, one per sense, and a new brain: the **UnexpectedMaker FeatherS2 Neo**.

Two 3D-printed parts make the swap possible. A Modulino holder replaces the old Uno bay and carries the board and four of the five modules. A Modulino Distance mount replaces the HC-SR04 holder at the front. You'll find the STL files on Kev's [build write-up](/blog/my-first-robot-just-got-five-new-senses.html), and the [modulino_smars repository](https://github.com/kevinmcaleer/modulino_smars) has a `meshes` folder of related models.

## The brain does not care who made the sensors

The FeatherS2 Neo is an ESP32-S2 board made by an independent maker, not by Arduino. The Modulinos are Arduino's. They still work together. Arduino's MicroPython library says any board with I2C that can run a modern MicroPython is supported.

Kev chose this board for two reasons. It has a Qwiic-compatible connector, so the whole chain plugs straight in. It also has its own LiPo battery connector, so the board's power is built in.

If you have a different board, keep going. The only thing that changes is a pair of pin numbers, and we'll show you where.

## The five senses

```mermaid
flowchart LR
    B["FeatherS2 Neo"] --> D["Distance"]
    D --> M["Movement"]
    M --> Z["Buzzer"]
    Z --> L["LED Matrix"]
    L --> MO["Motors"]
```

Every Modulino has two Qwiic sockets wired together, so you click one module into the next. The order doesn't matter electrically. We'll explain why in the next lesson.

| Sense | Module | Chip or part | Fun fact |
|---|---|---|---|
| See | Distance | VL53L4CD time-of-flight | Uses a laser, not sound. Reads to about 1.2 m |
| Measure | Movement | LSM6DSOX six-axis IMU | Accelerometer and gyroscope in one |
| Hear and speak | Buzzer | Piezo buzzer | Has its own small processor onboard |
| Show | LED Matrix | 96 blue LEDs, 8 by 12 | You draw to a buffer, then push it |
| Move | Motors | MAX22211 dual H-bridge | Up to 3.8 A per channel, and senses current |
{:class="table table-single"}

The old HC-SR04 reached about 4 m, against about 1.2 m for the new sensor. It is not a straight upgrade. For a small tracked robot that mostly cares about things close by, it is a good trade, and it needs no pulse timing from you.

## Two power supplies

Almost everything clicks together, but there is one wire you do add by hand.

- The **Qwiic chain** carries 3.3 V logic to all five modules.
- A **LiPo battery** plugs into the FeatherS2 Neo's own JST socket and powers the board.
- The stock **9V battery** wires into the screw terminals on the Modulino Motors (`VIN` and `GND`).

The Modulino Motors wants **5 to 24 V** for its motor supply, and 9 V sits comfortably inside that. The grounds are common, but the positive rails stay separate.

> Never run the motors from the Qwiic 3.3 V line. It can't supply the current, and a stalled motor pulling several amps through a Qwiic cable is a fire risk, not just a brownout.

The FeatherS2 Neo also comes with loose male header pins. You don't need them. Nothing on this robot is soldered.

## Try it Yourself

1. Lay your five modules out on the bench and match each one to a row of the table.
2. Find the two Qwiic sockets on each module. Can you see why a chain works?
3. Challenge: decide where you'd mount the Distance module and why. Kev puts it at the front, where the old HC-SR04 sat.

## Common Issues

**Problem**: The robot has two batteries and you aren't sure what each one does.
**Solution**: The LiPo powers the board and the Qwiic chain. The 9V battery only feeds the motor terminals.
**Why**: Motors draw far more current than sensors, so they get their own supply. Mixing them up is the quickest way to damage something.
