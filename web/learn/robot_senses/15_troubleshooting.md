---
layout: lesson
title: Troubleshooting
author: Kevin McAleer
type: page
cover: /learn/robot_senses/assets/banners/15_troubleshooting.jpg
date: 2026-11-24
previous: 14_mapping_and_slam.html
next: 16_summary.html
description: A single checklist for the common Modulino SMARS problems, from a silent
  I2C bus and a dark LED matrix to reversed motors and a robot that won't react
percent: 80
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

Every robot misbehaves sometimes. The trick isn't never having problems. It's finding them quickly. This lesson collects the faults you're most likely to meet, with a short routine for finding the rest.

## Start here: the three-step routine

1. **Scan the bus.** Run the scan from lesson 4. It tells you in seconds whether a module is reachable. A missing module is nearly always a connector, not a broken part.
2. **Test one sense alone.** Run the single-sense program from its lesson. If it works alone but not in your robot code, the bug is in your code.
3. **Print the numbers.** Print the readings your decisions use. Most robot bugs are a threshold that doesn't match what the sensor really says.

## Problems, fixes and reasons

| Problem | Solution | Why |
|---|---|---|
| `RuntimeError: I2C interface couldn't be determined automatically` | Build an `I2C` bus with your board's SCL and SDA pins and pass `i2c_bus=i2c` to every Modulino | The library only auto-detects Arduino's own boards. The FeatherS2 Neo is not one of them |
| The scan finds nothing | Check the Qwiic cable from the board to the first module and that the board is powered | Power and data both travel down that one cable |
| One module is missing from the scan | Unplug and re-seat its Qwiic cable | Connectors click when they are home. A half-seated one drops out |
| LED Matrix stays dark | Call `matrix.show()` after drawing, and give the address `0x39` | Drawing writes to a buffer. Auto-discovery doesn't find the matrix on every board |
| The face is upside down | Flip both axes in your drawing code | The module can't know how it was mounted |
| `distance` crashes the loop with `TypeError` | Check for `None`, and catch `TypeError` and `OSError` around the read | `None` means nothing in range, and a sensor timeout can raise instead of returning |
| The robot spins instead of driving straight | Flip `INVERT_RIGHT` | The two motors face opposite ways |
| The robot drives backwards | Flip both invert flags | The same fix, applied to both sides |
| Motors don't turn | Check the 9V battery is on `VIN` and `GND` on the Motors module | The motors have their own supply, separate from Qwiic |
| The robot hesitates when it beeps | Use `blocking=False` for beeps inside the loop | A blocking note holds up the loop |
| It reports left when tipped right | Swap the sign of the roll axis | Which direction is positive depends on how the module is seated |
| The robot ignores being lifted | Lower the tilt threshold and check which axis is pitch | If the axes are swapped, the code watches the wrong one |
| A second Distance module is not found | Use one per chain | Its address is fixed in the chip |
{:class="table table-single"}

## Two addresses that cause confusion

Two Modulino addresses match common parts. The Buzzer's documented `0x3C` is the usual address of small OLED displays. The Motors' documented `0x48` is shared with some ADC and temperature chips. The Buzzer and Motors have an onboard processor, so their addresses can be changed if you hit a collision. The bare-sensor modules cannot be changed. Remember that a scan shows the 7-bit form, so `0x3C` appears as `0x1E`, and `0x48` as `0x24`.

## Power rules to remember

- Never run motors from the Qwiic 3.3 V line.
- The LiPo powers the board. The 9V battery only feeds the motor terminals. Grounds are common, but the positive rails stay separate.
- Keep a countdown before the motors arm, and test with the tracks off the ground.

## When you think the hardware is broken

Before you blame a module, check these in order:

1. Is it listed in the scan?
2. Does the single-sense program for it work alone?
3. Does it work with a different Qwiic cable?
4. Have you printed the values your code uses?

A module that appears in the scan is almost certainly alive. The problem is usually in what the code does with it.

If you hit something this course doesn't cover, the official Modulino documentation is at [docs.arduino.cc](https://docs.arduino.cc/), and Arduino's MicroPython library is on [GitHub](https://github.com/arduino/arduino-modulino-mpy).

## Try it Yourself

1. Unplug one cable and run the scan. Does the message match what you expected?
2. Break your robot on purpose: remove `i2c_bus=i2c` from one module. Read the error and fix it.
3. Challenge: write a `selfcheck()` function that scans the bus and prints which senses are missing, and call it at the top of `main.py`.
