---
layout: lesson
title: Qwiic and I2C
author: Kevin McAleer
type: page
cover: /learn/robot_senses/assets/banners/03_qwiic_and_i2c.jpg
date: 2026-11-24
previous: 02_meet_modulino_smars.html
next: 04_first_connection.html
description: Understand how five modules share one pair of wires using I2C, what Qwiic
  adds, and why every module has an address
percent: 20
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

Five sensors, and almost no wiring. How? Every Modulino talks over **I2C**, and every cable is a **Qwiic** cable. Once you understand those two ideas, the whole robot stops being magic.

## I2C in one minute

I2C is a way for many devices to share just two signal wires. One wire, **SDA**, carries data. The other, **SCL**, carries the clock. Your board is the **controller**. Each module is a **target** with its own **address**, a number that says "this message is for me".

When your code reads the distance sensor, the board puts the distance sensor's address on the bus. Only that module answers. The rest stay quiet.

This course doesn't teach I2C from scratch. If you'd like the full story, the [I2C and SPI course](/learn/i2c_spi/) has it. Start with [What is I2C?](/learn/i2c_spi/01_what-is-i2c.html) and [Scanning I2C Devices](/learn/i2c_spi/03_scanning-i2c-devices.html). The [MicroPython GPIO course](/learn/micropython_gpio/) also has an I2C lesson.

## What Qwiic adds

Qwiic is a connector standard. The plug is a small polarised 4-pin connector, so it only fits one way. The four pins are always in the same order: GND, 3.3 V, SDA, SCL. Arduino's Modulino connector, SparkFun's Qwiic and Adafruit's STEMMA QT are the same thing electrically and physically.

That is why five modules from Arduino click straight into a board Arduino never made. They all agree on the voltage (3.3 V) and the pin order.

Every Modulino has two Qwiic sockets, wired in parallel. You click one cable into the next module and keep going. There's no breadboard and no wiring diagram.

One practical limit is worth knowing. SparkFun's guide suggests a daisy-chain of around seven boards is where shared pull-ups and cable current limits start to bite. Five is comfortable.

## Every module has an address

An address is just a number. On the Modulino SMARS chain, a scan shows these:

| Address | Module |
|---|---|
| `0x1E` | Buzzer |
| `0x24` | Motors |
| `0x29` | Distance |
| `0x39` | LED Matrix |
| `0x6A` | Movement |
{:class="table table-single"}

Two things to know about addresses.

**Some can move, some can't.** The Buzzer, Motors and LED Matrix have a small processor onboard, so their address can be changed if two things collide. The bare sensors, Distance and Movement, have the address fixed in the chip. That means you can't put two Distance modules on one chain.

**Documentation sometimes shows a different number.** Arduino's library documents the LED Matrix as `0x72`, but a scan shows `0x39`. They are the same address written two ways. The documented number is the 8-bit form, which is the real address shifted left by one bit. A scan reports the 7-bit form. You can check it yourself with plain MicroPython:

```python
# Documented (8-bit) addresses from the library and Kev's build write-up
documented = {"Buzzer": 0x3C, "Motors": 0x48, "LED Matrix": 0x72}

for name, addr8 in documented.items():
    # Shift right by one bit to get the 7-bit address a scan reports
    addr7 = addr8 >> 1
    print(name, "documented", hex(addr8), "-> scan shows", hex(addr7))
```

The right shift `>> 1` drops the lowest bit, which is the same as halving the number. That gives `0x1E`, `0x24` and `0x39`, matching the scan table.

## Why addresses matter to your robot

Two devices with the same address can't share a bus. Kev's write-up warns that the Buzzer's documented `0x3C` is also the usual address of common small OLED displays, and that Motors' `0x48` is shared with some ADC and temperature chips. If you add a status display later, one of the two has to move. This is why you scan before you wire up something new.

## Try it Yourself

1. Add `Distance` at `0x29` and `Movement` at `0x6A` to a dictionary in the code above. They are already 7-bit, so print them without shifting.
2. Work out by hand what `0x39 << 1` is. Does it equal `0x72`?
3. Challenge: write a function `check(found)` that takes a list of addresses and prints which expected modules are missing. You'll write the real version in the next lesson.

## Common Issues

**Problem**: The scan shows `0x39` but the docs say `0x72`.
**Solution**: Nothing is wrong. The docs use the 8-bit form.
**Why**: Shifting the 7-bit address left by one gives the 8-bit form. Libraries and scans just print different forms of the same number.

**Problem**: I want two of the same Distance module.
**Solution**: You can't on one bus.
**Why**: Its address is fixed in silicon. The modules with an onboard processor can be re-addressed. The bare-sensor ones can't.
