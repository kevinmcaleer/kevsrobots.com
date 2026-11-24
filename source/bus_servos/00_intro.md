---
title: Introduction
description: Meet bus servos, the smart servos that chain on one cable and tell you where they are, and see how this course builds up to the Buddy robot arm
layout: lesson
type: page
cover: /learn/bus_servos/assets/banners/00_intro.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Ahoy there makers! Two servos on a single data wire, daisy chained, and I can ask each one exactly where it is at any moment. No jitter, no guessing, and almost no wiring. Those are bus servos, and this course takes you from one servo on the bench to a six-joint robot arm with its own web control panel.

Watch this first, then come back and let's build.

{% include youtubeplayer.html id="nzBZOTdEdtE" %}

---

## Overview

A hobby servo like the SG90 needs its own signal wire and cannot tell you where it is. A **bus servo** talks over a serial line instead. Every servo on the cable has an ID number, you address each one by number, and each one can answer back with its position, speed and temperature.

In this course you'll use the Feetech **STS3215**, driven from a Raspberry Pi Pico with MicroPython. We build in four steps, and each step adds one idea to the same growing project.

```mermaid
flowchart LR
    A["One servo"] --> B["Two servos on a chain"]
    B --> C["Teach mode: record and replay"]
    C --> D["Buddy: a 6-servo arm"]
```

Buddy is Kev's 6-joint robot arm. By the end it has calibrated joints, smooth moves where every joint arrives together, kinematics so you can say where the gripper should go, and a control panel that the Pico serves to your phone.

## Course Content

- **Your first bus servo**: why bus servos are different, the parts and wiring, the MicroPython library, and reading feedback
- **Two servos and beyond**: giving servos IDs, chaining them, a wave demo, teach and repeat, and choosing the right gearing
- **The Buddy arm**: the six joints, calibrating offsets, synchronised moves, and forward and inverse kinematics
- **Control panel and wrap-up**: a web interface served from the Pico, troubleshooting, and where to go next

## Key Results

After completing this course, you will:

- Explain what a bus servo is and how it differs from a PWM servo
- Wire a Pico to a servo driver board and move an STS3215 from MicroPython
- Read position, speed, movement and temperature back from a servo
- Set servo IDs and run several servos on one cable
- Record a sequence of poses by hand and replay it
- Calibrate a robot arm, move its joints together, and use kinematics to place the gripper
- Control the arm from a browser on your phone

## What you'll need

- A Raspberry Pi Pico (a **Pico W** for lesson 13, the web control panel)
- A Waveshare serial bus servo driver board
- Two or more Feetech STS3215 servos. Buddy uses six
- A 7.4V power supply, or a 2S LiPo battery, for the servo bus
- Jumper wires
- A computer with [Thonny](https://thonny.org/) or `mpremote` to copy files onto the Pico
- Optional, to build the arm itself: a 3D printer. Buddy's mechanical design is Kev's own remodel of an arm from Printables, and lesson 9 points you to the repo and the video

You should already be comfortable with the MicroPython basics and with how hobby servos work. If not, these two courses come first:

- [Learn MicroPython - The basics](/learn/micropython/)
- [Motors and motor drivers](/learn/motors_and_drivers/)

New to the Pico? [How to install MicroPython](/learn/how_to_install_micropython/) gets the firmware onto the board.

## How the course works

**One library, one repo.** All the code that talks to the servos lives in Kev's public repo, [kevinmcaleer/sts3215](https://github.com/kevinmcaleer/sts3215). Download it once. Whenever a lesson says `from sts3215 import STS3215`, that file must be on your Pico. Copy it with `mpremote cp sts3215.py :`, or open it in Thonny and use File, Save as, Raspberry Pi Pico. Lessons that use more files, such as `buddy.py` and `kinematics.py`, say which ones to copy.

**Complete programs.** Every program in a lesson is whole: imports, setup and the loop. Save it on the Pico and run it.

**Move it slowly.** Servos are strong. Start with speeds that let you stop them, and keep a hand near the power switch the first time you run a new program.

**Quizzes.** Some lessons end with a short quiz. Your score is kept in your browser.

**Mistakes are normal.** Most lessons have a Common Issues list. Wiring slips and duplicate IDs catch everyone once.

Ready? Let's find out why bus servos are such a big deal.
