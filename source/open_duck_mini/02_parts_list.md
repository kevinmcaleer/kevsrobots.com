---
title: The Parts List
description: What each part of the Open Duck Mini does, and where to find the bill of materials so you buy the right things
layout: lesson
type: page
cover: /learn/open_duck_mini/assets/banners/02_parts_list.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Let's go shopping. Before you order anything, it's worth knowing what each part is *for*. That way, when the BOM lists something you've never heard of, you'll know why it's there and what happens if you swap it.

---

## The BOM is the source of truth

The full **bill of materials (BOM)** is a Google Sheet, linked from the [Open_Duck_Mini README](https://github.com/apirrone/Open_Duck_Mini). It has the exact parts, quantities, suppliers and prices. The project aims to keep the whole build under $400.

This lesson doesn't repeat quantities, prices, screw counts or the exact servo board, because they change. **Order from the BOM sheet**, and use this lesson to understand it.

---

## What each part does

| Part | What it does in the duck |
|---|---|
| 14 x Feetech STS3215 servos | The muscles. Five per leg and four for the neck and head. They're serial bus servos, so they share one data line and each has its own ID |
| USB-serial servo control board | Connects the Pi to the servo bus over USB. The exact board is on the BOM |
| Raspberry Pi Zero 2W | The brain. Runs the walking policy and talks to everything else. A Pi 5 also works |
| BNO055 IMU | The inner ear. Measures how the body is tilted and turning, over I2C |
| 2 x foot micro switches | Tell the policy when each foot is on the ground |
| Battery pack | Powers the robot so it can walk untethered. Take the exact pack from the BOM |
| Xbox controller | Your remote control, over Bluetooth |
| M3 heat-set inserts | Give the plastic parts strong metal threads to screw into |
| M3x10 and M3x6 screws | Hold the printed parts and motors together |
| Loctite 243 | Thread locker for the metal-to-metal motor screws only |
| Bearings | Let joints such as the trunk and head turn smoothly |
{:class="table table-single"}

---

## The optional expression parts

The duck can walk without any of these. They give it personality, and you can add them later.

| Part | What it does |
|---|---|
| Eye LEDs | Light up the eyes |
| Projector LED | A light you can toggle from the controller |
| MAX98357A I2S amplifier and speaker | Plays sounds |
| Two antenna servos | Wiggle the antennas from the triggers |
| Ear servos | Move the ears |
{:class="table table-single"}

You turn these features on or off in a config file on the Pi, which we'll meet in lesson 10.

---

## Printed parts

Most of the duck's body is 3D printed. The printed parts are named in the CAD and the assembly guide, such as `trunk_bottom`, `trunk_top`, `foot_top`, `leg_spacer` and `body_front`. The feet are a mix of a TPU part and a PLA part, which we'll look at in the next lesson.

---

## Tools you'll want

- A 3D printer that can handle PLA and TPU
- A soldering iron, for heat-set inserts and for wiring
- Small hex keys or screwdrivers to suit the M3 screws
- A computer to flash the Pi's SD card and connect to it over SSH

---

## Try it Yourself

1. Open the BOM sheet and find each part from the first table in it. Make a note of anything you already own.
2. Find the USB-serial servo control board on the BOM and note its exact name. You'll want it in lesson 8.
3. Decide now whether you want the expression parts. If you're on a tight budget, leave them out for your first build.

## Common Issues

**Problem**: A part on the BOM is out of stock.
**Solution**: Ask the Open Duck community before buying a substitute.
**Why**: The policy was trained for these exact parts. A different servo moves differently, and the walk may not work.

**Problem**: You bought a different IMU because it was cheaper.
**Solution**: Use the BNO055 that the BOM lists.
**Why**: The runtime code is written for the BNO055.
