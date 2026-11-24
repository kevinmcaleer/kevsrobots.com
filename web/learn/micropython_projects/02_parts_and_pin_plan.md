---
layout: lesson
title: Parts and pin plan
author: Kevin McAleer
type: page
cover: /learn/micropython_projects/assets/banners/02_parts_and_pin_plan.jpg
date: 2026-11-24
previous: 01_meet_the_status_buddy.html
next: 03_reed_switches_and_magnets.html
description: The parts list for the status buddy and one pin plan that every lesson
  in the course uses
percent: 15
duration: 3
quiz_total: 19
date_updated: 2026-11-24
navigation:
- name: MicroPython Projects - Make It Show, Talk and Connect
- content:
  - section: Getting started
    content:
    - name: Introduction
      link: 00_intro.html
    - name: Meet the status buddy
      link: 01_meet_the_status_buddy.html
    - name: Parts and pin plan
      link: 02_parts_and_pin_plan.html
  - section: Sense
    content:
    - name: Reed switches and magnets
      link: 03_reed_switches_and_magnets.html
    - name: Reading the door
      link: 04_reading_the_door.html
    - name: Interrupts and debouncing
      link: 05_interrupts_and_debounce.html
  - section: Show
    content:
    - name: OLED wiring and first text
      link: 06_oled_wiring_and_first_text.html
    - name: Drawing shapes and icons
      link: 07_drawing_shapes_and_icons.html
    - name: The status screen
      link: 08_the_status_screen.html
  - section: Talk
    content:
    - name: Meet SAM
      link: 09_meet_sam.html
    - name: Giving the buddy a voice
      link: 10_giving_the_buddy_a_voice.html
  - section: Connect
    content:
    - name: Pico W and Wi-Fi
      link: 11_pico_w_and_wifi.html
    - name: Arduino Cloud
      link: 12_arduino_cloud.html
  - section: Shrink it
    content:
    - name: Porting to a XIAO
      link: 13_porting_to_a_xiao.html
  - section: Putting it together
    content:
    - name: The full gadget
      link: 14_the_full_gadget.html
    - name: Troubleshooting
      link: 15_troubleshooting.html
    - name: Summary
      link: 16_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

Before we wire anything, let's gather the parts and agree on a **pin plan**. If every lesson uses the same pins, the programs you write in lesson 5 will still work in lesson 14.

---

## Parts list

| Part | What it is for | Needed from |
|---|---|---|
| Raspberry Pi Pico W | The brain. A plain Pico works too, without Wi-Fi | Lesson 3 |
| Reed switch (normally open) | Senses the magnet | Lesson 3 |
| Neodymium magnet, around 10mm | Goes on the door | Lesson 3 |
| 0.96 inch SSD1306 OLED, I2C, 128x64 | The status screen | Lesson 6 |
| Small speaker or piezo buzzer | SAM's voice | Lesson 9 |
| 1K resistor (optional) | Smooths out pops and clicks from the speaker | Lesson 9 |
| Jumper wires and a breadboard | Connecting it all | Lesson 3 |
{:class="table table-single"}

Strong neodymium magnets work best. The cheap black ceramic ones may not be strong enough to close a reed switch from a useful distance.

You also need **Thonny** (or `mpremote`) and MicroPython on your Pico. If you haven't set that up, [Learn MicroPython - The basics](/learn/micropython/) walks you through it.

## The pin plan

Here is every connection the finished buddy uses:

| Part | Connects to | Pico pin |
|---|---|---|
| Reed switch, one side | GP15 | Pin 20 |
| Reed switch, other side | GND | Pin 23 |
| OLED SDA | GP4 | Pin 6 |
| OLED SCL | GP5 | Pin 7 |
| OLED VCC | 3.3V | Pin 36 |
| OLED GND | GND | Pin 38 |
| Speaker positive (through the optional 1K resistor) | GP0 | Pin 1 |
| Speaker negative | GND | Any GND pin |
{:class="table table-single"}

Notice that the OLED is on **GP4 and GP5**. Kev's OLED post uses GP0 and GP1 for I2C, but the SAM speech library drives its speaker from GP0. Both can't have the same pin, so in this course the display moves to GP4 and GP5, which are a standard I2C0 pair on the Pico. If you follow the OLED post separately, remember that difference.

> **Important:** Power the OLED from the **3.3V** pin, not 5V. Most OLED modules expect 3.3V logic, and 5V might damage them.

## One file for pin numbers

In lesson 13 we move these numbers into a single `config.py`, so that porting the buddy to a different board only means editing that one file. Until then, each lesson sets the pins at the top of its program so you can see exactly what is going on.

## Try it Yourself

1. Check your Pico's pinout and find GP15 (pin 20), GP4 (pin 6), GP5 (pin 7) and GP0 (pin 1) with your finger.
2. Find two different GND pins, and one 3.3V pin.
3. Challenge: why do you think GND pins are scattered all around the Pico rather than in one place? ([Raspberry Pi Pico GPIO Mastery](/learn/micropython_gpio/) has a lesson on ground pins.)

## Common Issues

**Problem**: My display's pins are in a different order from the table.

**Solution**: Go by the labels printed on the module (VCC, GND, SCL, SDA), not by position.

**Why**: Manufacturers arrange the four pins in different orders, and the labels are the only reliable guide.
