---
layout: lesson
title: Introduction
author: Kevin McAleer
type: page
cover: /learn/micropython_projects/assets/banners/00_intro.jpg
date: 2026-11-24
next: 01_meet_the_status_buddy.html
description: Meet the status buddy, a desk gadget that senses a door, shows it on
  an OLED, speaks with SAM and reports to Arduino Cloud
percent: 5
duration: 4
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

Ahoy there makers! Blinking an LED is a brilliant first step, but sooner or later you want your project to do something you can see, hear and check from your phone. This course takes you there, one small gadget at a time.

Watch this first, then come back and let's build.

{% include youtubeplayer.html id="K0X4W09tiEA" %}

---

## Overview

You already know how to read a pin and flash an LED. Now we give a Pico three new abilities: to **show** information on a tiny OLED screen, to **talk** with a speech engine from 1982, and to **connect** to the internet.

Rather than three separate demos, we build one thing that grows in every section. It is a desk gadget called the **status buddy**. A reed switch and a magnet tell it whether a door (or a lid, or a drawer) is open. The buddy then shows the answer on its screen, says it out loud, and reports it to Arduino Cloud so you can check from your phone.

```mermaid
flowchart LR
    A["Sense<br/>reed switch"] --> B["Show<br/>OLED screen"]
    B --> C["Talk<br/>SAM speech"]
    C --> D["Connect<br/>Wi-Fi + Arduino Cloud"]
    D --> E["Shrink it<br/>XIAO"]
```

## Course Content

- **Getting started**: what the status buddy is, the parts list and the pin plan
- **Sense**: reed switches and magnets, reading the door, interrupts and debouncing
- **Show**: wiring an SSD1306 OLED, drawing text and shapes, and a live status screen
- **Talk**: meeting SAM, then giving the buddy a voice
- **Connect**: Pico W Wi-Fi, then pushing the door state to Arduino Cloud
- **Shrink it**: moving the whole gadget onto a Seeed XIAO
- **Putting it together**: the full program, a troubleshooting guide and what to build next

## Key Results

After completing this course, you will:

- Read a reed switch reliably, using a pull-up, an interrupt and debouncing
- Connect an OLED over I2C and draw text, shapes and a status screen
- Make your Pico speak, and give it a different voice
- Connect a Pico W to Wi-Fi and send a value to Arduino Cloud
- Keep pin numbers in one `config.py` so a project can move to a new board
- Have a finished desk gadget you can put on a real door

## What you'll need

- A Raspberry Pi Pico W. A plain Pico does everything except the Wi-Fi lessons
- A normally open reed switch and a neodymium magnet
- A 0.96 inch SSD1306 I2C OLED display (128x64)
- A small speaker or piezo buzzer, and an optional 1K resistor
- Jumper wires and a breadboard
- Thonny (or `mpremote`) to copy files to the Pico
- A free [Arduino Cloud](https://cloud.arduino.cc/) account, for lesson 12
- Optional, a Seeed XIAO board, for lesson 13

This course follows on from two others. If a line of code looks unfamiliar, check these first:

- [Learn MicroPython - The basics](/learn/micropython/)
- [Raspberry Pi Pico with MicroPython - GPIO Mastery](/learn/micropython_gpio/)

## How the course works

**One gadget, four stages.** Every section adds a stage to the status buddy. Each lesson tells you which stage you are on, and the finished program is in lesson 14.

**Complete programs.** Every code block you are told to run is a whole program: imports, setup and loop. Save it to the Pico, run it, and it should work. Short snippets that only make sense inside a bigger program say so.

**Extra files.** Three parts of this course use code that is not built into MicroPython: the `ssd1306.py` display driver, the SAM speech library, and the Arduino Cloud client. Each lesson tells you where to get it and how to copy it onto the Pico.

**Secrets stay secret.** Wi-Fi names, passwords and Arduino Cloud keys always live in a separate `secrets.py` file. Examples only ever show placeholders.

**Quizzes.** The Show, Talk and Connect sections each end with a short quiz. Your score is kept in your browser.

**Mistakes are normal.** Most lessons have a Common Issues list for the things that trip people up, and lesson 15 gathers the big ones in one place.

Ready? Let's meet the buddy.
