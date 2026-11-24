---
layout: course_overview
title: MicroPython Projects - Make It Show, Talk and Connect
author: Kevin McAleer
cover: /learn/micropython_projects/assets/cover.jpg
date: 2026-11-24
description: Take your MicroPython beyond blinking LEDs. Build one desk gadget, a
  status buddy, that senses a door with a reed switch, shows it on an OLED screen,
  says it out loud with SAM speech, and reports to Arduino Cloud over Wi-Fi. Then
  shrink it onto a tiny Seeed XIAO.
level: beginner
course_duration: 1h 18m
lesson_count: 17
first_lesson: 00_intro.html
quiz_total: 19
video: K0X4W09tiEA
outcomes:
- Wire a reed switch and read it reliably with a pull-up, an interrupt and debouncing
- Connect an SSD1306 OLED over I2C and draw text, shapes and a live status screen
- Make a Pico talk with SAM, and change its voice
- Connect a Pico W to Wi-Fi and push a value to Arduino Cloud
- Keep every pin number in one config file so a project moves between boards easily
- Port the whole gadget to a Seeed XIAO
prerequisites:
- Learn MicroPython - The basics
- Raspberry Pi Pico with MicroPython - GPIO Mastery
you_need:
- A Raspberry Pi Pico W (a plain Pico works for everything except the Wi-Fi lessons)
- A reed switch (normally open) and a neodymium magnet
- A 0.96 inch SSD1306 I2C OLED display (128x64)
- A small speaker or piezo buzzer, and an optional 1K resistor
- Jumper wires and a breadboard
- Thonny, or mpremote, on your computer
- A free Arduino Cloud account (lesson 12)
- Optional, a Seeed XIAO RP2040, RP2350 or ESP32-S3 (lesson 13)
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
