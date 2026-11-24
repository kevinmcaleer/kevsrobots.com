---
layout: lesson
title: Printing the Parts
author: Kevin McAleer
type: page
cover: /learn/open_duck_mini/assets/banners/03_printing_the_parts.jpg
date: 2026-11-24
previous: 02_parts_list.html
next: 04_bus_servos_and_ids.html
description: Print the Open Duck Mini's parts, understand why the feet use both TPU
  and PLA, and fit heat-set inserts the right way
percent: 24
duration: 4
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

Time to warm up the printer. The duck's body is almost entirely 3D printed, so this is the longest stage of the build in clock time. The good news is your printer does the work while you get on with configuring servos in the next lesson.

---

## Start with the print guide

The settings for each part live in `docs/print_guide.md` in the [Open_Duck_Mini repo](https://github.com/apirrone/Open_Duck_Mini). Follow it for the print settings and which material each part needs. This course doesn't copy those settings, because the guide is kept up to date and this page isn't.

This lesson covers the *why*, so you can make good choices when your printer or filament behaves a little differently.

---

## Two materials: PLA and TPU

Almost everything is printed in **PLA**. It's stiff, easy to print and holds screws and inserts well. That's what you want for parts that carry load and need to stay accurate, such as the trunk, thighs and hips.

The feet are different. Each foot is two parts screwed together:

| Part | Material | Why |
|---|---|---|
| `foot_bottom_pla` | PLA | A rigid base that keeps the foot flat and holds its shape |
| `foot_bottom_tpu` | TPU | A flexible, grippy sole that touches the floor |
{:class="table table-single"}

TPU is a flexible filament. A soft sole grips the floor better than hard plastic and soaks up a little of the shock as each foot lands. That helps the walk the policy learned in simulation carry over to your floor.

TPU can be fussier to print than PLA, especially on printers with long filament paths. Print it slowly and check your printer's guidance for flexible filaments.

---

## Infill and mass

Printed parts are not solid. They have **infill**, a pattern inside that saves plastic and time. That changes how much each part weighs, and weight matters to a walking robot.

The Open Duck team set the simulated part masses from slicer estimates for exactly this reason. So stick to the print guide's infill settings. If your parts are much heavier or lighter, your duck won't match the duck the policy was trained on.

---

## Heat-set inserts

The duck uses lots of **M3 heat-set inserts**. These are small brass sleeves with a thread inside. You press them into a hole in the plastic with a hot soldering iron. The plastic melts around the knurled outside, then cools and locks the insert in place. Now your screws bite into metal, not plastic, so you can take the duck apart and put it back together without wearing the holes out.

How to fit one:

1. Use a soldering iron, ideally with a tip made for inserts.
2. Sit the insert on the hole, straight.
3. Let the iron's heat push it in gently. Don't force it.
4. Stop when it's flush with the surface, and keep it straight as it goes in.
5. Let it cool before you put any screw in.

You'll fit inserts in the trunk, the feet, the leg spacers and the head. The assembly guide says which holes get them, and we'll call them out in the assembly lessons.

---

## Try it Yourself

1. Print one small part first and test-fit an M3 insert and screw in it. Get a feel for it before you do the big parts.
2. Print the feet early. If your printer struggles with TPU, you'll know before the rest of the duck is done.
3. Weigh a printed part, if you have kitchen scales, and compare it with what your slicer estimated.

## Common Issues

**Problem**: An insert went in crooked.
**Solution**: Reheat it with the iron and straighten it while the plastic is soft.
**Why**: A crooked insert makes the screw pull the parts out of line.

**Problem**: The TPU strings or jams.
**Solution**: Slow the print right down and follow your printer's advice for flexible filament.
**Why**: Soft filament can buckle in the filament path when pushed too fast.

**Problem**: A part is much heavier than expected.
**Solution**: Check your infill and wall settings against the print guide.
**Why**: The simulated duck assumes certain masses. A heavy duck may not walk with the pretrained policy.
