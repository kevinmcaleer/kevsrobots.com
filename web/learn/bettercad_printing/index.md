---
layout: lesson
title: Print It Right with BetterCad
author: Kevin McAleer
type: page
cover: /learn/bettercad_printing/assets/banners/00_intro.jpg
date: 2026-11-20
next: 01_set_up_your_printer.html
description: Use the BetterCad Print panel to check, calibrate and export parts that
  print cleanly and fit first time
percent: 8
duration: 4
quiz_total: 36
date_updated: 2026-11-20
navigation:
- name: Print It Right with BetterCad
- content:
  - section: Overview
    content:
    - name: Print It Right with BetterCad
      link: 00_intro.html
  - section: Will It Print
    content:
    - name: Set Up Your Printer
      link: 01_set_up_your_printer.html
    - name: Overhangs and Supports
      link: 02_overhangs_and_supports.html
    - name: Thin Walls
      link: 03_thin_walls.html
    - name: Choosing an Orientation
      link: 04_choosing_an_orientation.html
  - section: Will It Fit
    content:
    - name: Holes That Fit
      link: 05_holes_that_fit.html
    - name: Fits Between Parts
      link: 06_fits_between_parts.html
    - name: Where Parts Meet
      link: 07_where_parts_meet.html
  - section: Out to the Slicer
    content:
    - name: Material and Weight
      link: 08_material_and_weight.html
    - name: Exporting for Your Slicer
      link: 09_exporting_for_your_slicer.html
  - section: Project
    content:
    - name: Project - A Box With a Lid
      link: 10_project_box_with_a_lid.html
    - name: Summary and Next Steps
      link: 11_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

## Overview

Designing a part is half the job. The other half is getting it off the printer in one piece, the right size, without a bin full of failed attempts.

[BetterCad](https://bettercad.kevsrobots.com) has a **Print panel** built for exactly that. It knows the size of your printer's bed, spots overhangs and thin walls, suggests which way up to print, and remembers how *your* printer behaves so that holes and joints come out right. This course works through the panel from top to bottom, then puts it to use on a box with a lid that closes with a satisfying fit.

![The Print panel open beside the bracket sample](/learn/bettercad_printing/assets/print_panel.jpg){:class="img-fluid w-100"}

## Course Content

- Choosing your printer and checking a model fits the bed
- Finding overhangs, and designing them out
- Finding walls too thin to print
- Picking an orientation for strength, finish and fewer supports
- Calibrating screw holes to your printer with a test plate
- Press, sliding and loose fits between printed parts
- Checking where parts meet, and fixing joints that are too tight
- Materials, infill and the weight estimate
- STL, 3MF and STEP, and which to send where
- Project: tune and print a box with a lid

## Key Results

After this course you will be able to:

- Tell before you print whether a part needs supports
- Print a hole that an M3 screw slides into, first time
- Make two printed parts press together, slide or drop in, on purpose
- Save your printer's settings once and reuse them in every model
- Export the right file at the right resolution for your slicer

## What you'll need

- A computer with a recent browser
- [BetterCad](https://bettercad.kevsrobots.com). It's free and needs no account
- A 3D printer, slicer and some filament. PLA is ideal
- A few M3 screws for the hole test. M2 to M6 all work
- About 10 g of filament and under an hour of printing for the two calibration prints

You can read the course without a printer, but the calibration lessons only pay off when you print the test pieces.

## How the course works

Lessons are short and each covers one part of the Print panel. Steps to follow are numbered, keys look like <kbd>K</kbd>, and names of buttons and panels are in **bold**. Most lessons use the samples built into BetterCad, so there's nothing to draw until the project.

To open the Print panel, click the account menu at the top right and choose **Print settings**. It opens on the left, where the Browser normally sits. Click **Browser** at the top of the panel to go back.

> **BetterCad is in alpha.** Buttons move and features arrive often, so your screen may differ a little from the screenshots.

## Prerequisites

You should be able to find your way around BetterCad and make a simple part. [Learn BetterCad](/learn/bettercad/) covers everything you need, and lesson 10 of that course is a quick preview of this one.

## Try it Yourself

1. Open the **Bracket** sample from the start card, or [open it directly](https://bettercad.kevsrobots.com/#/sample/bracket).
2. Open **Print settings** from the account menu and scroll through the whole panel. How many headings does it have?
3. With nothing selected, look at Properties on the right. What does BetterCad think the bracket weighs?

## Check your understanding

{% quiz %}
? Where is the Print panel?
- [ ] On the timeline
- [x] Under Print settings in the account menu
- [ ] In the Export dialog
> It opens on the left, in place of the Browser.

? What does this course help you avoid?
- [ ] Learning to sketch
- [x] Failed prints and parts that don't fit
- [ ] Using a slicer
> You still slice as normal. The checks happen before you get that far.

? Do you need an account to use the Print panel?
- [x] No
- [ ] Yes
- [ ] Only for calibration
> BetterCad never needs an account. Signing in only adds cloud saving.
{% endquiz %}
