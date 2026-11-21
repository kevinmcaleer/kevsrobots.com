---
layout: lesson
title: Make Your Own Parts and Boards in Snakie
author: Kevin McAleer
type: page
cover: /learn/snakie_part_editor/assets/banners/00_intro.jpg
date: 2026-11-21
next: 01_what_a_part_is.html
description: Learn to add your own sensors, breakouts and microcontroller boards to
  Snakie with the Part Editor, drivers and custom blocks
percent: 7
duration: 4
quiz_total: 41
date_updated: 2026-11-21
navigation:
- name: Make Your Own Parts and Boards in Snakie
- content:
  - section: Overview
    content:
    - name: Make Your Own Parts and Boards in Snakie
      link: 00_intro.html
    - name: What a Part Is
      link: 01_what_a_part_is.html
  - section: Drawing the Part
    content:
    - name: A Tour of the Part Editor
      link: 02_tour_of_the_part_editor.html
    - name: Board Size and Details
      link: 03_board_size_and_details.html
    - name: Pins, Types and Capabilities
      link: 04_pins_types_and_capabilities.html
    - name: Holes, Components and Labels
      link: 05_holes_components_and_labels.html
    - name: Tracing a Board Photo
      link: 06_tracing_a_board_photo.html
  - section: Making It Smart
    content:
    - name: Schematic Symbol and Electrical Details
      link: 07_schematic_symbol_and_electrical_details.html
    - name: Drivers and Built-In Help
      link: 08_drivers_and_built_in_help.html
    - name: Using Your Part in a Project
      link: 09_using_your_part_in_a_project.html
    - name: Giving Your Part Its Own Blocks
      link: 10_giving_your_part_its_own_blocks.html
  - section: Boards and Sharing
    content:
    - name: Making a Microcontroller Board
      link: 11_making_a_microcontroller_board.html
    - name: Sharing Your Parts
      link: 12_sharing_your_parts.html
    - name: Summary and Next Steps
      link: 13_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

## Overview

Snakie ships with around eighty parts: Picos, sensors, motors, batteries. Sooner or later you'll hold a board in your hand that isn't on the list. That's where the fun starts, because Snakie lets you **make the part yourself**.

In this course you'll build one complete part from a blank board: a **VL53L0X laser distance sensor** breakout. By the end it will sit in your library next to the standard parts. You'll be able to drop it on the breadboard, wire it, install its driver with one click, and drag its very own blocks into a program.

{% include youtubeplayer.html id="QVe9meIHZtk" %}

New to Snakie? Watch Kev's introduction above first, then come back.

---

## Course Content

- What a part really is: a folder with a `parts.yml` file in it
- A tour of the Part Editor and its three views
- Board size, colour and the details that make a part searchable
- Pins: names, types, capabilities and the 2.54 mm grid
- Mounting holes, component shapes and labels
- Tracing a real board photo, with background removal
- The schematic symbol, I2C addresses and electrical limits
- Attaching a MicroPython driver and bundled help
- Using your part in a project, and the blocks Snakie makes for it
- Writing a `blocks.yml` so your part gets its own blocks
- Turning a part into a microcontroller board
- Sharing parts as a library

---

## Key Results

After this course you will be able to:

- Create a part in the Part Editor and save it to your **My Parts** library
- Place and describe pins so Snakie can wire, check and label them
- Turn a photo of a board into a clean, correctly sized part
- Attach a driver so Snakie offers to install it when the part is used
- Write a small `blocks.yml` that adds blocks for your part
- Make a custom microcontroller board appear in the board selector
- Package your parts as a library other people can install

---

## What you'll need

- **Snakie desktop app**. You can open the Part Editor in the browser version to look around, but saving a part needs the desktop app, because parts are saved as files on your computer.
- A board you'd like to add. We use a VL53L0X breakout, and you can follow along without owning one.
- Optional: a ruler or calipers, and a photo of your board taken from straight above.

This course was written with Snakie **0.92.0**. Snakie changes quickly, so a button may have moved by the time you read this.

---

## How the course works

Each lesson adds one thing to the same part, so do them in order. Screenshots show exactly what to expect, there's a **Try it Yourself** task in every lesson, and a short quiz at the end checks that the idea has stuck.

This is an intermediate course. You should already be comfortable wiring parts in the Electronics workspace. If not, start here:

- [Getting Started with Snakie](/learn/snakie_getting_started/)
- [Wire It Up - Electronics in Snakie](/learn/snakie_electronics/)
- [Snakie Blocks - From Blocks to Python](/learn/snakie_blocks/), for the blocks lessons
- [Coding Faster in Snakie](/learn/snakie_code_tools/), the other half of the Snakie Power User pathway

Let's make a part.

---

## Check your understanding

{% quiz %}
? Which version of Snakie do you need to save a part you've made?
- [ ] The browser version
- [x] The desktop app
- [ ] Either one
> Parts are saved as files in a folder on your computer, which only the desktop app can write.

? What will you have built by the end of the course?
- [ ] A new MicroPython firmware
- [x] A complete VL53L0X distance sensor part with a driver and its own blocks
- [ ] A printed circuit board
> One part, built up lesson by lesson, plus a small custom microcontroller board.
{% endquiz %}
