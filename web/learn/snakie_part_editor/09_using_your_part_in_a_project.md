---
layout: lesson
title: Using Your Part in a Project
author: Kevin McAleer
type: page
cover: /learn/snakie_part_editor/assets/banners/09_using_your_part_in_a_project.jpg
date: 2026-11-21
previous: 08_drivers_and_built_in_help.html
next: 10_giving_your_part_its_own_blocks.html
description: Place your own Snakie part on the breadboard, wire it to a Pico, install
  its driver and find the blocks Snakie makes for it
percent: 70
duration: 5
quiz_total: 41
date_updated: 2026-11-24
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

By the end of this lesson you will be able to place and wire your own part in a project, install its driver, and find the blocks Snakie derives for it.

This is the payoff. Your part leaves the editor and goes to work in a project, exactly like one of the standard parts.

---

## Find it in the library

Click **Save**, then **Done**. Back in the Electronics workspace, the Library has a new section at the top: **My Parts**, marked **Your library**.

Click your part. A card opens below the list.

![Your part in the library with its detail card](/learn/snakie_part_editor/assets/library.jpg){:class="img-fluid w-100"}

| Button | What it does |
|---|---|
| **+ Add to project** | Puts the part on the breadboard |
| Pencil | **Edit part** in the Part Editor |
| Two squares | **Duplicate part**, a quick way to make a variation |
| Bin | **Delete part** |
{:class="table table-single"}

The card also previews the part, with **Board** and **Schematic** tabs.

---

## Place and wire it

1. Click **+ Add to project**. Your sensor appears next to the Pico.
2. Drag from each sensor pin to a Pico pin:

| Sensor pin | Pico pin |
|---|---|
| VIN | 3V3 |
| GND | GND |
| SCL | GP5 |
| SDA | GP4 |
{:class="table table-single"}

![The sensor wired to a Pico](/learn/snakie_part_editor/assets/wired.jpg){:class="img-fluid w-100"}

Look at what your careful work in lesson 4 bought you. The power wire is red and the ground wire is its own colour, because you set the pin **types**. The pin labels are the names you typed.

---

## The driver banner

A banner appeared at the top of the canvas as soon as you placed the part:

```
1 part needs a driver — connect a board     [Install drivers]
```

That's the driver you attached in the last lesson. Connect your board and click **Install drivers**. Snakie copies `vl53l0x.py` into the board's `lib` folder.

No board connected? The button waits until there is one. You can close the banner with the cross and carry on.

---

## Blocks for your part

Now for the clever part. Open the **Blocks** workspace and start a **New blocks program**. In the toolbox, open **My parts**. Your sensor has a drawer of its own.

![Blocks Snakie made for the part](/learn/snakie_part_editor/assets/derived_blocks.jpg){:class="img-fluid w-100"}

You didn't make these. Snakie **derived** them from the module name you gave the part, `vl53l0x`.

| Block | What it does |
|---|---|
| **the VL53L0X Distance Sensor (class VL53L0X)** | The sensor object itself |
| **do update on … with …** | Calls a method on the sensor |
| **read of …** | Reads a value from the sensor |
{:class="table table-single"}

Snakie knows the module, but it has to guess the class name. It uses the module name in capitals, `VL53L0X`, which is right more often than not. The guess is in an editable box on the block, and so are the method names. If your driver calls its class `Sensor`, type that in.

These blocks are a little rough. They work, but **do update on** isn't how you'd describe measuring a distance. In the next lesson you'll replace them with blocks that read like plain English.

---

## Edit and refresh

Spotted a mistake in your part? Click it in the library, click the pencil, fix it and save.

If you change the part's files by hand, click the **Refresh library** button in the Library toolbar to reload them from disk.

---

## Try it Yourself

1. Place your sensor and wire the four pins.
2. Open the **Connections** list at the bottom of the canvas and check all four wires.
3. Click **Help** at the top right of the Electronics workspace. Your mini-help is there.
4. Open Blocks and find your part's drawer under **My parts**.
5. Think back to lesson 7. Which of the Pico's power pins are inside the 2.6 to 5.5 V range you set? Would a 9 V battery be?

---

## Common Issues

**Problem**: My part isn't in the library.
**Solution**: Save in the editor, then click **Refresh library**.
**Why**: The library lists what's on disk, and a part that was never saved isn't there.

**Problem**: **Install drivers** can't be clicked.
**Solution**: Connect a board first.
**Why**: Drivers are copied to the board, so there has to be one.

**Problem**: **My parts** in Blocks is empty.
**Solution**: Add the part to the project in Electronics first.
**Why**: The drawer only shows parts that are on your breadboard.

**Problem**: The derived blocks use the wrong class name.
**Solution**: Type the right one into the box on the block.
**Why**: Snakie can see the module name but can only guess the class.

---

## Check your understanding

{% quiz %}
? Why is the wire to VIN drawn in red?
- [ ] You chose red for it
- [x] The pin's type is Power
- [ ] All wires to sensors are red
> Wire colours come from the pin types you set in the editor.

? Where do the blocks in the My parts drawer come from, if you haven't written any?
- [ ] They're downloaded from the internet
- [x] Snakie derives them from the part's module name
- [ ] They're copied from the Standard Parts
> Every part with a module gets an object block, a call block and a read block.

? The derived block guesses the class name VL53L0X. How does Snakie choose it?
- [ ] It reads the driver file
- [x] It uses the module name in capitals
- [ ] It asks the board
> It's a guess, shown in an editable box so you can correct it.
{% endquiz %}
