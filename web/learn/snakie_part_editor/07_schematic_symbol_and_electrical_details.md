---
layout: lesson
title: Schematic Symbol and Electrical Details
author: Kevin McAleer
type: page
cover: /learn/snakie_part_editor/assets/banners/07_schematic_symbol_and_electrical_details.jpg
date: 2026-11-21
previous: 06_tracing_a_board_photo.html
next: 08_drivers_and_built_in_help.html
description: Lay out the schematic symbol for your Snakie part, record its I2C address
  and tell Snakie its voltage limits so circuits can be checked
percent: 56
duration: 5
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

So far you've described what the part looks like. Now you'll describe how it **behaves**: its circuit symbol, its I2C address and the voltage it's happy with. This is the information that lets Snakie catch mistakes before they cost you a sensor.

---

## The schematic symbol

Click **Schematic** in the top bar.

![The Schematic view with the pad to pin table](/learn/snakie_part_editor/assets/schematic.jpg){:class="img-fluid w-100"}

On the right is the symbol: a box with the part's name, the manufacturer, and a pin on each side. On the left is the **Pad ↔ pin table**, with one row per pin.

| Column | What it sets |
|---|---|
| **side** | Which side of the box the pin is on: left, right, top or bottom |
| **order** | Its position along that side. Lower numbers come first |
{:class="table table-single"}

There's a loose convention for symbols that makes circuits easier to read:

- Inputs and control pins on the **left**
- Outputs and buses on the **right**
- Power at the top and ground at the bottom

For our sensor, put SCL, SDA and GPIO1 on the right and XSHUT on the left. The symbol updates as you choose.

> The symbol doesn't have to match the physical board. It's a diagram, laid out for clarity.

Click **Breadboard** to go back.

---

## The I2C address

Every I2C device answers to an address. The VL53L0X uses `0x29`.

1. Scroll the side panel to **I²C**.
2. Type `0x29` into **Add address** and click **Add**.

The address appears as a chip, and Snakie recognises it: next to it you'll see *VL53L0X / VL53L1X ToF*.

Why bother? Snakie's **I²C-detect** instrument scans the bus on your real board. When it finds something at `0x29`, it can now suggest your part by name. A chip with two possible addresses, such as the BME280 at `0x76` or `0x77`, gets both.

---

## Electrical behaviour

Scroll to **Electrical**. The **Model** box tells Snakie's circuit checker what kind of thing this is.

| Model | Use it for |
|---|---|
| Passive (no model) | Parts that are only drawn. Wires pass through |
| Source (battery / PSU) | Anything that supplies power |
| Regulator (LDO / buck) | Parts that turn one voltage into another |
| Consumer (load) | Sensors, displays, motors: anything that uses power |
| Resistor, Potentiometer, LED, Diode, Switch / button | The basic components |
{:class="table table-single"}

A sensor is a **Consumer (load)**. Choose it and more boxes appear.

![The Electrical section set up as a consumer](/learn/snakie_part_editor/assets/electrical.jpg){:class="img-fluid w-100"}

Fill them in from the datasheet:

| Field | Value | Meaning |
|---|---|---|
| Typical draw (A) | 0.019 | About 19 mA while measuring |
| Supply min (V) | 2.6 | The lowest voltage the breakout runs on |
| Supply max (V) | 5.5 | The highest voltage it survives |
| + terminal | VIN | The pin power goes in |
| − terminal | GND | The pin power returns through |
{:class="table table-single"}

Snakie now warns you when the part is wired to a rail outside 2.6 to 5.5 V. If you leave the supply boxes empty, there's no check.

---

## Rails

The **Rails** section is for pins that are joined by a trace **inside** the part. A motor driver with three GND pins is a good example: wire one and all three are grounded.

Our sensor has one of each pin, so it needs no rails. When you do need one, click **+ Rail** and choose the pins that belong together.

---

## Try it Yourself

1. Arrange the schematic symbol as described.
2. Add the address `0x29`.
3. Set the model to Consumer and fill in the five values.
4. Save.
5. Look up a part you own. What are its supply limits and typical current?

---

## Common Issues

**Problem**: The pin names overlap the part name on the symbol.
**Solution**: Move a pin to another side, or shorten the part's name.
**Why**: The symbol is a small box, and long names in the middle collide with pins on the sides.

**Problem**: The Pad ↔ pin table is empty.
**Solution**: Add pins in the Breadboard view first.
**Why**: The schematic only arranges pins that already exist.

**Problem**: I don't know my part's current draw.
**Solution**: Fill in just the supply limits, which are easy to find.
**Why**: The voltage check is the one that protects your hardware.

---

## Check your understanding

{% quiz %}
? Does the schematic symbol have to match the real board's layout?
- [ ] Yes, pin for pin
- [x] No, it's a diagram arranged to be easy to read
- [ ] Only for sensors
> The Breadboard view shows the real layout. The symbol is for clarity.

? Why record the part's I2C address?
- [ ] It sets the address on the chip
- [x] So Snakie's I2C scan can recognise the part by name
- [ ] It makes the part faster
> The address is information for Snakie. The chip's real address doesn't change.

? Which electrical model fits a sensor?
- [ ] Source
- [ ] Passive
- [x] Consumer
> A sensor uses power, so it's a load, with a supply range Snakie can check.
{% endquiz %}
