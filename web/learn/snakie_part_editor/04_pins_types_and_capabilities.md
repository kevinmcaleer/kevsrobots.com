---
layout: lesson
title: Pins, Types and Capabilities
author: Kevin McAleer
type: page
cover: /learn/snakie_part_editor/assets/banners/04_pins_types_and_capabilities.jpg
date: 2026-11-21
previous: 03_board_size_and_details.html
next: 05_holes_components_and_labels.html
description: Add and place pins in the Snakie Part Editor and describe each one with
  a type, capabilities and signals so Snakie can wire and check it
percent: 35
duration: 6
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

By the end of this lesson you will be able to add pins to a part and set each pin's name, type, capabilities and I2C signal so Snakie can wire and check them.

Pins are the heart of a part. Draw them carefully and Snakie can colour the wires, check your circuit and even fill in your code. This lesson is the longest in the course, and the most important.

---

## What the sensor needs

Our breakout has six pins in a row along the bottom edge.

| # | Name | What it does | Type | Capabilities |
|---|---|---|---|---|
| 1 | VIN | Power in | Power | none |
| 2 | GND | Ground | Ground | none |
| 3 | SCL | I2C clock | IO | I²C |
| 4 | SDA | I2C data | IO | I²C |
| 5 | GPIO1 | Interrupt output | IO | Digital |
| 6 | XSHUT | Shutdown input | IO | Digital |
{:class="table table-single"}

The starter board has four pins, so you need to change those four and add two.

---

## Turn on the grid

Header pins sit 2.54 mm apart. Let the editor do the measuring.

1. In the bottom right of the canvas, click the **grid** button to show the 2.54 mm grid.
2. Click the **snap** button next to it, so it reads **Snap to the grid: on**.

Now anything you drag lands on a grid point.

---

## Add two pins

1. In **Layers**, find the **Pins** row and click its **＋** button. The tip says *Click the board to add a pin*.
2. Click the board twice. Two pins called P4 and P5 appear.
3. Click **Select** in the toolbar to put the pin tool away.

---

## Describe each pin

Click **VCC** in the Layers list. The Inspector shows the pin.

![A pin selected with its settings in the Inspector](/learn/snakie_part_editor/assets/pins.jpg){:class="img-fluid w-100"}

Work down the Inspector:

1. **Name**: change `VCC` to `VIN`.
2. **Type**: Power. The pad turns red.
3. **Board pin #**: `1`. This is the number printed next to the pin.
4. **Pad shape**: Round.

Now do the same for the other five, using the table above. For the IO pins there's more to set:

- **GPIO** is for microcontroller boards. Leave it empty on a sensor.
- **Capabilities** are the tick boxes: Digital, PWM, ADC, SPI, I²C and UART. Tick **I²C** for SCL and SDA, and untick the rest. Tick **Digital** for GPIO1 and XSHUT.
- When you tick I²C, an **I2C signal** box appears. Choose **SCL** for the clock pin and **SDA** for the data pin.

That last step is easy to miss. It records which pin is the clock and which is the data, so Snakie can label them and match them to the board's I2C pins.

| Type | Pad colour | Use it for |
|---|---|---|
| Power | Red | VIN, VCC, 3V3, 5V |
| Ground | Dark grey | GND |
| IO | Gold | Anything that carries a signal |
| Other | Neutral | Pins with no electrical job, such as NC |
{:class="table table-single"}

---

## Line the pins up

Drag each pin to the bottom edge of the board, one grid square apart, in order from VIN to XSHUT. Snapping keeps them level.

For exact placement, type into the **x** and **y** boxes at the bottom of the Inspector. Remember these are fractions of the board. Our pins all have `y` of about `0.86`, and `x` values that step from `0.246` to `0.754`.

As soon as a pin is nearer the bottom edge than any other, its label swings round to hang below the board. You don't need to rotate it. If a label ever points the wrong way, the **Rotation** buttons in the Inspector turn it by 90 degrees.

> Tip: hold **Ctrl** while dragging to snap to the grid, or **Shift** to move freely, whichever way the snap button is set.

---

## Pad shapes

| Pad shape | Looks like | Use it for |
|---|---|---|
| Square | A square pad with a hole | Pin 1 on many boards |
| Round | A round pad with a hole | Ordinary header pins |
| Header hole | A plain hole | Unpopulated headers |
| Castellated | A half-hole on the board edge | Boards like the Pico |
| SMD pad (no hole) | A flat pad | Surface-mount test points |
| Octagonal (servo) | An eight-sided pad | Servo headers |
| Pogo contact (sprung) | A sprung contact | Boards that press onto another board |
{:class="table table-single"}

---

## Try it Yourself

1. Add two pins and set up all six from the table.
2. Line them up along the bottom edge on the grid.
3. Click the **#** column heading in the pin list to sort by board pin number. Check they read 1 to 6.
4. Look at the labels under SCL and SDA. Each has a small blue tag showing its I2C signal.
5. Save.

---

## Common Issues

**Problem**: Clicking the board keeps adding pins.
**Solution**: Click **Select** in the toolbar.
**Why**: The pin tool stays active so you can add several pins in a row.

**Problem**: There's no GPIO box for VIN or GND.
**Solution**: Nothing to fix.
**Why**: Power and ground pins aren't GPIOs, so the Inspector hides the box and the capabilities.

**Problem**: The labels are piled up on the left of the board.
**Solution**: Select each pin and click **Rotate** until its label hangs below.
**Why**: A pin that has been rotated by hand keeps that direction, even after you move it.

---

## Check your understanding

{% quiz %}
? What does a pin's type control?
- [ ] How big the pad is
- [x] Its colour, and how Snakie treats it when checking the circuit
- [ ] The name printed next to it
> Power is red, ground is dark, IO is gold, and the electrical checks rely on them.

? You tick I²C on the SDA pin. What else should you set?
- [ ] The GPIO number
- [x] The I2C signal, to SDA
- [ ] The pad shape, to castellated
> The signal records which I2C wire this pin is.

? What spacing does the editor's grid use?
- [ ] 1 mm
- [ ] 5 mm
- [x] 2.54 mm
> 2.54 mm, or 0.1 inch, is the standard breadboard and header pitch.
{% endquiz %}
