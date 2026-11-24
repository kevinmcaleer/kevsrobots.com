---
layout: lesson
title: Making a Microcontroller Board
author: Kevin McAleer
type: page
cover: /learn/snakie_part_editor/assets/banners/11_making_a_microcontroller_board.jpg
date: 2026-11-21
previous: 10_giving_your_part_its_own_blocks.html
next: 12_sharing_your_parts.html
description: Turn a Snakie part into a microcontroller board with GPIO numbers so
  it appears in the board selector for your projects
percent: 84
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

By the end of this lesson you will be able to turn a part into a microcontroller board with GPIO numbers and see it in the board selector.

Sensors plug into a board. But what if the *board* is the thing that's missing? Maybe you've designed your own robot controller, or bought a new microcontroller that Snakie doesn't list yet.

In Snakie a board is just a part with one box ticked.

---

## What makes a board different

| | A sensor part | A board part |
|---|---|---|
| Where you find it | In the Library | In the Library **and** the board selector |
| How many in a project | As many as you like | One, the brain of the project |
| GPIO numbers on pins | Left empty | Filled in |
| Onboard LED | Usually none | Usually one |
{:class="table table-single"}

The **GPIO number** is the big one. It's the number you pass to `Pin()` in MicroPython, and it's how Snakie connects a wire on the breadboard to a line of code.

---

## Make a small board

Let's make a pretend board called Tiny Robot Brain.

1. Click **+ New part**.
2. Set **Name** to `Tiny Robot Brain`.
3. Click **Show** next to Details.
4. Set **Family** to `Microcontroller`.
5. Tick **This part is a microcontroller board**.

![The microcontroller board tick box in Details](/learn/snakie_part_editor/assets/board_flag.jpg){:class="img-fluid w-100"}

The starter pins already suit a board: VCC, GND, GP0 and GP1.

---

## GPIO numbers

Click **GP0**. In the Inspector:

| Field | Value | Meaning |
|---|---|---|
| Name | GP0 | The label printed on the board |
| Board pin # | 3 | Its physical position, counting round the board |
| GPIO | 0 | The number your code uses |
{:class="table table-single"}

Name and GPIO often look alike, but they aren't the same thing. Physical pin 4 on a Pico is **GP2**, with GPIO number **2**. The pin list in Layers has three columns for exactly this reason: the name, **#** for the board pin number and **GP** for the GPIO.

Then tick what each pin can do. If GP0 can do PWM and I2C, tick **PWM** and **I²C** and choose the I2C signal. The blocks and wiring tools use these to offer only pins that suit the job.

> Get the capabilities from the board's official pinout diagram. Guessing here leads to confusing bugs later.

---

## Board extras

A few settings in the **Board** section only make sense for a microcontroller.

| Setting | What it's for |
|---|---|
| **Onboard LED pin** | The pin the built-in LED is wired to |
| **Footprint** | A shared shape name, such as `xiao`. Boards with the same footprint plug into the same carrier boards |
{:class="table table-single"}

From **＋ Add** on the Components row you can also add a **Button**, such as BOOT or RESET, and an **LED**. If a user button is wired to a GPIO, give it that number so blocks can offer it.

---

## Pick your board

Click **Save**, then **Done**. Now open the board selector at the top of the Electronics workspace, the one that usually says *Raspberry Pi Pico 2 W*. Scroll down.

![The custom board in the board selector](/learn/snakie_part_editor/assets/board_selector.jpg){:class="img-fluid w-100"}

Your board is in the list. Choose it. If the project already has wires, Snakie asks before it swaps:

```
Swap to Tiny Robot Brain?

1 connection(s) will be kept — moved to the matching GPIO / rail where needed.

3 will be removed — no matching pin on Tiny Robot Brain:
  • 3V3 rail
  • GPIO 5
  • GPIO 4
```

Read that carefully. Snakie keeps every wire it can find a home for, by matching GPIO numbers and power rails. Our tiny board has no 3V3 pin and no GPIO 4 or 5, so those three wires go.

![The custom board on the canvas](/learn/snakie_part_editor/assets/custom_board.jpg){:class="img-fluid w-100"}

This is another reason to get the GPIO numbers right. They're what lets a project move from one board to another.

---

## Try it Yourself

1. Make the Tiny Robot Brain and save it.
2. Edit it and add four more pins: GP2 to GP5, with GPIO numbers 2 to 5.
3. Mark GP4 and GP5 as I²C, with signals SDA and SCL.
4. Add a 3V3 power pin.
5. Swap a wired project to your board again. How many wires survive this time?

---

## Common Issues

**Problem**: My board isn't in the board selector.
**Solution**: Tick **This part is a microcontroller board** in Details and save.
**Why**: Without the tick it's an ordinary part, listed only in the Library.

**Problem**: Swapping boards removed my wires.
**Solution**: Cancel the swap, add the missing pins to your board, then try again.
**Why**: A wire can only move to a pin with the same GPIO number or rail.

**Problem**: The code uses the wrong pin.
**Solution**: Check the pin's **GPIO** number, not its name or board pin number.
**Why**: MicroPython's `Pin()` takes the GPIO number.

---

## Check your understanding

{% quiz %}
? What turns a part into a board that appears in the board selector?
- [ ] Giving it more than ten pins
- [x] Ticking "This part is a microcontroller board"
- [ ] Naming it "board"
> One tick box in Details makes the difference.

? Which number does MicroPython's Pin() use?
- [ ] The board pin number
- [x] The GPIO number
- [ ] The pin's order in the Layers list
> Physical pin 4 on a Pico is GPIO 2. Code always uses the GPIO number.

? You swap boards and Snakie says three wires will be removed. Why?
- [ ] The wires were the wrong colour
- [ ] The new board is too small to draw them
- [x] The new board has no pin with a matching GPIO number or rail
> Wires move by GPIO number and power rail, so a missing pin means a lost wire.
{% endquiz %}
