---
layout: lesson
title: Placing Parts
author: Kevin McAleer
type: page
cover: /learn/snakie_electronics/assets/banners/03_placing_parts.jpg
date: 2026-11-20
previous: 02_the_parts_library.html
next: 04_wiring_with_noodles.html
description: Choose a board, add a sensor, a motor driver, motors and a battery pack
  to a Snakie project, and arrange them on the canvas
percent: 32
duration: 6
quiz_total: 33
date_updated: 2026-11-24
navigation:
- name: Wire It Up - Electronics in Snakie
- content:
  - section: Overview
    content:
    - name: Wire It Up - Electronics in Snakie
      link: 00_intro.html
    - name: The Electronics Workspace
      link: 01_the_electronics_workspace.html
  - section: Parts
    content:
    - name: The Parts Library
      link: 02_the_parts_library.html
    - name: Placing Parts
      link: 03_placing_parts.html
  - section: Wiring
    content:
    - name: Wiring with Noodles
      link: 04_wiring_with_noodles.html
    - name: The Schematic View
      link: 05_the_schematic_view.html
    - name: Checking Your Wiring
      link: 06_checking_your_wiring.html
  - section: From Wiring to Code
    content:
    - name: Drivers, Help and Code
      link: 07_drivers_help_and_code.html
    - name: Connections and Measurements
      link: 08_connections_and_measurements.html
    - name: Exporting and Sharing
      link: 09_exporting_and_sharing.html
  - section: Project
    content:
    - name: Project - An Obstacle-Avoiding Robot
      link: 10_project_obstacle_robot.html
    - name: Summary and Next Steps
      link: 11_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to place a board and parts on the canvas, rotate, rename, duplicate and delete them, and arrange them ready for wiring.

Time to put the robot's parts on the bench. There are six: a Pico, a distance sensor, a motor driver, two motors and a battery pack.

---

## Choose your board

The board selector at the top shows **Raspberry Pi Pico 2 W**. Click it to see the other boards Snakie knows.

![The board selector menu](/learn/snakie_electronics/assets/board_menu.jpg){:class="img-fluid w-100"}

Pick the board you own. The original Pico, the Pico W and the Pico 2 W all share the same pin layout, so everything in this course works on any of them. We'll use the Pico 2 W.

## Add the sensor

1. Search the Library for `sr04` and click **HC SR04**.
2. Click **+ Add to project** on the part's card.

![The HC SR04 placed beside the Pico](/learn/snakie_electronics/assets/placed_first.jpg){:class="img-fluid w-100"}

The sensor appears on the canvas, and **HC SR04** joins the Components list in the Browser. You can also drag a part straight from the Library onto the canvas.

Two other things happened. The **Help** button now shows a number, because the sensor has a help page. And a banner says **1 part needs a driver**. Leave it for now. Lesson 7 is all about it.

## A word about 5 volts

Snakie's HC SR04 is the 3.3 V kind, which can be wired straight to a Pico. If the module in your hand is the classic **5 V** version, it needs 5 V on VCC, and its Echo pin answers at 5 V too. A Pico's pins must never see more than 3.3 V.

The fix is a pair of resistors on the Echo wire, called a voltage divider, or simply buying the 3.3 V version (often sold as HC-SR04P or RCWL-1601). The sensor's help page in Snakie explains both.

## Select, rotate and tidy

Click the sensor. A dashed outline appears with a small toolbar above it.

![A selected part and its toolbar](/learn/snakie_electronics/assets/part_selected.jpg){:class="img-fluid w-100"}

| Button | What it does |
|---|---|
| ? | Opens the part's help page |
| Rotate | Turns the part 90° |
| Rename | Changes the label shown on the canvas |
| Duplicate | Makes a second copy |
| Delete | Removes the part from the project |
{:class="table table-single"}

Clicking a name in the Components list selects that part too, which is useful when the canvas gets busy.

## Add the rest

Add these the same way:

| Search for | Part | How many |
|---|---|---|
| `mx1508` | MX1508 | 1 |
| `TT` | Yellow TT Motor | 2 |
| `4×AA` or `battery` | 4×AA Battery Pack | 1 |
{:class="table table-single"}

Snakie drops each new part somewhere near the others. Parts are drawn at their real relative sizes, so the motors and the battery pack tower over the little Pico.

## Arrange the bench

A tidy layout makes wiring far easier. Drag the parts so the wires will be short and won't cross more than they have to:

- The **Pico** on the left
- The **sensor** above, where it will "look" forward
- The **motor driver** in the middle
- A **motor** to each side of it
- The **battery pack** near the driver

Then hide the Library with the arrow at its top right to make room, and click **Zoom to fit**.

![All six parts arranged on the canvas](/learn/snakie_electronics/assets/layout.jpg){:class="img-fluid w-100"}

The Components list now shows six entries. That's the whole robot, waiting to be wired.

> **Teachers**: If learners will build the circuit for real, check the label on each HC-SR04 module before it is wired. Many modules on sale are 5 V parts, and a Pico pin must never see more than 3.3 V, so a classroom set is best bought as the 3.3 V version or fitted with a voltage divider on the Echo wire. On screen, none of this can cause any harm.

---

## Try it Yourself

1. Rotate the battery pack so its terminals point towards the motor driver.
2. Rename the two motors `Left motor` and `Right motor`.
3. Duplicate the sensor, then delete the copy. Check the Components count goes to seven and back to six.

## Common Issues

**Problem**: A new part landed on top of another one.
**Solution**: Drag it clear. If you grab the wrong one, select the right one from the Components list first.
**Why**: New parts are placed near the existing ones, and big parts can overlap.

**Problem**: A message at the bottom offers help and covers part of the canvas.
**Solution**: Click **View help** to read it, or the **✕** to close it.
**Why**: Snakie tells you when a part you've just added has a help page.

**Problem**: I added the wrong part.
**Solution**: Select it and click the bin on its toolbar, or press <kbd>Ctrl</kbd>+<kbd>Z</kbd>.
**Why**: Adding a part is a step like any other, so it can be undone.

## Check your understanding

{% quiz %}
? Why does a classic 5 V HC-SR04 need care with a Pico?
- [ ] It's too heavy
- [x] Its Echo pin answers at 5 V and Pico pins are 3.3 V
- [ ] It uses too many pins
> A Pico pin must not see more than 3.3 V. Use a 3.3 V sensor or a voltage divider on Echo.

? How do you turn a part round on the canvas?
- [ ] Shake the mouse
- [ ] Delete it and add it again
- [x] Select it and click Rotate on its toolbar
> Each click of Rotate turns the part 90 degrees.

? Why arrange parts before wiring?
- [x] Shorter wires that cross less are easier to follow
- [ ] Snakie won't let you wire otherwise
- [ ] It changes the code
> Layout doesn't change the circuit, but it makes the diagram far easier to read.
{% endquiz %}
