---
title: The Blocks Workspace
description: Find your way around the Snakie Blocks workspace, the toolbox and the split view that shows blocks and Python side by side
layout: lesson
type: page
cover: /learn/snakie_blocks/assets/banners/01_the_blocks_workspace.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to find your way around the Blocks workspace and switch to the split view to see blocks and Python together.

Before we build anything, let's look at where everything lives.

---

## Open the Blocks workspace

At the top of Snakie there are four workspaces: **Blocks**, **Code**, **Electronics** and **Build**. Click **Blocks**.

![The Blocks workspace in Snakie](/learn/snakie_blocks/assets/blocks_workspace.jpg){:class="img-fluid w-100"}

Here's what's on the screen:

| Where | What it is |
|---|---|
| Left | The **toolbox**, a list of drawers full of blocks |
| Middle | The **canvas**, where you build your program |
| Bottom | The **console**, where your program prints and errors appear |
| Right | The **board** and the **instruments**, such as the turtle's drawing |
{:class="table table-single"}

## The toolbox

Click a drawer name and its blocks slide out. Drag one onto the canvas to use it.

| Drawer | What's in it |
|---|---|
| Turtle | A pen that draws on screen, with no hardware needed |
| Hardware | LEDs, buttons, PWM, servos, buzzers and analogue readings |
| Instruments | Send readings to Snakie's plotter, oscilloscope and multimeter |
| My parts | Blocks for the parts on your own breadboard |
| Wait | The two pause blocks |
| Control | forever, repeat, if, for each |
| Logic, Maths, Text, Lists | The everyday building blocks |
| Variables, Functions | Naming things, and naming groups of steps |
| Python | Grey blocks that hold code you type yourself |
{:class="table table-single"}

Turtle is first on purpose. It's the only drawer that needs nothing plugged in, and drawing a square is a great first ten minutes.

## The split view

Look at the workspace switcher again. Between **Blocks** and **Code** there's a small **dot**. Click it and you get both at once: blocks on the left, Python on the right.

![The split view with blocks and Python side by side](/learn/snakie_blocks/assets/split_view.jpg){:class="img-fluid w-100"}

This is the most important screen in the course. **The thing on the right is what the thing on the left means.** Change a number in a block and the Python changes. Change the Python and the block changes. They are two views of one program.

You can drag the divider between the panes to give either side more room.

## Try it here

This box starts in the split view. Use the **Code**, **Split** and **Blocks** buttons to switch, then change `4` to `6` in the blocks and watch the Python.

{% capture snakie_code %}
import turtle
for _ in range(4):
    turtle.forward(100)
    turtle.right(90)
{% endcapture %}
{% include snakie.html code=snakie_code name="square.py" view="split" %}

## Getting rid of blocks

Made a mess? Drag a block to the **bin** at the bottom right of the canvas, or select it and press <kbd>Delete</kbd>. <kbd>Ctrl</kbd>+<kbd>Z</kbd> undoes your last change (<kbd>⌘</kbd>+<kbd>Z</kbd> on a Mac).

To copy a block and everything inside it, select it and press <kbd>Ctrl</kbd>+<kbd>D</kbd> (<kbd>⌘</kbd>+<kbd>D</kbd> on a Mac).

## What block is this?

At the bottom left of the canvas is a box that says **What block is this? Drop one here.** Drop any block on it and Snakie opens a help page about that block, then hands the block straight back.

---

## Try it Yourself

1. Open every drawer in the toolbox and see what's inside.
2. Drag a block out, then throw it in the bin.
3. Switch between Blocks, the split view and Code using the switcher at the top.
4. Drop a block on **What block is this?** and read its help page.

## Common Issues

- **Problem**: I can't see the Python.
- **Solution**: Click the dot between **Blocks** and **Code** at the top of the window.
- **Why**: Blocks on its own hides the code pane. The dot is the halfway stop that shows both.

- **Problem**: A drawer is missing blocks I've seen in a screenshot.
- **Solution**: Turn on **Show advanced blocks** under the toolbox.
- **Why**: Snakie starts with the simple set. We'll meet the advanced ones in a later lesson.

## Check your understanding

{% quiz %}
? What does the pane on the right of the split view show?
- [ ] A simplified preview of the program
- [x] The real MicroPython the blocks wrote
- [ ] The console output
> The Python on the right is the actual program that runs. The blocks are another way of looking at it.

? Which drawer needs no hardware at all?
- [x] Turtle
- [ ] Hardware
- [ ] My parts
> The turtle draws on screen, so it works anywhere Snakie does.

? How do you see blocks and Python at the same time?
- [ ] Open two windows
- [x] Click the dot between Blocks and Code
- [ ] Press Run
> The dot is the split view: blocks on the left, Python on the right.
{% endquiz %}
