---
layout: lesson
title: Turtle Blocks - Draw a Square
author: Kevin McAleer
type: page
cover: /learn/snakie_blocks/assets/banners/02_turtle_draw_a_square.jpg
date: 2026-11-20
previous: 01_the_blocks_workspace.html
next: 03_variables_maths_and_loops.html
description: Build your first Snakie blocks program by drawing a square with the turtle,
  then change it into other shapes
percent: 21
duration: 4
quiz_total: 36
date_updated: 2026-11-20
navigation:
- name: Snakie Blocks - From Blocks to Python
- content:
  - section: Overview
    content:
    - name: Snakie Blocks - From Blocks to Python
      link: 00_intro.html
    - name: The Blocks Workspace
      link: 01_the_blocks_workspace.html
  - section: Blocks Basics
    content:
    - name: Turtle Blocks - Draw a Square
      link: 02_turtle_draw_a_square.html
    - name: Variables, Maths and Loops
      link: 03_variables_maths_and_loops.html
    - name: Making Decisions
      link: 04_making_decisions.html
  - section: Blocks and Hardware
    content:
    - name: Hardware Blocks - LEDs and Buttons
      link: 05_leds_and_buttons.html
    - name: PWM, Servos and Buzzers
      link: 06_pwm_servos_and_buzzers.html
    - name: Functions
      link: 07_functions.html
    - name: My Parts
      link: 08_my_parts.html
  - section: From Blocks to Python
    content:
    - name: Reading the Python
      link: 09_reading_the_python.html
    - name: Simple and Advanced Blocks
      link: 10_simple_and_advanced_blocks.html
  - section: Project
    content:
    - name: Project - Reaction Timer
      link: 11_project_reaction_timer.html
    - name: Summary and Next Steps
      link: 12_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

Your first blocks program draws a square. No wires, no board.

---

## Meet the turtle

The turtle is a little arrow that carries a pen. Tell it to move and it draws a line behind it. Tell it to turn and it faces a new way. That's nearly all there is to it.

![The Turtle drawer](/learn/snakie_blocks/assets/toolbox_turtle.jpg){:class="img-fluid w-100"}

## Build it

1. Open the **Blocks** workspace and start a new file.
2. Open **Control** and drag **repeat 10 times** onto the canvas. Change `10` to `4`.
3. Open **Turtle** and drag **move forward 100** inside the repeat block.
4. Drag **turn right 90 degrees** underneath it, still inside the repeat.
5. Press **Run**.

![The square program as blocks](/learn/snakie_blocks/assets/blocks_square.jpg){:class="img-fluid w-100"}

The **Turtle** instrument opens on the right and the turtle draws your square.

> **Shortcut**: on an empty Blocks canvas, Snakie offers a **Draw a square** starter that builds this program for you.

## What the blocks wrote

Here's the same program in a live box. Look at the Python on the right.

{% capture snakie_code %}
import turtle
for _ in range(4):
    turtle.forward(100)
    turtle.right(90)
{% endcapture %}
{% include snakie.html code=snakie_code name="square.py" view="split" %}

Line by line:

- `import turtle` loads the turtle. Snakie added this for you the moment you used a turtle block
- `for _ in range(4):` is the **repeat 4 times** block
- `turtle.forward(100)` and `turtle.right(90)` are the two blocks inside it. They're indented, which is how Python shows they are *inside* the loop

> **Note**: the boxes on this page show the blocks and the Python, but they have nowhere to draw. Press Run here and you'll see the turtle's moves listed in the console as lines starting `SNK TURT`. To watch the drawing itself, click **Open in Snakie** under the box and press Run there.

## Why 90 degrees?

A square has four corners, and the turtle has to turn all the way round by the time it gets back: 360 degrees. So each turn is 360 ÷ 4 = 90.

That one sum unlocks every regular shape:

| Shape | Sides | Turn |
|---|---|---|
| Triangle | 3 | 120 |
| Square | 4 | 90 |
| Hexagon | 6 | 60 |
| Circle (nearly) | 36 | 10 |
{:class="table table-single"}

## Add some colour

Stars are fun because the turtle turns *more* than it needs to. Five points, turning 144 degrees each time:

{% capture snakie_code %}
import turtle
turtle.pencolor("#ff0000")
turtle.pensize(3)
for _ in range(5):
    turtle.forward(150)
    turtle.right(144)
{% endcapture %}
{% include snakie.html code=snakie_code name="star.py" view="split" %}

![A red star drawn by the turtle in Snakie](/learn/snakie_blocks/assets/star_run.jpg){:class="img-fluid w-100"}

---

## Try it Yourself

1. Change the square into a triangle, then a hexagon.
2. Make the star blue and thicker.
3. Use **pen up** and **pen down** to draw two squares with a gap between them.
4. Challenge: draw a circle. How many sides does it need before it looks round?

## Common Issues

**Problem**: The turtle draws one line and stops.
**Solution**: Make sure both turtle blocks are *inside* the repeat block's mouth.
**Why**: Only the blocks inside the repeat are repeated. A block underneath it runs once, afterwards.

**Problem**: Nothing appears when I press Run.
**Solution**: Check the Turtle instrument is open on the right. Press **Clear** on it and run again.
**Why**: The drawing goes to the Turtle instrument, not the console.

**Problem**: The shape doesn't close up.
**Solution**: Check that sides × turn = 360.
**Why**: The turtle must turn a full circle in total to end up facing the way it started.

## Check your understanding

{% quiz %}
? What Python does the "repeat 4 times" block write?
- [ ] `repeat(4)`
- [x] `for _ in range(4):`
- [ ] `while 4:`
> A repeat block is a `for` loop over `range(4)`.

? How far should the turtle turn at each corner of a hexagon?
- [ ] 90 degrees
- [x] 60 degrees
- [ ] 6 degrees
> 360 ÷ 6 sides = 60 degrees.

? Who added the `import turtle` line?
- [ ] You have to type it
- [x] Snakie, when you used a turtle block
- [ ] The Pico
> Blocks bring their own imports with them, at the top of the file.
{% endquiz %}
