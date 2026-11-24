---
layout: lesson
title: Dimensions
author: Kevin McAleer
type: page
cover: /learn/bettercad/assets/banners/05_dimensions.jpg
date: 2026-11-20
previous: 04_sketching_with_constraints.html
next: 06_push_pull_and_shapes.html
description: Place a hole exactly where you want it with dimensions and sums, then
  resize a BetterCad sketch by editing one number
percent: 42
duration: 5
quiz_total: 39
date_updated: 2026-11-24
navigation:
- name: Learn BetterCad
- content:
  - section: Overview
    content:
    - name: Learn BetterCad
      link: 00_intro.html
  - section: Your First Part
    content:
    - name: Look Around and Sketch
      link: 01_look_around_and_sketch.html
    - name: Extrude and Fillet
      link: 02_extrude_and_fillet.html
    - name: Export Your First Print
      link: 03_export_your_first_print.html
  - section: Sketching Properly
    content:
    - name: Sketching With Constraints
      link: 04_sketching_with_constraints.html
    - name: Dimensions
      link: 05_dimensions.html
  - section: Building With Shapes
    content:
    - name: Push, Pull and Shapes
      link: 06_push_pull_and_shapes.html
    - name: Combining Parts and Holes
      link: 07_combining_parts_and_holes.html
    - name: Fillets, Chamfers and Shells
      link: 08_fillets_chamfers_and_shells.html
  - section: Models That Change
    content:
    - name: The Timeline and Parameters
      link: 09_the_timeline_and_parameters.html
    - name: Saving, Sharing and Printing
      link: 10_saving_sharing_and_printing.html
  - section: Project
    content:
    - name: Project - A Phone Stand
      link: 11_project_phone_stand.html
    - name: Summary and Next Steps
      link: 12_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to place a hole exactly with dimensions, and resize a sketch by editing one number.

A **dimension** is a constraint with a number in it. It's how you tell BetterCad "this hole is 25 mm from that edge" and have it stay true. In this lesson you'll put a screw hole in the 50 × 30 mm plate from the last lesson.

---

## Add a circle

With the plate sketch still open:

1. Press <kbd>C</kbd> for Circle.
2. Click somewhere inside the rectangle, then move the pointer away a little.
3. Type `3.4` and press <kbd>Enter</kbd>. Press <kbd>Esc</kbd>.

![A blue circle inside the black rectangle](/learn/bettercad/assets/circle_added.jpg){:class="img-fluid w-100"}

The circle is blue while the rectangle stays black. Its *size* is set (3.4 mm is a comfortable printed hole for an M3 screw) but its *position* isn't. Properties says **2 degrees of freedom**: left-right and up-down.

## Say how far across

1. Press <kbd>D</kbd> for Dimension.
2. Click the corner of the rectangle on the origin.
3. Click the centre of the circle.
4. Move the pointer **below** the rectangle. The dimension previews as a distance *across*.

![Previewing a dimension across](/learn/bettercad/assets/dim_across_place.jpg){:class="img-fluid w-100"}

5. Click to place it, then type `50 / 2` and press <kbd>Enter</kbd>.

![Typing a sum into the size box](/learn/bettercad/assets/dim_expression.jpg){:class="img-fluid w-100"}

Yes, you can type a sum. Any size box in BetterCad takes `+`, `-`, `*`, `/` and brackets, so you never need a calculator to find the middle of something.

Where you move the pointer before you click decides which distance you get:

| Pointer goes | You get |
|---|---|
| Above or below the two points | The distance across |
| Left or right of them | The distance up |
| Off at an angle, beside the line between them | The straight-line distance |
{:class="table table-single"}

## Say how far up

The Dimension tool is still on.

1. Click the origin corner again, then the centre of the circle.
2. Move the pointer to the **left** of the rectangle and click.
3. Type `30 / 2` and press <kbd>Enter</kbd>. Press <kbd>Esc</kbd>.

![The plate with every dimension in place](/learn/bettercad/assets/all_dims.jpg){:class="img-fluid w-100"}

Everything is black, and Properties says **Fully constrained**. The hole is 25 mm across and 15 mm up, the dead centre of the plate.

## Change your mind

Here's the payoff. Double-click the **50 mm** dimension, type `80` and press <kbd>Enter</kbd>.

![Editing the width dimension](/learn/bettercad/assets/edit_dim.jpg){:class="img-fluid w-100"}

![The plate at 80 mm wide](/learn/bettercad/assets/plate_resized.jpg){:class="img-fluid w-100"}

The plate stretches to 80 mm. Every rule still holds: the corners are square, the hole is still 3.4 mm, and it's still 25 mm from the left edge and 15 mm up.

Look closely, though. The hole is no longer in the middle. You typed `50 / 2`, BetterCad worked that out as 25, and 25 is what it remembers. It did exactly what you asked.

To make a hole that *stays* in the middle, the dimension has to refer to the width by name, not repeat its value. That's what **parameters** are for, and you'll meet them in lesson 9. Until then, a **Midpoint** or **Symmetric** constraint does the same job without any numbers.

Press <kbd>Enter</kbd> to finish the sketch. If you'd like to keep the plate, extrude it 3 mm: BetterCad sees the circle inside the rectangle and leaves it as a hole.

## Dimensioning well

- **Dimension from the thing that matters.** If a hole must line up with an edge, measure from that edge, not from the far side
- **Fewer is better.** Use Equal, Midpoint and Symmetric where you can, and save numbers for sizes you actually care about
- **Don't say it twice.** A sketch that is already black doesn't need another dimension

---

## Try it Yourself

1. Change the 25 mm dimension to `80 / 2`. Is the hole central again?
2. Add a second 3.4 mm hole 10 mm in from the right-hand edge. Which two things do you click for the dimension?
3. Challenge: delete the 15 mm dimension and use a **Midpoint** constraint between the circle's centre and the left-hand line. Now change the height to 40.

## Common Issues

- **Problem**: I clicked a line but BetterCad picked the axis.
- **Solution**: Click the line away from where it crosses an axis.
- **Why**: The axes count as lines you can dimension to, and where two overlap the axis wins.

- **Problem**: I got a diagonal distance when I wanted the distance across.
- **Solution**: Press <kbd>Ctrl</kbd>+<kbd>Z</kbd>, pick the two points again, and move the pointer straight below them before you click.
- **Why**: The pointer position chooses between across, up and straight-line.

- **Problem**: The size box won't take my sum.
- **Solution**: Check for a missing bracket or a stray letter.
- **Why**: Sums can use numbers, `+ - * /`, brackets and parameter names, and nothing else.

## Check your understanding

{% quiz %}
? You type `50 / 2` into a dimension. What does BetterCad store?
- [ ] A link to the 50 mm side
- [x] 25 mm
- [ ] Nothing, sums aren't allowed
> The sum is worked out once. To follow another size, use a parameter or a constraint.

? How do you choose between a distance across and a distance up?
- [ ] Hold Shift
- [ ] Pick it from a menu
- [x] By where you move the pointer before you click to place it
> Below or above gives across. Left or right gives up.

? How do you change a dimension you've already placed?
- [x] Double-click it and type a new size
- [ ] Delete the sketch and start again
- [ ] Drag the line
> A fully constrained line can't be dragged. Edit the number that controls it.
{% endquiz %}
