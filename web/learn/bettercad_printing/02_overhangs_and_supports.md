---
layout: lesson
title: Overhangs and Supports
author: Kevin McAleer
type: page
cover: /learn/bettercad_printing/assets/banners/02_overhangs_and_supports.jpg
date: 2026-11-20
previous: 01_set_up_your_printer.html
next: 03_thin_walls.html
description: Find the parts of a model that would print in mid-air with the BetterCad
  overhang check, and learn how to design them away
percent: 24
duration: 5
quiz_total: 36
date_updated: 2026-11-24
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

By the end of this lesson you will be able to run the overhang check, explain the 45 degree rule, and design overhangs away.

A 3D printer builds from the bed upwards, and every layer needs something to sit on. Where there's nothing underneath, you get an **overhang**. Small ones print fine. Large ones sag, string or fail unless the slicer adds **supports**, which cost time and plastic and leave a rough surface behind.

The overhang check shows you where they are while you can still do something about it.

---

## Run the check

With the **Bracket** sample open, go to **Checks** in the Print panel and click **Overhangs**.

![The overhang check on the bracket](/learn/bettercad_printing/assets/overhangs_bracket.jpg){:class="img-fluid w-100"}

The result is one line: **Needs support: 1 area, 13 mm²**. Each area is listed with its size and a **Show** button that swings the view round to it. Problem faces are tinted on the model, and the tint is stronger the closer a face is to flat.

Here the culprit is the top of the screw hole in the upright. It's a tiny roof. At 13 mm² it will print without help.

## The 45 degree rule

Each layer can stick out a little beyond the one below. Stack those small steps and you get a slope. Most printers manage a slope up to about 45° from vertical with no support at all. Past that, quality drops quickly.

The **Steepest** box sets the limit the check uses. It starts at 45° and goes from 30° to 70°.

![The same check with a stricter 30 degree limit](/learn/bettercad_printing/assets/overhangs_30.jpg){:class="img-fluid w-100"}

At 30° the bracket's area grows from 13 to 19 mm², because more of the hole's curve now counts.

| Set it to | When |
|---|---|
| 30° to 40° | Your printer struggles with overhangs, or you're printing fast or in PETG |
| 45° | The sensible default |
| 55° to 70° | A well-tuned printer with good cooling, in PLA |
{:class="table table-single"}

## A real overhang

A tee shape makes the problem obvious: a 10 mm post with a 40 mm bar across the top.

![A tee shape with a large overhang under the crossbar](/learn/bettercad_printing/assets/tee_overhang.jpg){:class="img-fluid w-100"}

**Needs support: 1 area, 400 mm².** That's the whole underside of the bar, hanging in space on one side of the post. Printed like this it needs a tower of support.

## Three ways to fix an overhang

**Turn the part.** Lay the tee on its side and nothing hangs at all.

![The tee lying flat, with no overhangs](/learn/bettercad_printing/assets/tee_flat.jpg){:class="img-fluid w-100"}

**Nothing overhangs more than 45°, so it needs no support.** This is the cheapest fix there is, and it gets a whole lesson of its own next.

**Add a slope.** Replace a flat underside with a 45° chamfer or a wedge, so each layer steps out gradually. This is why BetterCad's phone stand sample uses a wedge for its back rest.

**Change the shape of holes.** A round hole on its side has a flat roof at the very top. On small holes it doesn't matter. On large ones, a teardrop or a hole with a pointed top prints cleanly.

## What the check doesn't know

The check flags every downward-facing surface past the limit. It doesn't yet recognise a **bridge**: a flat span held at both ends, like the top of a window. Printers can cross a gap of a few centimetres in mid-air, so a short bridge that gets flagged can usually be ignored. The panel reminds you: "Short bridges often print fine without it."

Faces within 0.2 mm of the bed are never flagged. They're the first layer.

---

## Try it Yourself

1. Open the **Phone stand** sample and run the check. Then change its `lean` parameter to `20`. Does it still pass?
2. Build the tee: a Box 10 × 10 × 20, and a Box 40 × 10 × 5 with a Z position of 20, grouped. Check that you get 400 mm².
3. Fix the tee without turning it. Add a Wedge under the bar, group it and check again.

## Common Issues

- **Problem**: I can't see the tinted area.
- **Solution**: Click **Show** beside it in the list, or turn the view to look from underneath.
- **Why**: Overhangs face downwards, away from the usual viewpoint.

- **Problem**: The check flags the top of every horizontal hole.
- **Solution**: Ignore them on holes smaller than about 8 mm.
- **Why**: The roof of a small hole is a very short bridge.

- **Problem**: The check says no support is needed but my print still drooped.
- **Solution**: Lower **Steepest** to 40° or 35° and check again.
- **Why**: 45° is typical, but your printer, speed and material set the real limit.

## Check your understanding

{% quiz %}
? What is an overhang?
- [ ] A wall that's too thin
- [x] Part of a model with nothing printed underneath it
- [ ] A part that's too big for the bed
> Each layer needs support from the one below.

? What's usually the cheapest way to remove an overhang?
- [x] Turn the part to print another way up
- [ ] Add more infill
- [ ] Print it hotter
> A different orientation can remove the overhang with no change to the design.

? The check flags a 15 mm flat span held at both ends. What is it?
- [ ] A certain failure
- [x] A bridge, which will probably print fine
- [ ] A thin wall
> The check doesn't recognise bridges yet, so short ones are flagged too.
{% endquiz %}
