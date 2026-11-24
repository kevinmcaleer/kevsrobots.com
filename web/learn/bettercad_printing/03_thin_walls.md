---
layout: lesson
title: Thin Walls
author: Kevin McAleer
type: page
cover: /learn/bettercad_printing/assets/banners/03_thin_walls.jpg
date: 2026-11-20
previous: 02_overhangs_and_supports.html
next: 04_choosing_an_orientation.html
description: Use the BetterCad thin wall check to find parts of a model that are too
  thin to print, and learn how thick a wall should be
percent: 32
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

By the end of this lesson you will be able to run the thin wall check and choose a wall thickness that suits your nozzle.

A printer draws with a line of molten plastic about as wide as its nozzle. Ask it for a wall thinner than that and the slicer has two choices: skip it, or make it thicker than you drew. Neither is what you wanted. The thin wall check finds those places.

---

## Run the check

In the Print panel, under **Checks**, click **Thin walls**.

![The thin wall check passing on the box with a lid](/learn/bettercad_printing/assets/thin_ok.jpg){:class="img-fluid w-100"}

On a healthy part you'll see **No walls thinner than 0.8 mm.**

Now a part that fails. This is a panel 30 mm wide, 20 mm tall and only 0.6 mm thick.

![A 0.6 mm panel failing the thin wall check](/learn/bettercad_printing/assets/thin_wall.jpg){:class="img-fluid w-100"}

**1 thin area, thinnest 0.6 mm.** The whole panel is tinted, and it's listed with its thinnest point and a **Show** button.

## Why 0.8 mm?

The **Thinnest** box sets the limit. It starts at 0.8 mm: two passes of a standard 0.4 mm nozzle. One pass will print, but a single-line wall is fragile and often has gaps. Two lines fused side by side make a real wall.

| Nozzle | Sensible limit |
|---|---|
| 0.25 mm | 0.5 mm |
| 0.4 mm | 0.8 mm |
| 0.6 mm | 1.2 mm |
| 0.8 mm | 1.6 mm |
{:class="table table-single"}

If you've fitted a different nozzle, change the limit to match.

## How thick should a wall be?

Passing the check means a wall *can* print. Whether it's strong enough is a design decision. Walls are strongest when they're a whole number of nozzle widths, because the slicer fills them with clean lines and no slivers.

| Wall (0.4 mm nozzle) | Lines | Good for |
|---|---|---|
| 0.8 mm | 2 | Light covers, dividers, anything that only has to exist |
| 1.2 mm | 3 | Small boxes and trays |
| 1.6 mm | 4 | Everyday enclosures and brackets |
| 2.0 to 2.4 mm | 5 to 6 | Things that get dropped, clamped or screwed into |
{:class="table table-single"}

## Where thin walls hide

Thin walls rarely come from typing a small number. They appear by accident, where two features nearly meet:

- **A hole too close to an edge.** A 3.4 mm hole centred 2 mm from an edge leaves 0.3 mm of wall beside it
- **Two holes or pockets too close together**
- **A fillet on a thin rim**, which feathers down to nothing
- **Shelling a part with a wall thicker than a rib or a tab**, which eats the feature from inside
- **Text and fine detail**

That's why the check is worth running even when you're sure of your numbers.

## Fixing them

Go back along the timeline to the step that set the size, and change it. In a parametric model the fix is often one number: double-click the Shell step and change the wall, or open Parameters and raise `wall`.

For a hole near an edge, move the hole in. A good rule is to leave at least one hole diameter between the hole and the edge.

## A guide, not a guarantee

The check works by sampling the surface: from each small patch it looks straight through the part to see how far away the other side is. The panel is honest about that: "This samples the surface, so it can miss a wall." A wall that meets another at a shallow angle can slip through.

Your slicer's preview is the final word. If a wall looks hollow or missing there, it's too thin, whatever any check says.

---

## Try it Yourself

1. Add a Box and set its D to `0.6`. Run the check, then raise D until it passes. What's the smallest value that does?
2. Make a 30 × 30 × 5 Box and use the Hole tool to put an M3 hole 2 mm from one edge. Does the check find the thin wall?
3. Change **Thinnest** to `1.2` and run the check on the Bracket sample.

## Common Issues

- **Problem**: The check takes a moment to answer.
- **Solution**: Wait. You can keep working while it runs.
- **Why**: It measures the whole surface in the background.

- **Problem**: A wall passes the check but my slicer leaves it out.
- **Solution**: Turn on your slicer's "detect thin walls" option, or make the wall a multiple of the nozzle width.
- **Why**: A 1.0 mm wall with a 0.4 mm nozzle is two lines and an awkward 0.2 mm gap.

- **Problem**: Several thin spots are listed as one area.
- **Solution**: Click **Show** and inspect the whole region.
- **Why**: Thin patches that touch, or nearly touch, are grouped together.

## Check your understanding

{% quiz %}
? Why is 0.8 mm the default limit?
- [ ] It's the thinnest layer a printer can make
- [x] It's two widths of a standard 0.4 mm nozzle
- [ ] It's a rule of the STL format
> Two lines side by side make a sound wall. One line is fragile.

? Where do accidental thin walls usually come from?
- [ ] Choosing the wrong material
- [x] Two features that nearly meet, like a hole close to an edge
- [ ] Exporting at draft resolution
> The gap left between features is easy to overlook.

? The check passes but the slicer preview shows a gap in a wall. Which do you trust?
- [ ] The check
- [x] The slicer preview
- [ ] Neither
> The check samples the surface and can miss a wall. The slicer shows what will be printed.
{% endquiz %}
