---
layout: lesson
title: Set Up Your Printer
author: Kevin McAleer
type: page
cover: /learn/bettercad_printing/assets/banners/01_set_up_your_printer.jpg
date: 2026-11-20
previous: 00_intro.html
next: 02_overhangs_and_supports.html
description: Pick your printer in BetterCad, see its bed around your model, check
  the part fits and save the settings for next time
percent: 16
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

By the end of this lesson you will be able to pick your printer in BetterCad, check that a model fits its bed, and save your settings as your own printer.

Every check in the Print panel depends on knowing what you're printing on. So the first job is to tell BetterCad about your printer.

---

## Pick your printer

Open the **Bracket** sample, then open **Print settings** from the account menu. Under **Printer**, choose yours from the list.

![The bracket inside a Bambu A1 Mini build volume](/learn/bettercad_printing/assets/printer_picked.jpg){:class="img-fluid w-100"}

| Printer | Build volume (mm) |
|---|---|
| Bambu A1 Mini | 180 × 180 × 180 |
| Bambu A1, P1 or X1 | 256 × 256 × 256 |
| Prusa Core One | 250 × 220 × 270 |
| Prusa Core One L | 300 × 300 × 330 |
| Prusa MK4 | 250 × 210 × 220 |
| Prusa Mini | 180 × 180 × 180 |
| Ender 3 | 220 × 220 × 250 |
| Elegoo Neptune 4 | 225 × 225 × 265 |
{:class="table table-single"}

With **Show the build volume** ticked, the bed is drawn under your model with the printable space as a faint box, and its name written along the edge. The panel confirms: **The model fits a Bambu A1 Mini.**

Not on the list? Choose **Custom** and type your bed's X, Y and Z.

![A custom printer with its own bed size](/learn/bettercad_printing/assets/printer_custom.jpg){:class="img-fluid w-100"}

## Where is zero?

Slicers disagree about where the middle of the world is, and BetterCad tells you which kind yours is.

- Bambu and Prusa slicers put 0, 0 in the **middle** of the bed
- Ender and Elegoo slicers put it at the **front-left corner**, which BetterCad marks with a small cross

![An Ender 3 bed, with zero at the front-left corner](/learn/bettercad_printing/assets/printer_ender.jpg){:class="img-fluid w-100"}

Click **Centre on bed** and the model moves to the middle of the plate with its lowest point resting on it. That's one step on the timeline, so <kbd>Ctrl</kbd>+<kbd>Z</kbd> undoes it.

## When it doesn't fit

Here's a 300 mm bar on a printer with a 180 mm bed.

![A bar that is too long for the bed, tinted red](/learn/bettercad_printing/assets/too_big.jpg){:class="img-fluid w-100"}

The part is tinted, and the warning is specific: **The model is 120 mm too wide for a Bambu A1 Mini. Rotate it or pick a bigger printer.** The same warning appears at the top of the Export dialog, so you can't miss it on the way out.

![The same warning in the Export dialog](/learn/bettercad_printing/assets/too_big_export.jpg){:class="img-fluid w-100"}

You have three ways out: turn the part so its long side runs diagonally or upright, split it into pieces that join, or make it smaller.

## Save it as your printer

The bed, and the hole and fit settings you'll tune later in this course, are stored *in the model*. That's useful when you share it, because the other person sees the same bed. But you don't want to set them up again for every new design.

1. Click **Save as a printer**.
2. Type a name, such as "My A1 Mini".

![Naming a saved printer](/learn/bettercad_printing/assets/save_printer.jpg){:class="img-fluid w-100"}

3. Click **Save**.

From now on, pick it from **My printers** at the top of the panel in any model and all its values are copied in. Saved printers are kept in your browser, and in your account too if you sign in, so they follow you to another computer.

If you change a setting later, the panel offers **Save to** that printer.

---

> **Teachers**: saved printers are kept in the browser, and in an account too if the learner signs in. On shared or wiped computers a saved printer will not follow the learner, so write your printer's numbers on the wall or share them as a handout and let learners re-enter them. Agree one name for each school printer so the class picks the right one.

## Try it Yourself

1. Pick your own printer, or enter it as Custom, and save it with a name.
2. Add a Box from the Library and make it 10 mm longer than your bed. Read the warning, then set its Rotation Z to `45`. Does it fit now?
3. Untick **Show the build volume**. What changes in the view?

## Common Issues

- **Problem**: The bed isn't drawn.
- **Solution**: Tick **Show the build volume**.
- **Why**: The setting is remembered by your browser, and it may have been switched off earlier.

- **Problem**: My model is nowhere near the bed.
- **Solution**: Click **Centre on bed**.
- **Why**: Models are built around the origin, which for a corner-origin printer is the front-left of the plate.

- **Problem**: I set up my printer but a new model doesn't have it.
- **Solution**: Choose it from **My printers**.
- **Why**: Settings live in each model. A saved printer copies them in when you pick it.

## Check your understanding

{% quiz %}
? Where does an Ender 3's slicer put 0, 0?
- [ ] In the middle of the bed
- [x] At the front-left corner
- [ ] At the back-right corner
> Ender and Elegoo slicers use the front-left corner. Bambu and Prusa use the middle.

? What does Save as a printer keep?
- [ ] Only the printer's name
- [ ] The model's shape
- [x] The bed size, hole offset, fit gaps and material
> Pick it in another model and those values are copied in.

? A part is 20 mm too long for the bed. What's a quick thing to try?
- [x] Turn it to lie diagonally
- [ ] Export it anyway
- [ ] Change the material
> A bed's diagonal is longer than either side.
{% endquiz %}
