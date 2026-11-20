---
layout: lesson
title: Exporting for Your Slicer
author: Kevin McAleer
type: page
cover: /learn/bettercad_printing/assets/banners/09_exporting_for_your_slicer.jpg
date: 2026-11-20
previous: 08_material_and_weight.html
next: 10_project_box_with_a_lid.html
description: Choose between STL, 3MF and STEP, pick a resolution and export one file
  or many from BetterCad
percent: 80
duration: 6
quiz_total: 36
date_updated: 2026-11-20
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

The checks pass and the settings are saved. The last step in BetterCad is to write a file your slicer can open. There are a few choices to make, and each has a right answer most of the time.

---

## The Export dialog

Press <kbd>Ctrl</kbd>+<kbd>E</kbd>, or open the menu beside **Save** and choose **Export…**.

![The Export dialog](/learn/bettercad_printing/assets/export_stl.jpg){:class="img-fluid w-100"}

The first line reminds you which printer the model is set up for, and warns you there if it doesn't fit.

## Choose a format

| Format | Send it to | What's in it |
|---|---|---|
| STL | Any slicer ever made | Triangles, and nothing else. No units, no names |
| 3MF | Modern slicers: PrusaSlicer, Bambu Studio, OrcaSlicer, Cura | Triangles in millimetres, with the model's title and each part named |
| STEP | Other CAD tools: Fusion, FreeCAD, Onshape. Also CNC | The exact shape, with true curves, each part named |
| GLB, glTF | Web pages, game engines and 3D viewers | The model as it looks, for showing off |
| URDF | Robot simulators | A robot description. That's a course of its own |
{:class="table table-single"}

**For printing, use 3MF.** It knows it's in millimetres, so it never opens a thousand times too small, and it keeps a multi-part model as separate named parts in one file.

Use **STL** when something only accepts STL.

Use **STEP** when someone wants to *edit* your design in another CAD program. It's exact geometry, so a round hole is still a round hole when they open it. It has no resolution setting.

![Exporting a STEP file](/learn/bettercad_printing/assets/export_step.jpg){:class="img-fluid w-100"}

And keep the `.bcad`. None of these formats holds your timeline or parameters.

## Choose what to export

| What | You get |
|---|---|
| All bodies | Everything in one file |
| Selected | Only the parts you had selected when you opened the dialog |
| Each on its own | One file per part |
{:class="table table-single"}

![Exporting each body on its own as 3MF](/learn/bettercad_printing/assets/export_each.jpg){:class="img-fluid w-100"}

**Each on its own** suits parts that print in different colours, materials or orientations. In Chrome and Edge you pick a folder and the files appear in it. In other browsers each file downloads separately.

Hole shapes that haven't been grouped with a solid are left out, with a warning and a **Group them now** button. A hole on its own isn't a thing you can print.

## Choose a resolution

STL and 3MF are made of flat triangles, so curves are approximated. **Resolution** says how far the triangles may stray from the true surface.

| Resolution | Within | Use it for |
|---|---|---|
| Draft | 0.1 mm | Quick test prints and big simple shapes |
| Normal | 0.02 mm | Almost everything |
| Fine | 0.005 mm | Lenses, threads, and curves you'll look at closely |
| Custom | You choose | Special cases |
{:class="table table-single"}

![A custom resolution](/learn/bettercad_printing/assets/export_custom.jpg){:class="img-fluid w-100"}

It's an absolute distance, so a small part and a large part get the same accuracy. The dialog shows an estimate of the triangle count and file size before you commit.

**Normal** is already finer than a printer can reproduce. A 0.4 mm nozzle can't show a 0.02 mm error. Fine makes bigger files and slower slicing for no visible gain on most prints, and on a detailed model it can take a while to produce.

If you can see flat facets on a printed curve, the cause is more often the slicer's own resolution setting or a very coarse Draft export.

## Place on the bed

Leave **Place on the bed** ticked. It moves everything so the lowest point is at Z = 0, and the part arrives in the slicer sitting on the plate.

Untick it only when the position matters, such as when several files have to line up with each other in another program.

## In the slicer

A short checklist on the other side:

1. The part should arrive the right size. If an STL looks tiny, the slicer has guessed inches. Tell it millimetres
2. Check the orientation is the one you chose
3. Set the infill you used for the estimate
4. Look at the layer preview. This is where a missing thin wall or an unexpected support shows up
5. Print

---

## Try it Yourself

1. Export the Bracket sample as STL at Draft, Normal and Fine. Compare the triangle counts in the dialog.
2. Open the **Box with a lid** sample and export **Each on its own** as 3MF. What are the files called?
3. Export a STEP file and open it in another CAD program if you have one. Are the holes still true circles?

## Common Issues

**Problem**: The part opens tiny in my slicer.
**Solution**: Export 3MF, or set the slicer's import units to millimetres.
**Why**: STL files don't record units, so slicers guess.

**Problem**: A part is missing from the exported file.
**Solution**: Check it isn't hidden, and that it isn't a hole shape waiting to be grouped.
**Why**: Hidden bodies and loose holes aren't exported.

**Problem**: Export is slow at Fine.
**Solution**: Use Normal.
**Why**: Fine makes many more triangles, and on a filleted part that takes time.

## Check your understanding

{% quiz %}
? Which format is the best default for 3D printing?
- [ ] STL
- [x] 3MF
- [ ] STEP
> 3MF carries millimetres and named parts, and modern slicers all read it.

? A friend wants to modify your part in Fusion. What do you send?
- [ ] STL
- [ ] 3MF
- [x] STEP
> STEP holds exact geometry that other CAD tools can work on.

? What does the Resolution setting control?
- [x] How closely the triangles follow the true surface
- [ ] The layer height of the print
- [ ] The size of the model
> It's the largest allowed distance between the mesh and the real shape.
{% endquiz %}
