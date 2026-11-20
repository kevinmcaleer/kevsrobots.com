---
layout: lesson
title: Export Your First Print
author: Kevin McAleer
type: page
cover: /learn/bettercad/assets/banners/03_export_your_first_print.jpg
date: 2026-11-20
previous: 02_extrude_and_fillet.html
next: 04_sketching_with_constraints.html
description: Export a part from BetterCad as STL, 3MF or STEP, and learn which format
  to use and what resolution means
percent: 28
duration: 5
quiz_total: 39
date_updated: 2026-11-20
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

A model in BetterCad isn't a print yet. Your slicer, the program that prepares files for your printer, needs it as a file it understands.

---

## Export an STL

> Press Ctrl+E, or Export from the arrow beside Save, and save the STL.

1. With your block from the last lesson open, press <kbd>Ctrl</kbd>+<kbd>E</kbd>.

![The Export dialog](/learn/bettercad/assets/export_stl.jpg){:class="img-fluid w-100"}

2. Leave the format on **STL** and the resolution on **Normal**.
3. Click **Export** and choose where to save it.

That's a printable part. Open the file in your slicer and it will be sitting on the bed, the right size, ready to slice.

## Which format?

The Export dialog offers several formats. Three matter for printing and sharing:

| Format | Use it for | What's in it |
|---|---|---|
| STL | Any slicer, old or new | Triangles, and nothing else |
| 3MF | Modern slicers such as PrusaSlicer, Bambu Studio and Cura | Triangles, real units, the model's name, and each body as its own named object |
| STEP | Other CAD programs such as Fusion, FreeCAD and Onshape, or a CNC shop | The exact shapes, with true curves |
{:class="table table-single"}

A quick rule: **3MF for your own printer, STL if something won't open 3MF, STEP if someone needs to edit it.**

![Exporting as 3MF](/learn/bettercad/assets/export_3mf.jpg){:class="img-fluid w-100"}

The other formats, GLB, glTF and URDF, are for showing a model on a web page and for describing robots. They come up in the BetterCad for Robot Builders courses.

## Resolution

STL and 3MF files are made of flat triangles, so a curve is really lots of short straight bits. **Resolution** sets how far those triangles may stray from the true surface:

| Setting | Triangles stay within | Good for |
|---|---|---|
| Draft | 0.1 mm | Quick test prints |
| Normal | 0.02 mm | Nearly everything |
| Fine | 0.005 mm | Large smooth curves you'll see up close |
{:class="table table-single"}

Normal is already finer than most printers can show. The dialog tells you how many triangles and how big the file will be before you export, so you can see what a setting costs.

STEP has no resolution. It stores the exact curve, which is why it's the one to send to another CAD program.

![Exporting as STEP, which is exact](/learn/bettercad/assets/export_step.jpg){:class="img-fluid w-100"}

## The other options

- **Place on the bed** moves the model so its lowest point sits at Z = 0. Leave it on for printing
- **All bodies / Selected / Each on its own** chooses what goes in the file. A box and its lid can go in one file, or one file each
- The line at the top, *For a Prusa Core One* or whichever printer you've chosen, is a reminder of the bed BetterCad checks your model against. You'll set your own printer in lesson 10

## Export is not Save

Exporting makes a file for *printing*. It doesn't keep your sketches, steps or parameters. To keep the model so you can change it later, **Save** it as a `.bcad` file. That's covered in lesson 10. BetterCad also autosaves in your browser as you work, so you won't lose anything in the meantime.

---

## Try it Yourself

1. Export your block as **3MF** and open it in your slicer.
2. Export it again as STL at **Draft** and at **Fine**. Compare the triangle counts the dialog shows.
3. If you have a printer, print the block. Measure it with a ruler or calipers. Is it 40 × 20 × 10 mm?

## Common Issues

**Problem**: Ctrl+E opens something in my browser instead.
**Solution**: Click the 3D view first, or use **Export** in the menu beside Save.
**Why**: Some browsers keep Ctrl+E for the address bar when the page isn't focused.

**Problem**: The file downloads without asking where to put it.
**Solution**: Look in your Downloads folder.
**Why**: Chrome and Edge let you pick a place. Safari and Firefox download it.

**Problem**: My slicer shows the part tiny or enormous.
**Solution**: Use 3MF, or set your slicer's STL units to millimetres.
**Why**: An STL has no units in it. BetterCad always writes millimetres.

## Check your understanding

{% quiz %}
? Which format would you send to a friend who wants to edit your part in Fusion?
- [ ] STL
- [ ] 3MF
- [x] STEP
> STEP keeps the exact shapes, so another CAD program can work on them.

? What does a finer resolution change?
- [ ] The size of the part
- [x] How closely the triangles follow curved surfaces
- [ ] The infill
> Resolution only affects how curves are turned into triangles.

? Does exporting an STL save your sketches and steps?
- [ ] Yes
- [x] No, save a .bcad file for that
- [ ] Only at Fine resolution
> An export is just the finished shape. The .bcad file is the model you can keep editing.
{% endquiz %}
