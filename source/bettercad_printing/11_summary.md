---
title: Summary and Next Steps
description: A recap of Print It Right with BetterCad, a pre-flight checklist to keep by the printer and where to go next
layout: lesson
type: page
cover: /learn/bettercad_printing/assets/banners/11_summary.jpg
date_updated: 2026-11-20
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

You've been through the Print panel from top to bottom, calibrated your printer twice and finished with a box whose lid fits because you decided it would. Most of what separates a frustrating print from a good one happened before the slicer opened.

![The box with a lid](/learn/bettercad_printing/assets/box_with_lid.jpg){:class="img-fluid w-100"}

---

## What you covered

| Lesson | The question it answers |
|---|---|
| Set up your printer | Does it fit the bed? |
| Overhangs and supports | Will any of it print in mid-air? |
| Thin walls | Is anything too thin to exist? |
| Choosing an orientation | Which way up is strongest and cleanest? |
| Holes that fit | Will the screw go in? |
| Fits between parts | Will the parts go together the way I want? |
| Where parts meet | Did I leave a gap everywhere I meant to? |
| Material and weight | How much plastic is this? |
| Exporting for your slicer | Which file, at what resolution? |
{:class="table table-single"}

## Your printer's numbers

Fill these in and keep them. They're saved in your BetterCad printer too.

| Setting | Default | Mine |
|---|---|---|
| Hole offset | 0.4 mm | |
| Press fit | 0.15 mm | |
| Sliding fit | 0.25 mm | |
| Loose fit | 0.4 mm | |
| Lid gap that works | 0.3 mm | |
| Steepest overhang I trust | 45° | |
{:class="table table-single"}

Print the two calibration plates again when you change material, nozzle or printer.

## The pre-flight checklist

Before every export:

1. **Printer picked**, and the panel says the model fits
2. **Overhangs** checked. Anything large is turned, chamfered or accepted
3. **Thin walls** checked
4. **Orientation** chosen for strength first, supports second, looks third
5. **Holes** made with the Hole tool at your offset
6. **Where parts meet** checked with the parts assembled. Nothing too tight
7. **Export 3MF** at Normal, placed on the bed
8. **Save the .bcad**

## The ideas worth keeping

- **Printers are consistent, not accurate.** Yours is wrong by the same small amount every time. Measure that once and design for it
- **Offsets go on diameters, gaps go on each side.** A 0.4 mm hole offset and a 0.2 mm fit gap are the same clearance
- **Orientation is free.** It changes strength, finish and support, and costs nothing
- **Layers are grain.** Lay the part so the load runs along them
- **Checks are guides.** The overhang check doesn't know about bridges, and the thin wall check samples. The slicer preview has the final say
- **Change one thing at a time.** When a fit is wrong, adjust the gap by 0.05 mm and reprint the smallest part

## What's next

- New to the app, or want to revise? **[Learn BetterCad](/learn/bettercad/)** covers sketching, shapes, the timeline and parameters
- The **[Start with BetterCad](/learn/learning_pathways/bettercad.html)** pathway keeps the BetterCad courses in order as more are added. Courses on robot parts, joints and exporting a robot for simulation are on the way
- Ready to put electronics in your box? **[Getting Started with Snakie](/learn/snakie_getting_started/)** gets a Raspberry Pi Pico running MicroPython, and **[Snakie Blocks](/learn/snakie_blocks/)** has you programming it by snapping blocks together
- Want to compare with a traditional CAD program? Try **[Introduction to FreeCAD for Beginners](/learn/freecad/)**

---

## Try it Yourself

1. Fill in the table of your printer's numbers.
2. Take a part you designed before this course and run it through the pre-flight checklist. What does it find?
3. Design a holder for something on your desk, using at least one hole from the Hole tool and one fitted cut.

## Check your understanding

{% quiz %}
? When should you print the calibration plates again?
- [ ] Before every print
- [x] When you change material, nozzle or printer
- [ ] Never
> Those are the things that change how your printer lays down plastic.

? A 0.4 mm hole offset is the same clearance as which gap each side?
- [ ] 0.4 mm
- [ ] 0.8 mm
- [x] 0.2 mm
> The offset is added to the diameter, so it's half that on each side.

? Which has the final say on whether a wall will print?
- [ ] The thin wall check
- [x] The slicer's layer preview
- [ ] The weight estimate
> The checks are guides. The preview shows exactly what the printer will do.
{% endquiz %}
