---
title: Create the Top Panel
description: Design the slotted top panel for the mini-rack and export a DXF file for laser cutting
type: page
layout: lesson
cover: /learn/mini_rack/assets/banners/08_top_panel.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to design a slotted panel, repeat a slot with the rectangular pattern tool, and export a DXF file for a laser cutter.

Ahoy there makers! The top panel is the lid of the rack. It carries the handles, and its long slots let warm air escape from the Pis below. Unlike the corners, this part is not printed. It is cut from flat sheet, so we finish by making a DXF file, the 2D drawing format laser cutters read.

In the original project the panels are laser cut from basswood. The top panel is the green slotted sheet in the components photo from lesson 1.

1. Create a new file named `top_panel` in Fusion 360

1. Create a new sketch on the XY plane, named `panel_base`

    ![dimensions](/learn/mini_rack/assets/top_panel_01_sketch.png){:class="img-fluid w-100"}

1. Extrude the body profile by `2mm` (or whatever the depth of the wood you're looking to use is):

    ![Extrude](/learn/mini_rack/assets/top_panel_02_extrude.png){:class="img-fluid w-100"}

    ![Extrude](/learn/mini_rack/assets/top_panel_03_extrude.png){:class="img-fluid w-100"}

1. Create a new sketch on the top of the new part, named `holes`

    ![Sketch](/learn/mini_rack/assets/top_panel_04_sketch.png){:class="img-fluid w-100"}

1. Extrude-Cut the `holes` sketch:

    ![Extrude](/learn/mini_rack/assets/top_panel_05_extrude.png){:class="img-fluid w-100"}

    ![Extrude](/learn/mini_rack/assets/top_panel_06_extrude.png){:class="img-fluid w-100"}

1. Create a new sketch on the top of the new part, named `slots`

    ![Sketch](/learn/mini_rack/assets/top_panel_07_sketch.png){:class="img-fluid w-100"}

1. Extrude-Cut the `slots` sketch:

    ![Extrude](/learn/mini_rack/assets/top_panel_08_extrude.png){:class="img-fluid w-100"}

    ![Extrude](/learn/mini_rack/assets/top_panel_09_extrude.png){:class="img-fluid w-100"}

1. Use the rectangular pattern tool to create the remaining slots:

    ![Pattern](/learn/mini_rack/assets/top_panel_10_pattern.png){:class="img-fluid w-100"}

    ![Pattern](/learn/mini_rack/assets/top_panel_11_pattern.png){:class="img-fluid w-100"}

1. Finally, to create a DXF file for laser-cutting, create a new sketch on the top of the new part, named `dxf` and then use the projection (press the `p` key) tool to project all the edges of the part:

    ![DXF](/learn/mini_rack/assets/top_panel_12_sketch.png){:class="img-fluid w-100"}

## Why a pattern, not copies

You draw one slot, and then the rectangular pattern repeats it for you. If you later want the slots closer together, you change one number in the pattern instead of redrawing every slot. In the components photo you can see how the slots run in staggered rows. Warm air rises, and rows of slots let it leave the top.

## Getting a DXF out

The last step projects every edge of the part into a new sketch named `dxf`. Press `p` to use the Project tool. A laser cutter does not care about thickness, only about the outline and the cut paths, so a projected sketch is exactly what it needs. Then right-click the sketch in the Browser tree and choose Save as DXF.

The extrude thickness in step 3 is only there so the part looks right on screen. It should match your sheet: `2mm`, or "whatever the depth of the wood you're looking to use is".

## Laser cutter safety

A laser cutter is a fire risk. Never leave it running unattended, keep the extraction fan on, and only cut materials your machine is rated for. A laser also burns away a thin line of material, called the kerf, so holes can come out a little larger than drawn. Cut a test piece from offcuts first.

> **Teachers**: Laser cutters need trained supervision, and most schools allow only staff to operate them. Learners can complete the design and DXF export at the computer, and the teacher or technician cuts the panels. Allow time in the queue: a full class set of panels may take more than one session. If you have no laser cutter, a local makerspace or an online cutting service can cut from your DXF.

## Try it Yourself

1. Change the slot length in the `slots` sketch, then see how the pattern updates.
2. Add one extra row of slots using the rectangular pattern tool.
3. Challenge: after you export the DXF, open it in a free viewer or in your laser cutter's software. Does the outline look the way you expect?

## Common Issues

- **Problem**: The rectangular pattern does nothing.
- **Solution**: Set the pattern type to Features, select the slot cut in the timeline, and pick an axis.
- **Why**: A pattern needs to know which feature to repeat, and which direction.

- **Problem**: The laser cutter says my DXF is empty.
- **Solution**: Check the `dxf` sketch contains projected lines, and export the sketch, not the body.
- **Why**: The cutter reads sketch lines, and an exported body is a 3D solid.

- **Problem**: The panel does not fit the rack.
- **Solution**: Measure your real sheet thickness and the finished cut, and adjust the design.
- **Why**: Real material varies, and the laser kerf changes the final size slightly.

## Check your understanding

{% quiz %}
? What is the last step for a laser cutter?
- [ ] Fillet the edges
- [x] Project the edges into a `dxf` sketch
- [ ] Mirror the body
> The final step creates a `dxf` sketch by projecting all the edges.

? What does the rectangular pattern tool do here?
- [x] Repeats the slot to make the rest of the slots
- [ ] Adds the holes
- [ ] Sets the thickness
> It copies the slot into a grid of slots.

? Why does the extrude thickness matter?
- [ ] It sets the DXF format
- [x] It should match the thickness of the wood you use
- [ ] It sets the number of slots
> The lesson uses `2mm`, or whatever thickness your wood is.
{% endquiz %}
