---
layout: lesson
title: Reference Bodies
author: Kevin McAleer
type: page
cover: /learn/bettercad_robot_parts/assets/banners/01_reference_bodies.jpg
date: 2026-11-21
previous: 00_intro.html
next: 02_the_robot_parts_catalogue.html
description: Bring a real part into BetterCad as a reference body that you can design
  around but that never prints or exports
percent: 14
duration: 5
quiz_total: 42
date_updated: 2026-11-24
navigation:
- name: Design Robot Parts with BetterCad
- content:
  - section: Overview
    content:
    - name: Design Robot Parts with BetterCad
      link: 00_intro.html
  - section: Designing Around Real Parts
    content:
    - name: Reference Bodies
      link: 01_reference_bodies.html
    - name: The Robot Parts Catalogue
      link: 02_the_robot_parts_catalogue.html
    - name: Measure Before You Trust
      link: 03_measure_before_you_trust.html
    - name: Cutting a Pocket
      link: 04_cutting_a_pocket.html
  - section: Mounts
    content:
    - name: Standoffs for a Board
      link: 05_standoffs_for_a_board.html
    - name: Horn Mounts
      link: 06_horn_mounts.html
  - section: Gears and Drives
    content:
    - name: Your First Gear
      link: 07_your_first_gear.html
    - name: Bores, Hubs and Small Gears
      link: 08_bores_hubs_and_small_gears.html
    - name: Meshing Gears
      link: 09_meshing_gears.html
    - name: Racks, Pulleys and Belts
      link: 10_racks_pulleys_and_belts.html
  - section: Parts You Already Have
    content:
    - name: From Mesh to Editable Part
      link: 11_from_mesh_to_editable_part.html
  - section: Project
    content:
    - name: Project - A Geared Servo Deck
      link: 12_project_geared_servo_deck.html
    - name: Summary and Next Steps
      link: 13_summary.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to bring a real part into a model as a reference body, switch a body between reference and printed, and explain what happens to reference bodies on export.

You're making a bracket for a motor. The quickest way to get it right is to have the motor sitting in the model while you draw, so you can see where it touches and measure straight off it.

But the motor isn't yours to print. If it's in the model as an ordinary body it turns up in your STL, adds to the print weight and trips the overhang check. BetterCad's answer is the **reference body**.

---

## What a reference body is

A reference body is there to design around. It shows in the view, see-through and tinted, and it behaves like any other body when you measure it, snap to it or sketch on it.

It is left out of everything to do with printing:

| A reference body is | A reference body is not |
|---|---|
| Shown in the view, see-through | Exported to STL, 3MF or STEP |
| Measurable, and something sketches can snap to | Counted in the print weight |
| Listed in the Browser like any body | Checked for overhangs, thin walls or bed fit |
{:class="table table-single"}

## Import a file as a reference

Most makers have a folder of STLs for parts they own. Any of them can come in as a reference.

1. Download [bracket.stl](/learn/bettercad_robot_parts/assets/bracket.stl). Pretend it's a part you bought.
2. Start a new model with <kbd>Ctrl</kbd>+<kbd>N</kbd>.
3. Click the word **Insert** on the Model tab and choose **Import as a reference…**.
4. Pick the file.

![An STL imported as a reference body](/learn/bettercad_robot_parts/assets/reference_import.jpg){:class="img-fluid w-100"}

The bracket arrives see-through. Select it and look at Properties. The **Reference** row reads **Reference**, the Size section says "It came from a file, so it keeps the size it came with", and the Print section says "A reference body doesn't print, so it has no print weight."

**Import as a reference…** takes STL, STEP and 3MF files.

## Switch a body between reference and printed

Any body can be flipped either way.

1. Select the bracket.
2. Click **Reference** in Properties. It changes to **Printed**, the body turns solid, and the Print section fills in with a volume and weight.
3. Click it again to turn it back.

The right-click menu has the same switch: **Make a reference** on a printed body, **Make printable** on a reference.

This is handy for your own parts too. Designing a lid for a box you printed last month? Open the box, make it a reference, and draw the lid round it without any risk of printing a second box.

## Reference bodies and export

When you export, reference bodies are skipped. There's one rough edge worth knowing about in the current alpha: the Export dialog counts them in a warning that reads "2 hole shapes weren't exported. Group them with a solid first." They aren't hole shapes, they're your references, and they're meant to be left out. Ignore the warning and don't click **Group them now**.

---

## Try it Yourself

1. Import bracket.stl as a reference, then add a **Box** from the Library beside it.
2. Look at **This model** in Properties with nothing selected. Does the weight include the bracket?
3. Make the bracket printable and look again.

## Common Issues

- **Problem**: My imported part is solid, not see-through.
- **Solution**: You used **Import STL, STEP or 3MF…**. Select the body and click **Printed** in Properties to make it a reference.
- **Why**: The plain import brings a file in as a part to print.

- **Problem**: I can't resize the imported part.
- **Solution**: That's expected. A body from a file keeps the size it came with.
- **Why**: An STL is a skin of triangles with no sizes to edit. Lesson 11 shows how to get editable holes out of one.

- **Problem**: Export says "Reference bodies aren't exported. Turn off Reference in Properties to export one."
- **Solution**: Select a printed body as well, or choose **All bodies**.
- **Why**: References never export, so there was nothing left to write.

## Check your understanding

{% quiz %}
? Which of these does a reference body still do?
- [ ] Add to the print weight
- [x] Let you measure it and snap sketches to it
- [ ] Appear in the exported STL
> It's there to design around. Everything about printing ignores it.

? How do you bring an STL in as a reference in one step?
- [ ] Drag it onto the timeline
- [ ] Open it from the account menu
- [x] Insert, then Import as a reference
> It takes STL, STEP and 3MF files.

? You opened a box you printed last month and want to design a lid round it. What's the safe move?
- [x] Make the box a reference
- [ ] Delete the box
- [ ] Hide the box
> As a reference it stays visible to design against but can't end up in the export.
{% endquiz %}
