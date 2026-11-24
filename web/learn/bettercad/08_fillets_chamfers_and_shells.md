---
layout: lesson
title: Fillets, Chamfers and Shells
author: Kevin McAleer
type: page
cover: /learn/bettercad/assets/banners/08_fillets_chamfers_and_shells.jpg
date: 2026-11-20
previous: 07_combining_parts_and_holes.html
next: 09_the_timeline_and_parameters.html
description: Round and bevel edges, then hollow a solid into a thin-walled enclosure
  with the BetterCad Shell tool
percent: 63
duration: 6
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

By the end of this lesson you will be able to round and bevel edges, choose between a fillet and a chamfer, and hollow a solid into an enclosure with Shell.

Your enclosure is still a solid brick with holes in it. Two tools finish it: **Fillet** softens the corners, and **Shell** scoops out the inside. The order you use them in matters, and this lesson shows why.

---

## Round the upright edges

You met Fillet in lesson 2. This time you'll round the four upright corners of the box.

1. Press <kbd>G</kbd>.
2. Click the three upright edges you can see.
3. Drag to turn the view, or tap the <kbd>←</kbd> arrow key a few times, until the fourth edge comes round. Click it. Turning the view doesn't put the tool down.
4. Type `3`.

![Four upright edges picked for a 3 mm fillet](/learn/bettercad/assets/fillet_four_edges.jpg){:class="img-fluid w-100"}

The panel lists the four edges and says **4 edges on Box 1**.

5. Press <kbd>Enter</kbd>.

![The box with rounded corners](/learn/bettercad/assets/rounded_box.jpg){:class="img-fluid w-100"}

## Fillet or chamfer?

The Fillet panel has a **Shape** switch with two settings.

| Shape | What you get | Good for |
|---|---|---|
| Fillet | A rounded edge | Upright corners, top edges, anywhere you'll touch |
| Chamfer | A flat 45° bevel | Bottom edges, and edges that face down when printing |
{:class="table table-single"}

The reason is the way printers work. Each layer needs something underneath it. A fillet on a bottom edge starts almost flat, so its first layers hang in the air and droop. A chamfer rises at 45°, which every printer can manage.

A simple rule: **fillet what faces up or sideways, chamfer what faces down.**

## Hollow it out

> Press Shell (S), click the face to leave open, type a wall thickness and press Enter.

1. Press <kbd>S</kbd>.
2. Click the top face. That's the face that will be left open.

![The Shell tool with the top face picked](/learn/bettercad/assets/shell_face.jpg){:class="img-fluid w-100"}

3. Type `2` for a 2 mm wall and press <kbd>Enter</kbd>.

![The finished enclosure, with screw posts](/learn/bettercad/assets/shelled.jpg){:class="img-fluid w-100"}

BetterCad reports **Hollowed Box 1 to a 2 mm wall**, and look at what you got for free. Shell keeps a 2 mm wall around *everything*, including the four screw holes, so each one is now a **screw post** running the full height of the box. The cable hole goes cleanly through the wall, and the rounded corners are rounded inside as well as out.

That's why the holes came first. Had you shelled the box and then added holes, you'd have four holes in a thin floor and nothing to screw a lid into.

### Shell settings

| Setting | What it does |
|---|---|
| Open faces | The faces to remove. Click more than one for a tube or a tray with an open end |
| Wall | The wall thickness |
| Direction: Inside | Keeps the outside size and hollows inwards |
| Direction: Outside | Keeps the inside size and adds the wall around it |
{:class="table table-single"}

Use **Outside** when the *inside* is what must fit, such as a box around a circuit board you've measured.

How thick should a wall be? Walls are printed as whole lines of plastic, so multiples of your nozzle width work best. With a standard 0.4 mm nozzle, 1.2 mm is light, 1.6 mm is a good everyday wall and 2 mm is sturdy.

## When a fillet won't fit

Try to round the thin top rim. Press <kbd>G</kbd>, click one of the long top edges and type `3`.

![BetterCad explaining that a 3 mm fillet does not fit](/learn/bettercad/assets/fillet_too_big.jpg){:class="img-fluid w-100"}

**A 3 mm fillet does not fit here. Try 1.9 mm or less.** The wall is only 2 mm thick, so there isn't room for a 3 mm curve. BetterCad tells you the largest size that will work. Type `1` and it previews happily. Press <kbd>Esc</kbd> to back out without keeping it.

## See how the samples do it

The **Box with a lid** sample on the start card is a finished example of the same ideas, with a lid that fits. You can also [open it directly](https://bettercad.kevsrobots.com/#/sample/box-with-lid).

![The Box with a lid sample](/learn/bettercad/assets/box_with_lid.jpg){:class="img-fluid w-100"}

You'll want your enclosure for the next lesson, so save the sample for afterwards. When you do open it, step through its timeline with <kbd>Alt</kbd>+<kbd>←</kbd> to watch it being built.

---

## Try it Yourself

1. Use **Section** (<kbd>X</kbd>) to slice through the enclosure and look at the wall and a screw post from the side.
2. On a spare box, switch **Shape** to Chamfer and bevel the four bottom edges by 1 mm.
3. Shell a spare box with *two* opposite faces open. What have you made?

## Common Issues

- **Problem**: I can't reach the edge at the back.
- **Solution**: Turn the view with a drag or the arrow keys while the Fillet tool is on, then click it.
- **Why**: You can only pick what's in view. The tool waits while you look around.

- **Problem**: Shell failed, or the inside looks broken.
- **Solution**: Use a thinner wall, or make the fillets larger than the wall.
- **Why**: The inside is the outside shrunk by the wall thickness. If a feature is smaller than the wall, there's nothing left of it to shrink.

- **Problem**: I shelled first and my holes have no posts.
- **Solution**: Drag the Shell step to after the Hole step on the timeline, or undo and do them in the other order.
- **Why**: Shell only wraps a wall around what exists when it runs.

## Check your understanding

{% quiz %}
? Which edge treatment suits the bottom edge of a print?
- [ ] A fillet
- [x] A chamfer
- [ ] Neither, leave it sharp
> A 45° chamfer prints without support. A fillet there starts nearly flat and droops.

? You shell a box that already has four through holes. What happens to the holes?
- [ ] They disappear
- [ ] They're filled in
- [x] Each one gets a wall around it and becomes a post
> Shell keeps the wall thickness around every surface, holes included.

? You need a box whose inside is exactly 50 mm wide. Which Shell direction do you use?
- [ ] Inside
- [x] Outside
- [ ] It makes no difference
> Outside keeps the size you modelled as the inside and adds the wall around it.
{% endquiz %}
