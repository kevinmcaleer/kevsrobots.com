---
layout: lesson
title: Project - A Phone Stand
author: Kevin McAleer
type: page
cover: /learn/bettercad/assets/banners/11_project_phone_stand.jpg
date: 2026-11-20
previous: 10_saving_sharing_and_printing.html
next: 12_summary.html
description: Design a parametric phone stand in BetterCad from three shapes and five
  parameters, check it prints with no supports, and export it
percent: 84
duration: 8
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

By the end of this lesson you will be able to design a parametric phone stand from three shapes and five parameters, check that it prints without supports, and export it.

Time to put it all together. You'll design a phone stand that prints flat with no supports, and because every size comes from a parameter, you can make it fit a phone, a tablet or a Raspberry Pi display by changing a number.

It's three shapes: a flat **base**, a wedge for the **back rest**, and a small **lip** at the front to stop the phone sliding off.

![The finished phone stand](/learn/bettercad/assets/stand_done.jpg){:class="img-fluid w-100"}

---

## Step 1: Plan the parameters

Good parametric models start with a list of the sizes that matter. Start a new model (<kbd>Ctrl</kbd>+<kbd>N</kbd>), open **Parameters** and add these five.

| Name | Expression | What it controls |
|---|---|---|
| `width` | `70` | How wide the stand is |
| `height` | `50` | How high the back rest reaches |
| `lean` | `50` | How far back the rest leans. Bigger is more laid back |
| `shelf` | `14` | Room for the phone, in its case, to sit on |
| `base` | `4` | Thickness of the base |
{:class="table table-single"}

![Five parameters in the Parameters panel](/learn/bettercad/assets/stand_params.jpg){:class="img-fluid w-100"}

There's a **Comment** column too. Fill it in. Future you will not remember what `shelf` meant.

Close the panel.

## Step 2: The base

The base has to be long enough for the back rest, the shelf and the lip, one after another.

1. Open the **Library** and click **Box**.
2. In Properties, set the size:

| Box | Type this |
|---|---|
| W | `lean + shelf + 4` |
| D | `width` |
| H | `base` |
{:class="table table-single"}

3. Click **Fit**.

![The base plate](/learn/bettercad/assets/stand_base.jpg){:class="img-fluid w-100"}

Properties shows a size of 68 × 70 × 4 mm. The 68 is `50 + 14 + 4`, worked out for you. The `4` is the thickness of the lip you're about to add.

## Step 3: The back rest

The **Wedge** from the Library is a ramp: a box sliced corner to corner. Its sloping face is what the phone leans against.

1. In the Library, click **Wedge**.
2. Set its size and position:

| Box | Type this |
|---|---|
| W | `lean` |
| D | `width` |
| H | `height` |
| Position X | `lean / 2 - (lean + shelf + 4) / 2` |
| Position Z | `base` |
{:class="table table-single"}

![The wedge in place at the back of the base](/learn/bettercad/assets/stand_rest.jpg){:class="img-fluid w-100"}

That X position looks fierce, so take it apart. Shapes are placed by their centres. The base is centred on zero, so its back edge is at minus half its length: `-(lean + shelf + 4) / 2`. The wedge's centre must be half its own length in from there: add `lean / 2`. With the starting values that comes to -9 mm, and you can see that number appear in the box.

Position Z is `base`, which sits the wedge on top of the base plate.

## Step 4: The lip

1. In the Library, click **Box** again.
2. Set its size and position:

| Box | Type this |
|---|---|
| W | `4` |
| D | `width` |
| H | `base + 8` |
| Position X | `(lean + shelf + 4) / 2 - 2` |
{:class="table table-single"}

![The lip at the front of the base](/learn/bettercad/assets/stand_lip.jpg){:class="img-fluid w-100"}

Same idea as before: the front edge of the base is at plus half its length, and the lip's centre is half its 4 mm thickness back from that. The lip starts at the ground and stands 8 mm above the base.

## Step 5: Make it one part

Three shapes that touch are still three shapes. A slicer would print them as three separate objects.

1. Click the first body in the Browser, then <kbd>Shift</kbd>-click the last so all three are selected.

![All three shapes selected](/learn/bettercad/assets/stand_selected.jpg){:class="img-fluid w-100"}

2. Press <kbd>Ctrl</kbd>+<kbd>G</kbd>.

BetterCad reports **Grouped 3 bodies**, and the stand is one solid.

## Step 6: Test the parameters

This is the moment a parametric model pays you back. Open **Parameters** and change `lean` to `70`.

![The stand with lean set to 70](/learn/bettercad/assets/stand_lean_70.jpg){:class="img-fluid w-100"}

The back rest leans further, the base grows to match, and the lip stays at the front where it belongs. Nothing overlaps and nothing is left behind, because every position was written in terms of the same five names.

Try a few more:

| For | Try |
|---|---|
| A big phone in a thick case | `shelf` = `18` |
| A small tablet | `width` = `120`, `height` = `90`, `lean` = `80` |
| A desk display that sits upright | `lean` = `25` |
{:class="table table-single"}

Set `lean` back to `50` when you've finished exploring.

## Step 7: Check it prints

Open **Print settings** from the account menu and click **Overhangs**.

![The overhang check on the phone stand](/learn/bettercad/assets/stand_overhangs.jpg){:class="img-fluid w-100"}

**Nothing overhangs more than 45°, so it needs no support.** That's by design. Lying on its base, every face of the stand either sits on the bed, stands upright or slopes upwards. The wedge is doing the job that a thin leaning panel would need supports for.

Run **Thin walls** too. The thinnest part is the 4 mm base and lip, far above the 0.8 mm limit.

## Step 8: Export

1. Press <kbd>Ctrl</kbd>+<kbd>E</kbd>.
2. Choose **3MF**, leave **Place on the bed** ticked and click **Export**.
3. Press <kbd>Ctrl</kbd>+<kbd>S</kbd> and save the `.bcad` as well.

Properties puts the stand at about 66 g of PLA at 15% infill. Slice it with three walls and it will hold a phone for years.

## Make it yours

A few ways to take it further, all with tools you already know:

- **Round it off.** Fillet the upright edges of the lip and the top edge of the rest. Leave the bottom edges sharp, or chamfer them
- **Add a cable slot.** Put a Box in the middle of the lip, make it a hole and group it, so a charging cable can pass through
- **Save plastic.** Cut a large hole shape through the back rest from side to side
- **Compare notes.** Open the built-in **Phone stand** sample and read its timeline and parameters. It's built the same way

---

> **Teachers**: the whole project, including the print checks and export, can be done on screen with no printer. If learners do print their stands, one printer makes one stand at a time, so have everyone export a 3MF and pass the overhang and thin wall checks first, then queue prints across a later session. Printers have hot nozzles and heated beds, so supervise them as you would any workshop tool.

## Try it Yourself

1. Add a `lip` parameter for the lip thickness and replace every `4` that refers to it. How many places need changing?
2. Add a cable slot 12 mm wide in the lip. Which parameters should its position use so it stays central?
3. Challenge: make a second, wider stand for a tablet by changing only parameters, and check its overhangs.

## Common Issues

- **Problem**: The wedge slopes the wrong way.
- **Solution**: Set its Rotation Z to `180`.
- **Why**: A wedge has a high end and a low end. The high end goes at the back.

- **Problem**: There's a gap between the wedge and the base.
- **Solution**: Check the wedge's Position Z is `base`, not a typed number.
- **Why**: If Z is a plain 4 and you change `base`, the wedge stays where it was.

- **Problem**: After changing a parameter, one shape is left behind.
- **Solution**: Select it and look for a position or size that's a plain number where a name should be.
- **Why**: Only boxes containing a parameter name follow that parameter.

## Check your understanding

{% quiz %}
? Why is the base's width set to `lean + shelf + 4` and not 68?
- [ ] BetterCad can't store 68
- [x] So the base resizes itself when lean or shelf change
- [ ] To make the file smaller
> A sum of parameters is recalculated every time one of them changes.

? Why does this stand print with no supports?
- [x] No face slopes downward by more than 45°
- [ ] It's too small to need them
- [ ] BetterCad adds supports for you
> Every face sits on the bed, stands upright or slopes upwards.

? What does grouping the three shapes do?
- [ ] Deletes two of them
- [ ] Locks their sizes
- [x] Joins them into one solid part
> Grouped solids become a single body, which is what the slicer should see.
{% endquiz %}
