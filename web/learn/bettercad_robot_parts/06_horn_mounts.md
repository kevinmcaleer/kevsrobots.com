---
layout: lesson
title: Horn Mounts
author: Kevin McAleer
type: page
cover: /learn/bettercad_robot_parts/assets/banners/06_horn_mounts.jpg
date: 2026-11-21
previous: 05_standoffs_for_a_board.html
next: 07_your_first_gear.html
description: Bolt a servo horn to a 3D printed part with the BetterCad Horn mount,
  with no printed splines to strip
percent: 49
duration: 5
quiz_total: 42
date_updated: 2026-11-21
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

A servo's output shaft has tiny splines, far too fine to print well. Try, and you get a part that grips for a week and then slips.

Every servo comes with the fix in the bag: a set of plastic **horns** that already fit the splines perfectly. So don't print the splines. Print a part that the horn **bolts** to.

---

## What a horn mount is

BetterCad's Horn mount cuts three things into a flat face:

- A shallow round **recess** that the horn's hub sits in, so it's centred
- A **screwdriver hole** through the middle, so you can still reach the screw that holds the horn on the servo
- **Screw holes** where the horn's arms are, to bolt the horn to your part

## Make one

A small turntable is a good test piece.

1. Start a new model. Add a **Cylinder** from the Library and set its size to `40`, `40` and `5`.
2. Click the word **Insert** and choose **Mount**.
3. Click the top of the disc.
4. In the Mount panel, set **Kind** to **Horn mount**.

![The Horn mount settings on a 40 mm disc](/learn/bettercad_robot_parts/assets/horn_mount.jpg){:class="img-fluid w-100"}

| Setting | What it is | Starts at |
|---|---|---|
| **Screw** | The screws through the horn's arms | M2.5 |
| **Horn** | Single, Double or Cross | Single |
| **Hub** | Diameter of the recess for the horn's hub | 7.5 mm |
| **Recess** | Depth of that recess | 2 mm |
| **Arm hole** | How far out the arm screw holes are from the centre | 10 mm |
| **Screwdriver hole** | Diameter of the hole through the middle | 3 mm |
| **Centre X** and **Y** | Where the horn's centre is, from the face's centre | Where you clicked |
{:class="table table-single"}

5. Choose **Cross** for a four-armed horn.
6. Set **Screw** to **M2**, which suits the small holes in a micro servo's horn.
7. Set **Centre X** and **Centre Y** to `0` so the horn is dead centre on the disc.
8. Click **OK**.

![The finished horn mount from above](/learn/bettercad_robot_parts/assets/horn_done.jpg){:class="img-fluid w-100"}

One hub recess with a hole through it, and four screw holes round it. **Single** gives one arm hole, **Double** gives two opposite each other, and **Cross** gives four.

## Measure your horn

The panel says it plainly: "Measure your horn: these suit a micro servo's."

Horns differ even more than servos do. Before you print, check three numbers with callipers:

1. The **hub's diameter**, the round boss in the middle. Set **Hub** a few tenths bigger.
2. The **distance from the centre to the hole you'll use** on an arm. Horn arms have a row of holes. Pick one and set **Arm hole** to its distance from the centre.
3. The **hole size** in the arm. Most micro servo horns take M2 or the small self-tapping screws that came in the bag.

## Which way up?

Think about how it goes together. The horn pushes onto the servo. Your printed part sits on the horn, hub in the recess. Screws go through the horn's arms into your part, and the horn's own centre screw goes in through the screwdriver hole.

So the horn mount goes on the face that touches the horn. On a turntable that's the underside, which means you model it on top and flip the part to print, or click the bottom face when you place the mount.

---

## Try it Yourself

1. Edit the Mount step and switch **Horn** between Single, Double and Cross. Watch the holes change.
2. Set **Recess** to 0. What's left?
3. Measure a horn from your own servo and enter its three numbers.

## Common Issues

**Problem**: The horn won't sit in the recess.
**Solution**: Increase **Hub** by 0.2 to 0.4 mm.
**Why**: Printed holes and recesses come out a little small.

**Problem**: The arm screw holes don't line up with the horn.
**Solution**: Measure from the horn's centre to the hole you want and set **Arm hole** to that.
**Why**: The defaults suit a common micro servo horn. Yours may be longer.

**Problem**: I can't reach the horn's centre screw.
**Solution**: Make **Screwdriver hole** wide enough for your screwdriver's shaft, not just its tip.
**Why**: The hole goes right through so the screw can be done up after the part is on.

## Check your understanding

{% quiz %}
? Why bolt a horn to a printed part instead of printing the splines?
- [ ] Printed splines are too strong
- [x] Splines are too fine to print well and soon slip
- [ ] BetterCad can't draw splines
> The horn in the bag already fits the shaft perfectly.

? What is the screwdriver hole for?
- [ ] Hanging the part on a hook
- [ ] Letting air out of the recess
- [x] Reaching the screw that holds the horn on the servo
> It runs through the middle of the hub recess.

? How many arm screw holes does a Cross horn mount make?
- [ ] One
- [ ] Two
- [x] Four
> Single makes one, Double two and Cross four.
{% endquiz %}
