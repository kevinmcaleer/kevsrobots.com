---
layout: lesson
title: Front Back Cut
author: Kevin McAleer
type: page
cover: /learn/smars_fusion360/assets/banners/07_cut.jpg
date: 2023-07-20
previous: 06_front.html
next: 08_side.html
description: Extrude-cut the profile through the whole body
percent: 40
duration: 3
quiz_total: 48
date_updated: 2026-11-24
navigation:
- name: Design a SMARS Robot in Fusion 360 - Part 1, The Chassis Shell
- content:
  - section: Overview
    content:
    - name: Overview
      link: 00_intro.html
  - section: Create the Base
    content:
    - name: The Chassis Base
      link: 01_base.html
    - name: Extrude the Base
      link: 02_extrude_base.html
    - name: Shell the Extruded Base
      link: 03_shell.html
    - name: Fillet the Shell
      link: 04_fillet.html
    - name: Thin the Front and Back sides
      link: 05_thin.html
  - section: Refine the Shape
    content:
    - name: Front Cutout Profile
      link: 06_front.html
    - name: Front Back Cut
      link: 07_cut.html
    - name: Side Profile
      link: 08_side.html
    - name: Side Cutout
      link: 09_sidecut.html
    - name: Fillet applied to side cutouts
      link: 10_fillet.html
    - name: Back cutout profile
      link: 11_back.html
    - name: Back profile cut
      link: 12_back.html
  - section: Wheel Holes and Arduino Slots
    content:
    - name: Chassis Wheel hole profile
      link: 13_wheel.html
    - name: Wheel hole extrude cut
      link: 14_cut.html
    - name: Arduino slot profile
      link: 15_slots.html
    - name: Arduino slot cut
      link: 16_cut.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to use **Extrude** in cut mode to remove material all the way through a body.

---

You have the profile. Now let's use it. This is the second half of the cut recipe from the last lesson: take the sketch and tell Fusion to remove, rather than add, material.

## Cutting through

The profile [created previously](06_front.html) is now used to extrude a cut all the way through the body.

![Chassis Front to back cut Cad Drawing](assets/chassis_front_back_cut.png){:class="img-fluid w-75 m-3"}

It is the same **Extrude** tool you used to make the base, but the operation is set to **Cut**. Instead of making a solid, it subtracts the shape from the body wherever the profile passes through. Setting the extent to go all the way through means one cut makes an opening in both the front and the back wall.

## What to check

Orbit round the chassis. You should be able to look through the front opening and out of the back. The opening should be the same shape at each end.

## Join, cut or new body?

Extrude has more than one operation, and the choice matters:

- **Join** adds the new solid to the body you already have.
- **Cut** removes material.
- **New Body** makes a separate solid that does not touch the first.

Many first-time CAD mistakes come from leaving the operation on the wrong setting. If your cut has added a block instead of removing one, that is why.

> **Teachers**: pair learners up and ask one to predict, before the cut is applied, what the model will look like from the front. It builds the habit of checking before pressing OK.

## Common Issues

- **Problem**: The extrude added material instead of cutting.
- **Solution**: Edit the feature and set the operation to Cut.
- **Why**: The same tool can join, cut or create a new body, depending on the operation.

- **Problem**: Only the front wall was cut, not the back.
- **Solution**: Edit the cut and set the extent so it passes all the way through the body.
- **Why**: A cut only removes material it actually reaches.

## Try it Yourself

1. Edit the cut in the timeline and switch it to Join and then back to Cut. What do you see in each case?
2. Look through the opening and describe what you can see inside the chassis.
3. Challenge: could this cut be made from only one end? What would be different about the back wall?

## Check your understanding

{% quiz %}
? What setting turns Extrude from adding material to removing it?
- [ ] The Shell thickness
- [x] Setting the operation to Cut
- [ ] The fillet radius
> An extrude can join, cut or create a new body. Cut removes material.

? Why does one cut make openings in both the front and back?
- [x] Because the cut goes all the way through the body
- [ ] Because the cut is mirrored
- [ ] Because the shell is open
> Cutting all the way through removes material at both walls.

? Your cut added a block instead of removing one. What is the most likely cause?
- [ ] The sketch was too small
- [x] The operation was left on Join
- [ ] The fillet was too big
> Check the operation setting first.
{% endquiz %}
