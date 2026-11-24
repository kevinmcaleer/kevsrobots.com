---
layout: lesson
title: Fillet the Shell
author: Kevin McAleer
type: page
cover: /learn/smars_fusion360/assets/banners/04_fillet.jpg
date: 2023-07-20
previous: 03_shell.html
next: 05_thin.html
description: Round the bottom edges and corners with a 4mm fillet
percent: 25
duration: 4
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

By the end of this lesson you will be able to use **Fillet** to round edges and explain why rounded edges help a 3D-printed part.

---

Sharp edges look harsh, catch on things, and are weak points on a printed part. A **fillet** replaces a sharp edge with a smooth curve. The size you type is the radius of that curve.

## Rounding the chassis

The Chassis is now given a fillet around the bottom edges and each of the corners, to make it pleasing to the eye.

* Roundness = 4mm

![Chassis Fillet the Shell Cad Drawing](assets/chassis_roundness.png){:class="img-fluid w-75 m-3"}

Select the edges shown in the picture, start the Fillet tool and type 4mm. All the selected edges are rounded together as a single feature.

## Why fillets help a print

- **Strength**: sharp inside and outside corners concentrate stress. A curve spreads it out, so the part is less likely to crack.
- **Printing**: the bottom edge of a printed part sits on the build plate. A rounded bottom edge helps avoid the slightly squashed, flared edge that can happen on the first layer.
- **Handling**: your robot will be picked up, dropped and knocked into furniture. Rounded corners are kinder to hands and to the robot.

The same tool is used again in lesson 10, with a much smaller value, to soften the edges of the side cut-outs.

## What to check

Zoom in on a corner. The edge should look like a smooth curve with no kinks or gaps. If a corner looks flat or one edge is still sharp, that edge was missed in the selection.

## Common Issues

- **Problem**: Fillet fails, or reports it cannot be built.
- **Solution**: Check you typed 4mm and selected only the edges shown. Try a slightly smaller value to test.
- **Why**: A fillet cannot be larger than the geometry next to it. If the radius is too big for the wall, Fusion cannot make the curve.

- **Problem**: Some edges are rounded and others are not.
- **Solution**: Edit the fillet in the timeline and add the missing edges to the selection.
- **Why**: Only the edges you select get rounded.

## Try it Yourself

1. Edit the fillet and change 4mm to 8mm. Does it still work? What changed? Return it to 4mm.
2. Compare a sharp corner and a filleted one by zooming in. Which would you prefer to print?
3. Challenge: the fillet is done before the front and back are thinned. Edit the fillet in the timeline and watch what the later steps do. Why is the timeline order important?

## Check your understanding

{% quiz %}
? What does the number you type into the Fillet tool mean?
- [ ] The number of edges to round
- [x] The radius of the curve that replaces the sharp edge
- [ ] The wall thickness
> The fillet value is the radius of the rounding, here 4mm.

? Which of these is a reason to round the edges of a 3D-printed part?
- [x] Rounded corners spread stress and are less likely to crack
- [ ] Rounded corners always print faster
- [ ] Sharp corners cannot be printed
> Fillets reduce stress concentration and make the part nicer to handle.

? You apply a fillet and some edges are still sharp. What should you do?
- [ ] Start the whole design again
- [x] Edit the fillet in the timeline and add the missing edges
- [ ] Use the Shell tool
> The timeline lets you edit an earlier feature without redoing the later steps.
{% endquiz %}
