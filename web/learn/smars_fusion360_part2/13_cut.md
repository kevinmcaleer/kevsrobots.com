---
layout: lesson
title: Stub wheel cutout profile
author: Kevin McAleer
type: page
cover: /learn/smars_fusion360_part2/assets/banners/13_cut.jpg
date: 2023-07-20
previous: 12_revolve.html
next: 14_extrude.html
description: Cut out the center
percent: 70
duration: 3
quiz_total: 51
redirect_from: /learn/smars_fusion360/29_cut.html
date_updated: 2026-11-24
navigation:
- name: Design a SMARS Robot in Fusion 360 - Part 2, Motor Holders and Stub Wheels
- content:
  - section: Overview
    content:
    - name: Overview
      link: 00_intro.html
  - section: Motor Holders
    content:
    - name: Motor holder profile
      link: 01_motor.html
    - name: Motor holder profile extruded
      link: 02_extrude.html
    - name: Motor nub profile
      link: 03_nub.html
    - name: Motor nub extrude
      link: 04_extrude.html
    - name: Motor holder and nub chamfer
      link: 05_chamfer.html
  - section: Front Cut
    content:
    - name: Front profile
      link: 06_front.html
    - name: Front cut
      link: 07_cut.html
  - section: Stub Wheels
    content:
    - name: Unpowered wheel stub profile
      link: 08_stub.html
    - name: Stub extrude profile
      link: 09_cut.html
    - name: Stub extrude
      link: 10_stub.html
    - name: Stub wheel profile
      link: 11_stub.html
    - name: Stub wheel revolve
      link: 12_revolve.html
    - name: Stub wheel cutout profile
      link: 13_cut.html
    - name: Stub wheel cutout
      link: 14_extrude.html
  - section: Motor Holder Slots
    content:
    - name: Motor holder slots profile
      link: 15_slots.html
    - name: Motor holder slot extrude-cut
      link: 16_cut.html
  - section: Final Design
    content:
    - name: Final view
      link: 17_final.html
---


![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to sketch the slot profile that lets the stub wheel flex like a spring.

## Why cut a slot in a solid part?

The stub wheel needs a cut out section to allow the two halves of the stub wheel to squeeze together when pushing on an unpowered wheel. This is a snap fit: the halves flex inward as the wheel is pushed on, then spring back out so the wheel stays put without a screw.

The dimensions are:

* cutout width = 2.5mm
* cutout height = 16mm (the wheel diameter)

![Chassis stub wheel cutout profile Cad Drawing](assets/chassis_stub_wheel_cutout_profile.png){:class="img-fluid w-75 m-3"}

## Design for printing

Snap fits are a 3D printing favourite because they need no screws, but plastic is stiff. A thin part bent too far can crack, especially along the layer lines. The slot gives the halves room to move a little without needing much force.

```mermaid
flowchart LR
    A[Push wheel on] --> B[Halves squeeze together into the slot]
    B --> C[Wheel passes into place]
    C --> D[Halves spring back and hold the wheel]
```

The height of 16mm matches the wheel diameter, so the slot runs right across the stub.

**Check after this step:** the profile is 2.5mm wide and 16mm long and sits centred across the stub, as in the drawing.

> **Teachers**: Snap-fit parts depend on your printer's accuracy, so print one stub as a test before printing a class set. Different printers and filaments give slightly different fits, so allow time to adjust and reprint.

## Try it Yourself

1. Sketch the profile and add both dimensions.
2. Check it is centred over the stub.
3. Challenge: predict what would happen to the spring if the slot were much narrower, and if it were much wider.

## Common Issues

- **Problem**: The profile is not centred across the stub.
- **Solution**: Add a midpoint or symmetry constraint to the stub centre.
- **Why**: An off-centre slot makes one half thicker than the other, so the flex is uneven.

- **Problem**: The sketch is on the wrong plane.
- **Solution**: Restart on the plane or face shown in the drawing.
- **Why**: The slot must pass through the stub to split it into two halves.

## Check your understanding

{% quiz %}
? Why does the stub wheel have a slot?
- [ ] To save plastic
- [x] So the two halves can squeeze together when a wheel is pushed on
- [ ] To let wires through
> The cut out allows the halves to squeeze together.

? What is the cutout width?
- [x] 2.5mm
- [ ] 16mm
- [ ] 1mm
> The width is 2.5mm; the height is 16mm.

? What kind of fit is this?
- [ ] A screw fit
- [x] A snap fit
- [ ] A glue fit
> The halves flex, then spring back to hold the wheel.
{% endquiz %}
