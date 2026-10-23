---
layout: lesson
title: Final view
author: Kevin McAleer
type: page
cover: /learn/smars_fusion360_part2/assets/cover.jpg
date: 2023-07-20
previous: 16_cut.html
description: The completed design
percent: 100
duration: 2
quiz_total: 6
redirect_from: /learn/smars_fusion360/33_final.html
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


You should now have a finished Chassis design, ready for saving out to an STL file and printing.

(Note this design has a few extra flourishes)

![Chassis final Cad Drawing](assets/chassis_final.png){:class="img-fluid w-75 m-3"}

## Export it for printing

1. In the browser on the left, right-click the chassis body.
2. Choose **Save As Mesh**, pick **STL** and set the refinement to **High**.
3. Open the STL in your slicer. Print it with the base flat on the bed; PLA at 0.2mm layer height works well.

## Check your understanding

{% quiz %}
? Why are the motor holder slots only 1.5mm deep?
- [ ] To save printing time
- [x] The motor holder parts are a pressure fit, so they only need a shallow slot
- [ ] Fusion 360 can't cut deeper slots
> A pressure fit relies on a snug fit rather than screws, so a short, tight slot is enough.

? Which file format do you export to 3D print your chassis?
- [ ] PNG
- [x] STL
- [ ] PDF
> STL describes the surface of the part as triangles, which is what slicers like Cura and PrusaSlicer read.
{% endquiz %}

## Try it yourself

1. Add your initials to the side of the chassis with the **Text** tool and an extrude-cut.
2. Challenge: design a lid that clips onto the top of the chassis.

## What's next

- Build and code your robot with [Build a SMARS robot](/learn/smars/) and [SMARS programming](/learn/smars_code/)
- Prefer free and open source software? Design SMARS in [FreeCAD](/learn/freecad_smars/)
