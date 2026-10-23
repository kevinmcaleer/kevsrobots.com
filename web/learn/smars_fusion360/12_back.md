---
layout: lesson
title: Back profile cut
author: Kevin McAleer
type: page
cover: /learn/smars_fusion360/assets/smars_fusion360.png
date: 2023-07-20
previous: 11_back.html
next: 13_wheel.html
description: Cut out the back
percent: 65
duration: 1
quiz_total: 7
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


The [profile created above](11_back.html) is used to extrude a cut through the back side of the body. This is where the Arduino will slide in. The slots will come shortly.

![Chassis Back profile cut Cad Drawing](assets/chassis_back_extrude.png){:class="img-fluid w-75 m-3"}

## Check your understanding

{% quiz %}
? How do you cut a shape out of a solid body in Fusion 360?
- [ ] Draw the shape and press Delete
- [x] Sketch a profile, then Extrude it with the operation set to **Cut**
- [ ] Use the Shell tool on the profile
> An extrude can join, cut or create a new body. Setting it to Cut removes material wherever the profile passes through.

? What shape is the side cutout profile?
- [ ] A circle
- [ ] A rectangle
- [x] An inverted trapezium
> The 28mm top, 16mm bottom trapezium gives SMARS its classic tank-like side shape.
{% endquiz %}
