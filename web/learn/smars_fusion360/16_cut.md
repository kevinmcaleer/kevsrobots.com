---
layout: lesson
title: Arduino slot cut
author: Kevin McAleer
type: page
cover: /learn/smars_fusion360/assets/smars_fusion360.png
date: 2023-07-20
previous: 15_slots.html
description: Cut the slots out
percent: 100
duration: 2
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


The [Arduino slot profile](15_slots.html) created previously is used to extrude a cut into the body, to a depth of 0.5mm.

* Arduino slot depth 0.5mm

This feature is mirrored across to the other side of the chassis.

![Chassis Arduino slot cut Cad Drawing](assets/chassis_arduino_slot_cut.png){:class="img-fluid w-75 m-3"}

## Where you've got to

That's the outer shell finished! You now have the base, the shaped sides, the wheel holes and the slots the Arduino slides into. Save your design, because Part 2 builds on it.

## Check your understanding

{% quiz %}
? Why is the wheel hole 4.5mm across when the motor spindle is only 3mm?
- [x] To give the spindle plenty of clearance so it turns freely
- [ ] To make the chassis lighter
- [ ] So the wheel can push through the hole
> 3D prints aren't perfectly accurate, so holes need some clearance. 4.5mm leaves room for the 3mm spindle.

? You cut a slot on one side of the chassis. What's the quickest way to get the same slot on the other side?
- [ ] Sketch and cut it again
- [x] **Mirror** the feature across the middle of the chassis
- [ ] Use the Shell tool
> Mirror copies a feature across a plane, so both sides stay identical, even if you change the original later.
{% endquiz %}

## Try it yourself

1. Change the shell thickness from 3mm to 2.5mm in the timeline and watch the whole design update.
2. Measure your Arduino Uno board: will it slide into the slots you've cut?

## What's next

Carry on with [Part 2: Motor Holders and Stub Wheels](/learn/smars_fusion360_part2/) to finish the chassis.
