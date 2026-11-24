---
layout: lesson
title: Stub extrude profile
author: Kevin McAleer
type: page
cover: /learn/smars_fusion360_part2/assets/banners/09_cut.jpg
date: 2023-07-20
previous: 08_stub.html
next: 10_stub.html
description: Cut the profile
percent: 50
duration: 3
quiz_total: 51
redirect_from: /learn/smars_fusion360/25_cut.html
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

By the end of this lesson you will be able to sketch the circle that links the stub to the chassis, centred on the wheel hole from Part 1.

## A base for the stub

A small extrusion is needed to join the stub wheel to the fillet at the bottom of the chassis. Without it the stub would have nothing solid to attach to.

Dimension:

* stub wheel diameter = 15mm

The center of the circle is located at the centre of the [wheel hole](/learn/smars_fusion360/13_wheel.html) you cut in Part 1.

![Chassis stub extrude profile Cad Drawing](assets/chassis_stub_extrude.png){:class="img-fluid w-75 m-3"}

## Why the centre matters

Everything about the wheel depends on this centre being right. The unpowered wheel sits around it, so if the circle is a little off the wheel will rub on the chassis. Rather than clicking near the hole, snap the centre of the circle to the centre of the hole from Part 1, so the two are exactly concentric.

A note on numbers: this lesson gives the circle as 15mm, while the stub profile in [the previous lesson](08_stub.html) is 16mm across. Both are shown exactly as in the original lessons, so follow each one as written and check against the drawings.

**Check after this step:** the circle is centred on the wheel hole and is 15mm across.

## Try it Yourself

1. Start the sketch on the side of the chassis and snap the circle to the hole centre.
2. Check the circle and the hole share a centre point.
3. Challenge: why is it better to snap to a centre than to type distances from the edges?

## Common Issues

- **Problem**: The circle looks slightly off the wheel hole.
- **Solution**: Delete it and click the hole's centre point when placing the new circle.
- **Why**: Clicking close is not the same as snapping to the centre, and the difference shows as a wobble.

- **Problem**: The sketch is on the wrong side of the chassis.
- **Solution**: Check which face you selected and restart on the face with the wheel hole.
- **Why**: The profile must sit on the face where the stub will be built.

## Check your understanding

{% quiz %}
? What diameter is the circle in this lesson?
- [x] 15mm
- [ ] 3mm
- [ ] 2.5mm
> The stub wheel diameter in this lesson is 15mm.

? Where is the circle centred?
- [ ] On the chassis edge
- [x] On the centre of the wheel hole from Part 1
- [ ] On the nub
> The wheel hole centre is the reference.

? Why snap to the hole centre?
- [x] So the wheel is exactly concentric and does not rub
- [ ] To save time printing
- [ ] To make the circle bigger
> Concentric circles keep the wheel centred.
{% endquiz %}
