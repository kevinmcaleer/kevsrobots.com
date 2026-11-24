---
layout: lesson
title: Motor holder profile
author: Kevin McAleer
type: page
cover: /learn/smars_fusion360_part2/assets/banners/01_motor.jpg
date: 2023-07-20
previous: 00_intro.html
next: 02_extrude.html
description: Sketch the motor holder profile inside the chassis
percent: 10
duration: 4
quiz_total: 51
redirect_from: /learn/smars_fusion360/17_motor.html
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

By the end of this lesson you will be able to sketch the motor holder profile and use a circular pattern to copy half of it to the opposite side of the chassis.

## What are we making?

A SMARS robot gets its drive from two small motors, one on each side. Those motors need somewhere to sit that holds them still, otherwise they rattle about and the wheels wander. The motor holders are thin walls inside the chassis, and the sketch you draw now is the footprint of all of them.

Let's build it. These are the dimensions in the sketch:

* motor holder side wall thickness = 2mm
* motor holder side wall edge from body edge = 16.5mm
* motor holder wall height = 29.50mm
* unpowered wheel block from case edge = 13mm
* unpowered wheel block and battery nub from edge = 19mm
* battery nub height = 10mm
* battery nub thickness = 3mm

![Chassis Motor holder profile Cad Drawing](assets/chassis_motor_holder_profile.png){:class="img-fluid w-75 m-3"}

## Draw half, then pattern it

You do not need to draw every line twice. The top half of the sketch can be created and then a circular pattern used to rotate this around the centre to create an asymmetric mirror for the bottom half of the sketch.

Why a pattern instead of a mirror? A mirror flips a shape, so the two halves would be reflections of each other. Here you want a copy that is turned around the centre instead, which is what "asymmetric mirror" means in the original instructions. Drawing once and patterning also means that if you fix a dimension on the top half, the bottom half follows.

```mermaid
flowchart LR
    A[Sketch the top half] --> B[Add every dimension]
    B --> C[Circular pattern around the centre]
    C --> D[Check both halves match the drawing]
```

**Check after this step:** compare your sketch with the drawing. Both halves should look the same, turned around the centre, and every dimension in the list above should appear on one half or the other. If a line is still blue, it is not fully constrained yet, so add the missing dimension.

## Try it Yourself

1. Match each dimension in the list above to a line in your sketch. Can you find all seven?
2. Temporarily change the 2mm wall thickness, watch which other lines move with it, then undo.
3. Challenge: explain in your own words why a circular pattern was chosen here instead of a mirror.

## Common Issues

- **Problem**: The bottom half of the sketch looks flipped instead of turned.
- **Solution**: Undo and use a circular pattern around the centre point, not a mirror.
- **Why**: A mirror reflects the shape across a line, while the pattern rotates it around the middle.

- **Problem**: Lines stay blue and the sketch will not fully lock.
- **Solution**: Dimension each line from the list above until the sketch turns black.
- **Why**: Fusion shows blue for geometry that can still move, and unconstrained lines can shift later when you edit something nearby.

## Check your understanding

{% quiz %}
? Why use a circular pattern rather than a mirror for the bottom half of this sketch?
- [ ] Mirrors are not available in sketches
- [x] The pattern rotates the shape around the centre, giving an asymmetric copy rather than a reflection
- [ ] A pattern makes the walls thicker
> The lesson describes the pattern as rotating the top half around the centre to make an asymmetric mirror.

? What is the motor holder side wall thickness in this sketch?
- [ ] 1mm
- [x] 2mm
- [ ] 3mm
> The first dimension in the list is a 2mm side wall.

? What does the finished sketch describe, in plain terms?
- [ ] The wheels
- [x] The footprint of the motor holders and nearby blocks inside the chassis
- [ ] The lid
> This profile is extruded in the next lesson to make the walls.
{% endquiz %}
