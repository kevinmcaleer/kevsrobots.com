---
layout: lesson
title: Motor holder and nub chamfer
author: Kevin McAleer
type: page
cover: /learn/smars_fusion360_part2/assets/cover.jpg
date: 2023-07-20
previous: 04_extrude.html
next: 06_front.html
description: Refine the tops
percent: 30
duration: 1
quiz_total: 6
redirect_from: /learn/smars_fusion360/21_chamfer.html
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


A small chamfer is applied to the tops of the extruded [motor holder profiles](01_motor.html) and [motor nubs](04_extrude.html).

The dimension of the chamfer is:

- motor holder chamfer = 1mm

![Chassis Motor holder and nub chamfer Cad Drawing](assets/chassis_motor_holder_chamfer.png){:class="img-fluid w-75 m-3"}

## Check your understanding

{% quiz %}
? What are the small motor nubs for?
- [ ] To hold the battery
- [x] To stop the motors sliding backwards into the chassis
- [ ] To clip the lid on
> The 2mm nubs sit behind each motor and hold it in place.

? What does a **Chamfer** do to an edge?
- [x] Cuts it at an angle
- [ ] Rounds it into a curve
- [ ] Makes it thicker
> A chamfer bevels an edge at an angle. A fillet rounds it instead.
{% endquiz %}
