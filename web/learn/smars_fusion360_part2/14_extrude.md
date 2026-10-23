---
layout: lesson
title: Stub wheel cutout
author: Kevin McAleer
type: page
cover: /learn/smars_fusion360_part2/assets/cover.jpg
date: 2023-07-20
previous: 13_cut.html
next: 15_slots.html
description: Cut away the center section
percent: 75
duration: 1
quiz_total: 6
redirect_from: /learn/smars_fusion360/30_extrude.html
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


The [Stub wheel cutout profile](13_cut.html) is used to extrude-cut the section into the stub wheel to create two halves.

The cut operation goes through the stub wheel to the surface of the side of the chassis body.

![Chassis stub wheel cutout Cad Drawing](assets/chassis_stub_wheel_cutout.png){:class="img-fluid w-75 m-3"}

## Check your understanding

{% quiz %}
? How is the round stub wheel made?
- [ ] By extruding a circle
- [x] By **revolving** a profile around the centre of the wheel hole
- [ ] By filleting a cube
> Revolve spins a 2D profile around an axis to make a round solid, like a part turned on a lathe.

? Why does the stub wheel have a 2.5mm slot cut through its middle?
- [ ] To save plastic
- [ ] To let wires through
- [x] So the two halves can squeeze together when you push a wheel on
> The slot makes it springy, so an unpowered wheel snaps on and stays on.
{% endquiz %}
