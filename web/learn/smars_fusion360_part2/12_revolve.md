---
layout: lesson
title: Stub wheel revolve
author: Kevin McAleer
type: page
cover: /learn/smars_fusion360_part2/assets/banners/12_revolve.jpg
date: 2023-07-20
previous: 11_stub.html
next: 13_cut.html
description: Revolve the profile to create the stub
percent: 65
duration: 3
quiz_total: 51
redirect_from: /learn/smars_fusion360/28_revolve.html
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

By the end of this lesson you will be able to use the Revolve tool to spin a profile into a solid stub wheel joined to the chassis.

## Spinning the profile

The [Stub wheel profile](11_stub.html) created above is revolved around the center of the wheel hole to create a solid body. The revolve operation also joins this to the chassis.

![Chassis stub wheel revolve Cad Drawing](assets/chassis_stub_wheel_revolve.png){:class="img-fluid w-75 m-3"}

## What Revolve asks for

Revolve needs three choices: the profile, the axis to spin around, and how far round to go. Choose the profile from the last lesson, pick the centre of the wheel hole as the axis, and sweep it all the way round so the stub is a complete solid. The preview shows a round body forming, which is your check that the axis is right.

Why revolve instead of extruding circles stacked on top of each other? One profile gives you every ring and taper in a single feature. To tweak the shape later you edit one sketch and the whole stub updates.

```mermaid
flowchart TD
    A[Profile] --> D[Revolve]
    B[Axis at wheel hole centre] --> D
    C[Full turn] --> D
    D --> E[Round stub joined to chassis]
```

**Check after this step:** the stub is round when you orbit around it, centred on the wheel hole, and joined to the chassis as one body.

## Try it Yourself

1. Revolve the profile and orbit around the stub.
2. Look at the stub from the side and check it is centred on the wheel hole.
3. Challenge: edit the profile sketch from the timeline, watch how the stub changes, then undo.

## Common Issues

- **Problem**: The stub is lopsided or off to one side.
- **Solution**: Edit the revolve and reselect the axis at the centre of the wheel hole.
- **Why**: The axis is the centre of the finished part, so the wrong axis moves the whole stub.

- **Problem**: The stub is a separate body.
- **Solution**: Undo and revolve again with the operation set to join.
- **Why**: A separate body would not be attached to the chassis in the print.

## Check your understanding

{% quiz %}
? What does Revolve do?
- [ ] Copies a body to the other side
- [x] Spins a 2D profile around an axis to make a round solid
- [ ] Rounds an edge
> This is the same idea as turning a part on a lathe.

? What is the axis for this revolve?
- [x] The centre of the wheel hole
- [ ] The edge of the chassis
- [ ] The motor nub
> The profile is revolved around the center of the wheel hole.

? What does the revolve also do?
- [ ] Removes a hole
- [x] Joins the stub to the chassis
- [ ] Mirrors the chassis
> The revolve operation also joins the stub to the chassis.
{% endquiz %}
