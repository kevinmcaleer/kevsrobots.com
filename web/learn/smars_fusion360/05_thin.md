---
layout: lesson
title: Thin the Front and Back sides
author: Kevin McAleer
type: page
cover: /learn/smars_fusion360/assets/smars_fusion360.png
date: 2023-07-20
previous: 04_fillet.html
next: 06_front.html
description: Adjust the wall thickness
percent: 30
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


The Front and Back sides are currently the case thickness from the shell we applied earlier. This needs to be trimmed down so that they are only 2mm thick.

![Chassis Front and Back sides Cad Drawing](assets/chassis_thinned.png){:class="img-fluid w-75 m-3"}

## Check your understanding

{% quiz %}
? What does the **Shell** tool do to the extruded base?
- [ ] Rounds off the corners
- [x] Hollows it out, leaving walls of a set thickness
- [ ] Copies it to the other side
> Shell removes the inside of a solid and leaves walls 3mm thick, giving us the open box shape of the chassis.

? How big is the chassis base sketch?
- [x] 70mm long and 58mm wide
- [ ] 58mm long and 32mm wide
- [ ] 100mm long and 70mm wide
> The base is 70mm by 58mm, then extruded 32mm high.

? Why do we add a 4mm fillet to the bottom edges and corners?
- [ ] To make the walls thinner
- [ ] So the Arduino fits
- [x] To round the edges so the chassis looks and feels better
> Fillets round sharp edges. On a 3D print they also make corners stronger and nicer to handle.
{% endquiz %}
