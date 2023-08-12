---
layout: lesson
title: Sketch
author: Kevin McAleer
type: page
cover: /learn/pcb_design/assets/pcb.png
previous: 08_schematic.html
next: 10_pico.html
description: 4. Schematic & Components
percent: 30
duration: 1
navigation:
- name: Fusion 360 PCB Design for beginners
- content:
  - section: Introduction
    content:
    - name: 1. 3D Model Inititation
      link: 00_intro.html
  - section: 1. 3D Model Initiation
    content:
    - name: Start with a 3d model
      link: 01_start.html
    - name: Circuit Sketch & Outline
      link: 02_create.html
    - name: Extrude
      link: 03_extrude.html
  - section: 2. Sketching and PCB Creation
    content:
    - name: Sketch
      link: 04_sketch.html
    - name: Associative Design
      link: 05_assistate.html
    - name: Push
      link: 06_push.html
  - section: 3. Electronic Design
    content:
    - name: New Electronics Design
      link: 07_create.html
  - section: 4. Schematic & Components
    content:
    - name: New Schematic
      link: 08_schematic.html
    - name: Sketch
      link: 09_components.html
  - section: 5. Raspberry Pi Pico Creation
    content:
    - name: Electronics Library
      link: 10_pico.html
    - name: New Component
      link: 11_library.html
    - name: Symbol
      link: 12_schematic.html
    - name: Sketch
      link: 13_components.html
    - name: Sketch
      link: 14_add.html
  - section: 6. Power and Grounding
    content:
    - name: Sketch
      link: 15_power.html
    - name: Sketch
      link: 16_components.html
    - name: Sketch
      link: 17_nets.html
  - section: 7. Board Layout
    content:
    - name: Sketch
      link: 18_components.html
    - name: Sketch
      link: 19_place.html
    - name: Sketch
      link: 20_route.html
  - section: 8. Modifications & Checks
    content:
    - name: Sketch
      link: 21_mounting.html
    - name: Sketch
      link: 22_3d.html
    - name: Sketch
      link: 23_drc.html
    - name: Sketch
      link: 24_ground.html
  - section: 9. Final Steps
    content:
    - name: Sketch
      link: 25_export.html
    - name: Sketch
      link: 26_gerber.html
---


We’re going to place 4 momentary tact buttons (switches) onto our schematic and connect them to a Raspberry Pi Pico. We’ll also add a battery connector and another 4 buttons

First let’s find the switches in a library. Luckily Fusion has a bunch of libraries we can search in

---

1. Click the Open Library Manager button

    [![Outline](assets/pcb20.jpg){:class="img-fluid w-05 shadow-lg rounded-3"}](assets/pcb20.jpg)

    In the filter section type ‘1-1825910-0’ - this is a model number for a momentary switch (or tactile switch).

1. Click it and then click on the canvas 4 times

    [![Outline](assets/pcb21.jpg){:class="img-fluid w-05 shadow-lg rounded-3"}](assets/pcb21.jpg)

    It will place a symbol each time you click.

1. Press Escape to stop placing this type of symbol

---
