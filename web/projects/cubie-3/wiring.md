---
title: "Cubie-3 - Wiring"
description: >-
    An open-source 3d printable cube robot you can build yourself
excerpt: >-
    
layout: multipage
date: 2026-01-07
author: Kevin McAleer
difficulty: beginner
cover: /projects/cubie-3/assets/img/cover.jpg
hero:  /projects/cubie-3/assets/img/hero.png
mode: light
tags:
 - robots
 - cubie-3
groups:
 - robots
videos:
 - K-bFD4RvypU
navigation:
 - name: "Overview"
   link: index
 - name: "Bill of Materials"
   link: bom
 - name: "Circuit Diagram"
   link: circuit
 - name: "Wiring"
   link: wiring
 - name: "Assembly"
   link: assembly
 - name: "Code"
   link: code
 - name: "Downloadable STL files"
   link: stl

next: assembly
previous: circuit
---

## Raspberry Pi Camera cable

- Black section of the ribbon cable faces the USB ports on the Raspberry Pi.
- The camera cable connects to the Camera port on the Raspberry Pi 5 and the Camera Module v3.
- The Black sesion of the ribbon cable faces you when looking at the back of the Camera Module v3.

---

## Battery

- The battery connects to the Sunfounder Fusion HAT+ via the JST connector.

---

## Motors

- Each motor connects to the Sunfounder Fusion HAT+ via the white 2 pin connectors. Unsoldered cables are provided, these will need to be soldered to the motor wires.
- For consistency, when wiring up the n20 motors look for the tiny `+` symbol on the motor PCB and connect the red wire to this side.

---