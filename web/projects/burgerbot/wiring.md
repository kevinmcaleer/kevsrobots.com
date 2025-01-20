---
title: "Wiring"
description: >-
    An open-source, burger-shared robot you can build yourself
excerpt: >-
    
layout: multipage
date: 2024-12-04
author: Kevin McAleer
difficulty: beginner
cover: /assets/img/burgerbot/cover.jpg
hero:  /assets/img/burgerbot/hero.png
mode: light
tags:
 - robots
 - burgerbot
groups:
 - robots
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
---

## Drop in a Pico

Wiring up the Custom PCB is just a matter of dropping in the component and soldering them in place. The PCB is designed to be easy to solder, with large pads and clear silkscreen.

---

## Motor Driver board

The motor driver board will need to be soldered to the PCB - ensure the pins are aligned correctly. You can either surface mount this on the board, or solder header pins on the PCB and drop the motor driver board in place.

---

## Motors & Ultrasonic Sensor

The motors will need to have wires soldered to them, and the ultrasonic sensor will need to be connected to the PCB - ensure its facing outwards and the correct pins will be aligned.

![Wiring](/projects/burgerbot/assets/img/wiring02.png){:class="w-100 card-shadow rounded-3"}

---

## Battery Box

The battery box will also need to be connected to the PCB either directly or via a header pin and Dupont connector.

---

![Wiring](/projects/burgerbot/assets/img/wiring.jpg){:class="w-100 card-shadow rounded-3"}

---
  
  {% include centered_nav.html next="Assembly" next_link="assembly.html" previous="Circuit Diagram" previous_link="circuit.html" %}

---
