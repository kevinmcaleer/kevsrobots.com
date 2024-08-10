---
layout: lesson
title: Assembling the Robot
author: Kevin McAleer
type: page
cover: assets/assembly.png
date: 2024-08-02
previous: 02_components.html
next: 04_wiring.html
description: Step-by-step guide to assembling the wall drawing robot.
percent: 27
duration: 2
date_updated: 2024-08-02
navigation:
- name: Wall Drawing Robot Tutorial
- content:
  - section: Introduction
    content:
    - name: Introduction to the Wall Drawing Robot
      link: 01_intro.html
  - section: Build the Robot
    content:
    - name: Components Needed
      link: 02_components.html
    - name: Assembling the Robot
      link: 03_assembly.html
    - name: Wiring the Components
      link: 04_wiring.html
  - section: Programming
    content:
    - name: Setting Up the Raspberry Pi
      link: 05_setup_pi.html
    - name: Installing the Software
      link: 06_software.html
    - name: Writing Control Code
      link: 07_control.html
    - name: Understanding Positioning with Trigonometry
      link: 08_positioning.html
    - name: Creating Your First Drawing
      link: 09_draw.html
    - name: Vectorizing Images for Drawing
      link: 10_vectorize.html
  - section: Summary
    content:
    - name: Summary and Next Steps
      link: 11_summary.html
---


![Assembly](assets/wiring01.png){:class="cover"}

---

## Assembly

Follow the instructions below to assemble the wall drawing robot parts:

Item | Action
--- | ---
**Mount the Stepper Motors**: Secure the stepper motors with two screws to the left side and right sides 3D printed parts. <br /><br /> **Mount the Raspberry Pi Zero 2 W**: Attach the Raspberry Pi to the top of the left side motor mount using M2.5 screws.<br /><br/> **Plug the Stepper Motors**: Connect the stepper motors to the driver boards using connector. | ![Assembly](assets/assembly01.png){:class="w-100 img-fluid rounded-3 card-shadow card-hover"}
**Mount the ULN2003 Driver Boards**: Fix the driver boards to the back of the left and right side motor mounts. | ![Assembly](assets/assembly02.png){:class="w-100 img-fluid rounded-3 card-shadow card-hover"}
**Attach the Pulley String**: Connect the toothed pulley string to both stepper motors, ensuring it runs smoothly. | ![Assembly](assets/assembly04.png){:class="w-100 img-fluid rounded-3 card-shadow card-hover"}
**Prepare the Gondola**: Attach the marker pen to the gondola and secure the micro servo motor to the gondola. The servo will lift and lower the pen. <br /><br /> **Connect the Gondola**: Suspend the gondola between the two motors using the pulley string. Ensure it can move freely up and down. | ![Assembly](assets/assembly03.png){:class="w-100 img-fluid rounded-3 card-shadow card-hover"}
**Attach the timing cogs** Secure the timing cogs to the stepper motors using the small grub screws | ![Assembly](assets/assembly05.png){:class="w-100 img-fluid rounded-3 card-shadow card-hover"}
**Attach to board**: Secure the left and right assemblies to the whiteboard | ![Assembly](assets/assembly06.png){:class="w-100 img-fluid rounded-3 card-shadow card-hover"}
**Attach to the whiteboard**: Secure the left and right side motor mounts to the whiteboard using screws. | ![Assembly](assets/assembly07.png){:class="w-100 img-fluid rounded-3 card-shadow card-hover"}
{:class="table table-striped"}

---

Your robot assembly is complete. Now, let's move on to wiring the components.

---
