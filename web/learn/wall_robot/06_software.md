---
layout: lesson
title: Installing the Software
author: Kevin McAleer
type: page
cover: assets/software.png
date: 2024-08-02
previous: 05_setup_pi.html
next: 07_control.html
description: Guide to installing the necessary software for the wall drawing robot.
percent: 60
duration: 1
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
    - name: Creating Your First Drawing
      link: 08_draw.html
    - name: Vectorizing Images for Drawing
      link: 09_vectorize.html
  - section: Summary
    content:
    - name: Summary and Next Steps
      link: 10_summary.html
---


![Software](assets/software.png){:class="cover"}

---

## Installing the Software

1. **Install Python and Required Libraries**:
   - Open a terminal and run:
     ```sh
     sudo apt-get install python3 python3-pip
     pip3 install RPi.GPIO pigpio
     ```

2. **Install the Stepper Motor Control Library**:
   - Run:
     ```sh
     pip3 install stepper_motor
     ```

3. **Clone the Project Repository**:
   - Clone the code repository for the wall drawing robot:
     ```sh
     git clone https://github.com/your-repo/wall-drawing-robot.git
     cd wall-drawing-robot
     ```

---

The software is now installed. Let's move on to writing the control code.

---
