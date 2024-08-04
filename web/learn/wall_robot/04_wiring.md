---
layout: lesson
title: Wiring the Components
author: Kevin McAleer
type: page
cover: assets/wiring.png
date: 2024-08-02
previous: 03_assembly.html
next: 05_setup_pi.html
description: Guide to wiring the stepper motors, servo, and Raspberry Pi.
percent: 40
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


![Wiring](assets/wiring.png){:class="cover"}

---

## Wiring

Follow these steps to wire the components:

1. **Connect the Stepper Motors**: 
   - Connect the 28BYJ-48 stepper motors to the ULN2003 driver boards.
   - Connect the IN1, IN2, IN3, and IN4 pins on the driver boards to GPIO pins on the Raspberry Pi.

2. **Connect the Servo Motor**:
   - Connect the servo motor to a PWM-capable GPIO pin on the Raspberry Pi.
   - Connect the power and ground wires of the servo to the Raspberry Pi.

3. **Power the Raspberry Pi**:
   - Ensure your Raspberry Pi has a stable power supply.

---

With the wiring done, we can now proceed to setting up the Raspberry Pi.

---
