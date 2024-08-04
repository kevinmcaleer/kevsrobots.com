---
layout: lesson
title: Setting Up the Raspberry Pi
author: Kevin McAleer
type: page
cover: assets/setup_pi.png
date: 2024-08-02
previous: 04_wiring.html
next: 06_software.html
description: Instructions to set up and configure the Raspberry Pi.
percent: 45
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


![Setup Pi](assets/pi_setup01.png){:class="cover"}

---

## Setting Up the Raspberry Pi

Setup the Pi for the Scrawly Wally robot:

Item | Action
--- | ---
**Install Raspberry Pi OS** Download and install the latest version of Raspberry Pi OS on a microSD card. Insert the microSD card into the Raspberry Pi and boot it up. <br /><br />**Connect to WiFi**: Follow the on-screen instructions to connect your Raspberry Pi to your WiFi network. <br /><br />**Update the System**: Open a terminal and run: `sudo apt-get update && sudo apt-get upgrade`<br /><br />**Enable I2C, SPI, and Serial Interfaces**: Run `sudo raspi-config` and enable the necessary interfaces. | ![Setup Pi](assets/pi_setup01.png){:class="w-100 img-fluid rounded-3 card-shadow card-hover"}
{:class="table table-striped"}

---

Your Raspberry Pi is now set up. Next, we'll install the software needed for the robot.

---
